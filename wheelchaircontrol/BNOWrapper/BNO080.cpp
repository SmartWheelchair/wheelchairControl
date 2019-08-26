//
// USC RPL BNO080 driver.
//
 
/*
 * Overview of BNO080 Communications
 * ===============================================
 *
 * Hilcrest has developed a protocol called SHTP (Sensor Hub Transport Protocol) for binary communications with
 * the BNO080 and the other IMUs it sells.  Over this protocol, SH-2 (Sensor Hub 2) messages are sent to configure
 * the chip and read data back.
 *
 * SHTP messages are divided at two hierarchical levels: first the channel, then the report ID.  Each category
 * of messages (system commands, sensor data reports, etc.) has its own channel, and the individual messages
 * in each channel are identified by their report id, which is the first byte of the message payload (note that the
 * datasheets don't *always* call the first byte the report ID, but that byte does identify the report, so I'm going
 * with it).
 *
 * ===============================================
 *
 * Information about the BNO080 is split into three datasheets.  Here's the download links and what they cover:
 *
 * - the BNO080 datasheet: http://www.hillcrestlabs.com/download/5a05f340566d07c196001ec1
 * -- Chip pinouts
 * -- Example circuits
 * -- Physical specifications
 * -- Supported reports and configuration settings (at a high level)
 * -- List of packets on the SHTP executable channel
 *
 * - the SHTP protocol: http://www.hillcrestlabs.com/download/59de8f99cd829e94dc0029d7
 * -- SHTP transmit and receive protcols (for SPI, I2C, and UART)
 * -- SHTP binary format
 * -- packet types on the SHTP command channel
 *
 * - the SH-2 reference: http://www.hillcrestlabs.com/download/59de8f398934bf6faa00293f
 * -- list of packets and their formats for all channels other than command and executable
 * -- list of FRS (Flash Record System) entries and their formats
 *
 * ===============================================
 *
 * Overview of SHTP channels:
 *
 * 0 -> Command
 * -- Used for protocol-global packets, currently only the advertisement packet (which lists all the channels) and error reports
 *
 * 1 -> Executable
 * -- Used for things that control the software on the chip: commands to reset and sleep
 * -- Also used by the chip to report when it's done booting up
 *
 * 2 -> Control
 * -- Used to send configuration commands to the IMU and for it to send back responses.
 * -- Common report IDs: Command Request (0xF2), Set Feature (0xFD)
 *
 * 3 -> Sensor Reports
 * -- Used for sensors to send back data reports.
 * -- AFAIK the only report ID on this channel will be 0xFB (Report Base Timestamp); sensor data is send in a series of structures
 *    following an 0xFB
 *
 * 4 -> Wake Sensor Reports
 * -- same as above, but for sensors configured to wake the device
 *
 * 5 -> Gyro Rotation Vector
 * -- a dedicated channel for the Gyro Rotation Vector sensor report
 * -- Why does this get its own channel?  I don't know!!!
 */
 
#include "BNO080.h"
#include "BNO080Constants.h"
/// Set to 1 to enable debug printouts.  Should be very useful if the chip is giving you trouble.
/// When debugging, it is recommended to use the highest possible serial baudrate so as not to interrupt the timing of operations.
#define BNO_DEBUG 0
 
BNO080::BNO080(Serial *debugPort, PinName user_SDApin, PinName user_SCLpin, PinName user_INTPin, PinName user_RSTPin,
               uint8_t i2cAddress, int i2cPortSpeed) :
    _debugPort(debugPort),
    _i2cPort(user_SDApin, user_SCLpin),
    _i2cAddress(i2cAddress),
    _int(user_INTPin),
    _rst(user_RSTPin, 1),
    commandSequenceNumber(0),
    stability(UNKNOWN),
    stepDetected(false),
    stepCount(0),
    significantMotionDetected(false),
    shakeDetected(false),
    xAxisShake(false),
    yAxisShake(false),
    zAxisShake(false)
{
    // zero sequence numbers
    memset(sequenceNumber, 0, sizeof(sequenceNumber));
 
    //Get user settings
    _i2cPortSpeed = i2cPortSpeed;
    if(_i2cPortSpeed > 4000000) {
        _i2cPortSpeed = 4000000; //BNO080 max is 400Khz
    }
    _i2cPort.frequency(_i2cPortSpeed);
    
 
}
 
bool BNO080::begin()
{
    //Configure the BNO080 for SPI communication
 
    _rst = 0; // Reset BNO080
    wait(.002f); // Min length not specified in datasheet?
    _rst = 1; // Bring out of reset
 
    // wait for a falling edge (NOT just a low) on the INT pin to denote startup
    Timer timeoutTimer;
 
    bool highDetected = false;
    bool lowDetected = false;
 
    while(true) {
        if(timeoutTimer.read() > BNO080_RESET_TIMEOUT) {
            _debugPort->printf("Error: BNO080 reset timed out, chip not detected.\n");
            return false;
        }
 
        // simple edge detector
        if(!highDetected) {
            if(_int == 1) {
                highDetected = true;
            }
        } else if(!lowDetected) {
            if(_int == 0) {
                lowDetected = true;
            }
        } else {
            // high and low detected
            break;
        }
    }
 
    _debugPort->printf("BNO080 detected!\n");
 
    // At system startup, the hub must send its full advertisement message (see SHTP 5.2 and 5.3) to the
    // host. It must not send any other data until this step is complete.
    // We don't actually care what's in it, we're just using it as a signal to indicate that the reset is complete.
    receivePacket();
 
    // now, after startup, the BNO will send an Unsolicited Initialize response (SH-2 section 6.4.5.2), and an Executable Reset command
    waitForPacket(CHANNEL_EXECUTABLE, EXECUTABLE_REPORTID_RESET);
 
    // Next, officially tell it to initialize, and wait for a successful Initialize Response
    zeroBuffer();
    //changed from sendCommand
    sendCommand(COMMAND_INITIALIZE);
    
    wait(0.02f);
    
    if(!waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_COMMAND_RESPONSE) || shtpData[2] != COMMAND_INITIALIZE || shtpData[5] != 0) {
        _debugPort->printf("BNO080 reports initialization failed.\n");
        __enable_irq();
        return false;
    } else {
#if BNO_DEBUG
        _debugPort->printf("BNO080 reports initialization successful!\n");
#endif
    }
 
 
    // Finally, we want to interrogate the device about its model and version.
    zeroBuffer();
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
    shtpData[1] = 0; //Reserved
    sendPacket(CHANNEL_CONTROL, 2);
 
    waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_PRODUCT_ID_RESPONSE, 5);
 
    if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
        majorSoftwareVersion = shtpData[2];
        minorSoftwareVersion = shtpData[3];
        patchSoftwareVersion = (shtpData[13] << 8) | shtpData[12];
        partNumber = (shtpData[7] << 24) | (shtpData[6] << 16) | (shtpData[5] << 8) | shtpData[4];
        buildNumber = (shtpData[11] << 24) | (shtpData[10] << 16) | (shtpData[9] << 8) | shtpData[8];
 
#if BNO_DEBUG
        _debugPort->printf("BNO080 reports as SW version %hhu.%hhu.%hu, build %lu, part no. %lu\n",
                           majorSoftwareVersion, minorSoftwareVersion, patchSoftwareVersion,
                           buildNumber, partNumber);
#endif
 
    } else {
        _debugPort->printf("Bad response from product ID command.\n");
        return false;
    }
 
    // successful init
    return true;
 
}
 
void BNO080::tare(bool zOnly)
{
    zeroBuffer();
 
    // from SH-2 section 6.4.4.1
    shtpData[3] = 0; // perform tare now
 
    if(zOnly) {
        shtpData[4] = 0b100; // tare Z axis
    } else {
        shtpData[4] = 0b111; // tare X, Y, and Z axes
    }
 
    shtpData[5] = 0; // reorient all motion outputs
 
    sendCommand(COMMAND_TARE);
}
 
bool BNO080::enableCalibration(bool calibrateAccel, bool calibrateGyro, bool calibrateMag)
{
    // send the Configure ME Calibration command
    zeroBuffer();
 
    shtpData[3] = static_cast<uint8_t>(calibrateAccel ? 1 : 0);
    shtpData[4] = static_cast<uint8_t>(calibrateGyro ? 1 : 0);
    shtpData[5] = static_cast<uint8_t>(calibrateMag ? 1 : 0);
 
    shtpData[6] = 0; // Configure ME Calibration command
 
    shtpData[7] = 0; // planar accelerometer calibration always disabled
 
    sendCommand(COMMAND_ME_CALIBRATE);
 
    // now, wait for the response
    if(!waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_COMMAND_RESPONSE)) {
#if BNO_DEBUG
        _debugPort->printf("Timeout waiting for calibration response!\n");
#endif
        return false;
    }
 
    if(shtpData[2] != COMMAND_ME_CALIBRATE) {
#if BNO_DEBUG
        _debugPort->printf("Received wrong response to calibration command!\n");
#endif
        return false;
    }
 
    if(shtpData[5] != 0) {
#if BNO_DEBUG
        _debugPort->printf("IMU reports calibrate command failed!\n");
#endif
        return false;
    }
 
    // acknowledge checks out!
    return true;
}
 
bool BNO080::saveCalibration()
{
    zeroBuffer();
 
    // no arguments
    sendCommand(COMMAND_SAVE_DCD);
 
    // now, wait for the response
    if(!waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_COMMAND_RESPONSE)) {
#if BNO_DEBUG
        _debugPort->printf("Timeout waiting for calibration response!\n");
#endif
        return false;
    }
 
    if(shtpData[2] != COMMAND_SAVE_DCD) {
#if BNO_DEBUG
        _debugPort->printf("Received wrong response to calibration command!\n");
#endif
        return false;
    }
 
    if(shtpData[5] != 0) {
#if BNO_DEBUG
        _debugPort->printf("IMU reports calibrate command failed!\n");
#endif
        return false;
    }
 
    // acknowledge checks out!
    return true;
}
 
void BNO080::setSensorOrientation(Quaternion orientation)
{
    zeroBuffer();
 
    _debugPort->printf("y: %f", orientation.y());
 
    // convert floats to Q
    int16_t Q_x = floatToQ(orientation.x(), ORIENTATION_QUAT_Q_POINT);
    int16_t Q_y = floatToQ(orientation.y(), ORIENTATION_QUAT_Q_POINT);
    int16_t Q_z = floatToQ(orientation.z(), ORIENTATION_QUAT_Q_POINT);
    int16_t Q_w = floatToQ(orientation.w(), ORIENTATION_QUAT_Q_POINT);
 
    _debugPort->printf("Q_y: %hd", Q_y);
 
    shtpData[3] = 2; // set reorientation
 
    shtpData[4] = static_cast<uint8_t>(Q_x & 0xFF); //P1 - X component LSB
    shtpData[5] = static_cast<uint8_t>(Q_x >> 8); //P2 - X component MSB
 
    shtpData[6] = static_cast<uint8_t>(Q_y & 0xFF); //P3 - Y component LSB
    shtpData[7] = static_cast<uint8_t>(Q_y >> 8); //P4 - Y component MSB
 
    shtpData[8] = static_cast<uint8_t>(Q_z & 0xFF); //P5 - Z component LSB
    shtpData[9] = static_cast<uint8_t>(Q_z >> 8); //P6 - Z component MSB
 
    shtpData[10] = static_cast<uint8_t>(Q_w & 0xFF); //P7 - W component LSB
    shtpData[11] = static_cast<uint8_t>(Q_w >> 8); //P8 - W component MSB
 
    //Using this shtpData packet, send a command
    sendCommand(COMMAND_TARE); // Send tare command
 
    // NOTE: unlike literally every other command, a sensor orientation command is never acknowledged in any way.
}
 
 
bool BNO080::updateData()
{
    if(_int.read() != 0) {
        // no waiting packets
        return false;
    }
 
    while(_int.read() == 0) {
        if(!receivePacket()) {
            // comms error
            return false;
        }
 
        processPacket();
        //wait(0.002f); //added
    }
 
    // packets were received, so data may have changed
    return true;
}
 
uint8_t BNO080::getReportStatus(Report report)
{
    uint8_t reportNum = static_cast<uint8_t>(report);
    if(reportNum > STATUS_ARRAY_LEN) {
        return 0;
    }
 
    return reportStatus[reportNum];
}
 
const char* BNO080::getReportStatusString(Report report)
{
    switch(getReportStatus(report)) {
        case 0:
            return "Unreliable";
        case 1:
            return "Accuracy Low";
        case 2:
            return "Accuracy Medium";
        case 3:
            return "Accuracy High";
        default:
            return "Error";
    }
}
 
bool BNO080::hasNewData(Report report)
{
    uint8_t reportNum = static_cast<uint8_t>(report);
    if(reportNum > STATUS_ARRAY_LEN) {
        return false;
    }
 
    bool newData = reportHasBeenUpdated[reportNum];
    reportHasBeenUpdated[reportNum] = false; // clear flag
    return newData;
}
 
//Sends the packet to enable the rotation vector
void BNO080::enableReport(Report report, uint16_t timeBetweenReports)
{
    // check time
    float periodSeconds = timeBetweenReports / 1000.0;
 
    if(periodSeconds < getMinPeriod(report)) {
        _debugPort->printf("Error: attempt made to set report 0x%02hhx to period of %.06f s, which is smaller than its min period of %.06f s.\n",
                           static_cast<uint8_t>(report), periodSeconds, getMinPeriod(report));
        return;
    }
    /*
    else if(getMaxPeriod(report) > 0 && periodSeconds > getMaxPeriod(report))
    {
        _debugPort->printf("Error: attempt made to set report 0x%02hhx to period of %.06f s, which is larger than its max period of %.06f s.\n",
                           static_cast<uint8_t>(report), periodSeconds, getMaxPeriod(report));
        return;
    }
    */
    setFeatureCommand(static_cast<uint8_t>(report), timeBetweenReports);
 
    // note: we don't wait for ACKs on these packets because they can take quite a while, like half a second, to come in
}
 
void BNO080::disableReport(Report report)
{
    // set the report's polling period to zero to disable it
    setFeatureCommand(static_cast<uint8_t>(report), 0);
}
 
uint32_t BNO080::getSerialNumber()
{
    uint32_t serNoBuffer;
 
    if(!readFRSRecord(FRS_RECORDID_SERIAL_NUMBER, &serNoBuffer, 1)) {
        return 0;
    }
 
    return serNoBuffer;
}
 
float BNO080::getRange(Report report)
{
    loadReportMetadata(report);
 
    return qToFloat_dword(metadataRecord[1], getQ1(report));
}
 
 
float BNO080::getResolution(Report report)
{
    loadReportMetadata(report);
 
    return qToFloat_dword(metadataRecord[2], getQ1(report));
}
 
float BNO080::getPower(Report report)
{
    loadReportMetadata(report);
 
    uint16_t powerQ = static_cast<uint16_t>(metadataRecord[3] & 0xFFFF);
 
    return qToFloat_dword(powerQ, POWER_Q_POINT);
}
 
float BNO080::getMinPeriod(Report report)
{
    loadReportMetadata(report);
 
    return metadataRecord[4] / 1e6f; // convert from microseconds to seconds
}
 
float BNO080::getMaxPeriod(Report report)
{
    loadReportMetadata(report);
 
    if(getMetaVersion() == 3) {
        // no max period entry in this record format
        return -1.0f;
    }
 
    return metadataRecord[9] / 1e6f; // convert from microseconds to seconds
}
 
void BNO080::printMetadataSummary(Report report)
{
#if BNO_DEBUG
    if(!loadReportMetadata(report)) {
        _debugPort->printf("Failed to load report metadata!\n");
    }
 
    _debugPort->printf("======= Metadata for report 0x%02hhx =======\n", static_cast<uint8_t>(report));
 
    _debugPort->printf("Range: +- %.04f units\n", getRange(report));
    _debugPort->printf("Resolution: %.04f units\n", getResolution(report));
    _debugPort->printf("Power Used: %.03f mA\n", getPower(report));
    _debugPort->printf("Min Period: %.06f s\n", getMinPeriod(report));
    _debugPort->printf("Max Period: %.06f s\n\n", getMaxPeriod(report));
 
#endif
}
 
int16_t BNO080::getQ1(Report report)
{
    loadReportMetadata(report);
 
    return static_cast<int16_t>(metadataRecord[7] & 0xFFFF);
}
 
int16_t BNO080::getQ2(Report report)
{
    loadReportMetadata(report);
 
    return static_cast<int16_t>(metadataRecord[7] >> 16);
}
 
int16_t BNO080::getQ3(Report report)
{
    loadReportMetadata(report);
 
    return static_cast<int16_t>(metadataRecord[8] >> 16);
}
 
void BNO080::processPacket()
{
    if(shtpHeader[2] == CHANNEL_CONTROL) {
        // currently no command reports are read
    } else if(shtpHeader[2] == CHANNEL_EXECUTABLE) {
        // currently no executable reports are read
    } else if(shtpHeader[2] == CHANNEL_COMMAND) {
 
    } else if(shtpHeader[2] == CHANNEL_REPORTS || shtpHeader[2] == CHANNEL_WAKE_REPORTS) {
        if(shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP) {
            parseSensorDataPacket();
            
        }
    }
}
 
// sizes of various sensor data packet elements
#define SIZEOF_BASE_TIMESTAMP 5
#define SIZEOF_TIMESTAMP_REBASE 5
#define SIZEOF_ACCELEROMETER 10
#define SIZEOF_LINEAR_ACCELERATION 10
#define SIZEOF_GYROSCOPE_CALIBRATED 10
#define SIZEOF_MAGNETIC_FIELD_CALIBRATED 10
#define SIZEOF_MAGNETIC_FIELD_UNCALIBRATED 16
#define SIZEOF_ROTATION_VECTOR 14
#define SIZEOF_GAME_ROTATION_VECTOR 12
#define SIZEOF_GEOMAGNETIC_ROTATION_VECTOR 14
#define SIZEOF_TAP_DETECTOR 5
#define SIZEOF_STABILITY_REPORT 6
#define SIZEOF_STEP_DETECTOR 8
#define SIZEOF_STEP_COUNTER 12
#define SIZEOF_SIGNIFICANT_MOTION 6
#define SIZEOF_SHAKE_DETECTOR 6
 
void BNO080::parseSensorDataPacket()
{
    size_t currReportOffset = 0;
 
    // every sensor data report first contains a timestamp offset to show how long it has been between when
    // the host interrupt was sent and when the packet was transmitted.
    // We don't use interrupts and don't care about times, so we can throw this out.
    currReportOffset += SIZEOF_BASE_TIMESTAMP;
 
    while(currReportOffset < packetLength) {
        if(currReportOffset >= STORED_PACKET_SIZE) {
            _debugPort->printf("Error: sensor report longer than packet buffer!\n");
            return;
        }
 
        // lots of sensor reports use 3 16-bit numbers stored in bytes 4 through 9
        // we can save some time by parsing those out here.
        uint16_t data1 = (uint16_t)shtpData[currReportOffset + 5] << 8 | shtpData[currReportOffset + 4];
        uint16_t data2 = (uint16_t)shtpData[currReportOffset + 7] << 8 | shtpData[currReportOffset + 6];
        uint16_t data3 = (uint16_t)shtpData[currReportOffset + 9] << 8 | shtpData[currReportOffset + 8];
 
        uint8_t reportNum = shtpData[currReportOffset];
 
        if(reportNum != SENSOR_REPORTID_TIMESTAMP_REBASE) {
            // set status from byte 2
            reportStatus[reportNum] = static_cast<uint8_t>(shtpData[currReportOffset + 2] & 0b11);
 
            // set updated flag
            reportHasBeenUpdated[reportNum] = true;
        }
 
        switch(shtpData[currReportOffset]) {
            case SENSOR_REPORTID_TIMESTAMP_REBASE:
                currReportOffset += SIZEOF_TIMESTAMP_REBASE;
                break;
 
            case SENSOR_REPORTID_ACCELEROMETER:
 
                totalAcceleration = TVector3(
                                        qToFloat(data1, ACCELEROMETER_Q_POINT),
                                        qToFloat(data2, ACCELEROMETER_Q_POINT),
                                        qToFloat(data3, ACCELEROMETER_Q_POINT));
 
                currReportOffset += SIZEOF_ACCELEROMETER;
                break;
 
            case SENSOR_REPORTID_LINEAR_ACCELERATION:
 
                linearAcceleration = TVector3(
                                         qToFloat(data1, ACCELEROMETER_Q_POINT),
                                         qToFloat(data2, ACCELEROMETER_Q_POINT),
                                         qToFloat(data3, ACCELEROMETER_Q_POINT));
 
                currReportOffset += SIZEOF_LINEAR_ACCELERATION;
                break;
 
            case SENSOR_REPORTID_GRAVITY:
 
                gravityAcceleration = TVector3(
                                          qToFloat(data1, ACCELEROMETER_Q_POINT),
                                          qToFloat(data2, ACCELEROMETER_Q_POINT),
                                          qToFloat(data3, ACCELEROMETER_Q_POINT));
 
                currReportOffset += SIZEOF_LINEAR_ACCELERATION;
                break;
 
            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
 
                gyroRotation = TVector3(
                                   qToFloat(data1, GYRO_Q_POINT),
                                   qToFloat(data2, GYRO_Q_POINT),
                                   qToFloat(data3, GYRO_Q_POINT));
 
                currReportOffset += SIZEOF_GYROSCOPE_CALIBRATED;
                break;
 
            case SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED:
 
                magField = TVector3(
                               qToFloat(data1, MAGNETOMETER_Q_POINT),
                               qToFloat(data2, MAGNETOMETER_Q_POINT),
                               qToFloat(data3, MAGNETOMETER_Q_POINT));
 
                currReportOffset += SIZEOF_MAGNETIC_FIELD_CALIBRATED;
                break;
 
            case SENSOR_REPORTID_MAGNETIC_FIELD_UNCALIBRATED: {
                magFieldUncalibrated = TVector3(
                                           qToFloat(data1, MAGNETOMETER_Q_POINT),
                                           qToFloat(data2, MAGNETOMETER_Q_POINT),
                                           qToFloat(data3, MAGNETOMETER_Q_POINT));
 
                uint16_t ironOffsetXQ = shtpData[currReportOffset + 11] << 8 | shtpData[currReportOffset + 10];
                uint16_t ironOffsetYQ = shtpData[currReportOffset + 13] << 8 | shtpData[currReportOffset + 12];
                uint16_t ironOffsetZQ = shtpData[currReportOffset + 15] << 8 | shtpData[currReportOffset + 14];
 
                hardIronOffset = TVector3(
                                     qToFloat(ironOffsetXQ, MAGNETOMETER_Q_POINT),
                                     qToFloat(ironOffsetYQ, MAGNETOMETER_Q_POINT),
                                     qToFloat(ironOffsetZQ, MAGNETOMETER_Q_POINT));
 
                currReportOffset += SIZEOF_MAGNETIC_FIELD_UNCALIBRATED;
            }
            break;
 
            case SENSOR_REPORTID_ROTATION_VECTOR: {
                uint16_t realPartQ = (uint16_t) shtpData[currReportOffset + 11] << 8 | shtpData[currReportOffset + 10];
                uint16_t accuracyQ = (uint16_t) shtpData[currReportOffset + 13] << 8 | shtpData[currReportOffset + 12];
 
                rotationVector = TVector4(
                                     qToFloat(data1, ROTATION_Q_POINT),
                                     qToFloat(data2, ROTATION_Q_POINT),
                                     qToFloat(data3, ROTATION_Q_POINT),
                                     qToFloat(realPartQ, ROTATION_Q_POINT));
 
                rotationAccuracy = qToFloat(accuracyQ, ROTATION_ACCURACY_Q_POINT);
 
                currReportOffset += SIZEOF_ROTATION_VECTOR;
            }
            break;
 
            case SENSOR_REPORTID_GAME_ROTATION_VECTOR: {
                uint16_t realPartQ = (uint16_t) shtpData[currReportOffset + 11] << 8 | shtpData[currReportOffset + 10];
 
                gameRotationVector = TVector4(
                                         qToFloat(data1, ROTATION_Q_POINT),
                                         qToFloat(data2, ROTATION_Q_POINT),
                                         qToFloat(data3, ROTATION_Q_POINT),
                                         qToFloat(realPartQ, ROTATION_Q_POINT));
 
                currReportOffset += SIZEOF_GAME_ROTATION_VECTOR;
            }
            break;
 
            case SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR: {
                uint16_t realPartQ = (uint16_t) shtpData[currReportOffset + 11] << 8 | shtpData[currReportOffset + 10];
                uint16_t accuracyQ = (uint16_t) shtpData[currReportOffset + 13] << 8 | shtpData[currReportOffset + 12];
 
                geomagneticRotationVector = TVector4(
                                                qToFloat(data1, ROTATION_Q_POINT),
                                                qToFloat(data2, ROTATION_Q_POINT),
                                                qToFloat(data3, ROTATION_Q_POINT),
                                                qToFloat(realPartQ, ROTATION_Q_POINT));
 
                geomagneticRotationAccuracy = qToFloat(accuracyQ, ROTATION_ACCURACY_Q_POINT);
 
                currReportOffset += SIZEOF_GEOMAGNETIC_ROTATION_VECTOR;
            }
            break;
 
            case SENSOR_REPORTID_TAP_DETECTOR:
 
                // since we got the report, a tap was detected
                tapDetected = true;
 
                doubleTap = (shtpData[currReportOffset + 4] & (1 << 6)) != 0;
 
                currReportOffset += SIZEOF_TAP_DETECTOR;
                break;
 
            case SENSOR_REPORTID_STABILITY_CLASSIFIER: {
                uint8_t classificationNumber = shtpData[currReportOffset + 4];
 
                if(classificationNumber > 4) {
                    classificationNumber = 0;
                }
 
                stability = static_cast<Stability>(classificationNumber);
 
                currReportOffset += SIZEOF_STABILITY_REPORT;
            }
            break;
 
            case SENSOR_REPORTID_STEP_DETECTOR:
 
                // the fact that we got the report means that a step was detected
                stepDetected = true;
 
                currReportOffset += SIZEOF_STEP_DETECTOR;
 
                break;
 
            case SENSOR_REPORTID_STEP_COUNTER:
 
                stepCount = shtpData[currReportOffset + 9] << 8 | shtpData[currReportOffset + 8];
 
                currReportOffset += SIZEOF_STEP_COUNTER;
 
                break;
 
            case SENSOR_REPORTID_SIGNIFICANT_MOTION:
 
                // the fact that we got the report means that significant motion was detected
                significantMotionDetected = true;
 
                currReportOffset += SIZEOF_SIGNIFICANT_MOTION;
 
            case SENSOR_REPORTID_SHAKE_DETECTOR:
 
                shakeDetected = true;
 
                xAxisShake = (shtpData[currReportOffset + 4] & 1) != 0;
                yAxisShake = (shtpData[currReportOffset + 4] & (1 << 1)) != 0;
                zAxisShake = (shtpData[currReportOffset + 4] & (1 << 2)) != 0;
 
                currReportOffset += SIZEOF_SHAKE_DETECTOR;
 
            default:
                _debugPort->printf("Error: unrecognized report ID in sensor report: %hhx.  Byte %u, length %hu\n", shtpData[currReportOffset], currReportOffset, packetLength);
                return;
        }
    }
 
}
 
bool BNO080::waitForPacket(int channel, uint8_t reportID, float timeout)
{
    Timer timeoutTimer;
    timeoutTimer.start();
 
    while(timeoutTimer.read() <= 2*timeout) {
        if(_int.read() == 0) {
            if(!receivePacket(timeout)) {
                return false;
            }
 
            if(channel == shtpHeader[2] && reportID == shtpData[0]) {
                // found correct packet!
                _debugPort->printf("\r\t found the correct packet \r\n");
                return true;
            } else {
                // other data packet, send to proper channels
                _debugPort->printf("\r\t other data packets, sending to proper channel\r\n");
                processPacket();
            }
        }
    }
 
    _debugPort->printf("Packet wait timeout.\n");
    return false;
}
 
//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO080::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
    float qFloat = fixedPointValue;
    qFloat *= pow(2.0, qPoint * -1.0);
    return (qFloat);
}
 
float BNO080::qToFloat_dword(uint32_t fixedPointValue, int16_t qPoint)
{
    float qFloat = fixedPointValue;
    qFloat *= pow(2.0, qPoint * -1.0);
    return (qFloat);
}
 
//Given a floating point value and a Q point, convert to Q
//See https://en.wikipedia.org/wiki/Q_(number_format)
int16_t BNO080::floatToQ(float qFloat, uint8_t qPoint)
{
    int16_t qVal = static_cast<int16_t>(qFloat * pow(2.0, qPoint));
    return qVal;
}
 
//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO080::sendCommand(uint8_t command)
{
    shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
    shtpData[1] = commandSequenceNumber++; //Increments automatically each function call
    shtpData[2] = command; //Command
 
    //Caller must set these
    shtpData[3] = 0; //P0
        shtpData[4] = 0; //P1
        shtpData[5] = 0; //P2
        shtpData[6] = 0;
        shtpData[7] = 0;
        shtpData[8] = 0;
        shtpData[9] = 0;
        shtpData[10] = 0;
        shtpData[11] = 0;
 
    //Transmit packet on channel 2, 12 bytes
    sendPacket(CHANNEL_CONTROL, 12);
}
 
//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
    uint32_t microsBetweenReports = static_cast<uint32_t>(timeBetweenReports * 1000);
 
    const uint32_t batchMicros = 0;
 
    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND; //Set feature command. Reference page 55
    shtpData[1] = reportID; //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    shtpData[2] = 0; //Feature flags
    shtpData[3] = 0; //Change sensitivity (LSB)
    shtpData[4] = 0; //Change sensitivity (MSB)
    shtpData[5] = (microsBetweenReports >> 0) & 0xFF; //Report interval (LSB) in microseconds. 0x7A120 = 500ms
    shtpData[6] = (microsBetweenReports >> 8) & 0xFF; //Report interval
    shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
    shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
    shtpData[9] = (batchMicros >> 0) & 0xFF;  //Batch Interval (LSB)
    shtpData[10] = (batchMicros >> 8) & 0xFF; //Batch Interval
    shtpData[11] = (batchMicros >> 16) & 0xFF;//Batch Interval
    shtpData[12] = (batchMicros >> 24) & 0xFF;//Batch Interval (MSB)
    shtpData[13] = (specificConfig >> 0) & 0xFF; //Sensor-specific config (LSB)
    shtpData[14] = (specificConfig >> 8) & 0xFF; //Sensor-specific config
    shtpData[15] = (specificConfig >> 16) & 0xFF; //Sensor-specific config
    shtpData[16] = (specificConfig >> 24) & 0xFF; //Sensor-specific config (MSB)
 
    //Transmit packet on channel 2, 17 bytes
    sendPacket(CHANNEL_CONTROL, 17);
}
 
bool BNO080::readFRSRecord(uint16_t recordID, uint32_t* readBuffer, uint16_t readLength)
{
    // send initial read request
    zeroBuffer();
 
    shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST;
    // read offset of 0 -> start at the start of the record
    shtpData[2] = 0;
    shtpData[3] = 0;
    // record ID
    shtpData[4] = static_cast<uint8_t>(recordID & 0xFF);
    shtpData[5] = static_cast<uint8_t>(recordID >> 8);
    // block size
    shtpData[6] = static_cast<uint8_t>(readLength & 0xFF);
    shtpData[7] = static_cast<uint8_t>(readLength >> 8);
 
    sendPacket(CHANNEL_CONTROL, 8);
 
    // now, read back the responses
    size_t readOffset = 0;
    while(readOffset < readLength) {
        if(!waitForPacket(CHANNEL_CONTROL, SHTP_REPORT_FRS_READ_RESPONSE)) {
#if BNO_DEBUG
            _debugPort->printf("Error: did not receive FRS read response after sending read request!\n");
#endif
            return false;
        }
 
        uint8_t status = static_cast<uint8_t>(shtpData[1] & 0b1111);
        uint8_t dataLength = shtpData[1] >> 4;
 
        // check status
        if(status == 1) {
#if BNO_DEBUG
            _debugPort->printf("Error: FRS reports invalid record ID!\n");
#endif
            return false;
        } else if(status == 2) {
#if BNO_DEBUG
            _debugPort->printf("Error: FRS is busy!\n");
#endif
            return false;
        } else if(status == 4) {
#if BNO_DEBUG
            _debugPort->printf("Error: FRS reports offset is out of range!\n");
#endif
            return false;
        } else if(status == 5) {
#if BNO_DEBUG
            _debugPort->printf("Error: FRS reports record %hx is empty!\n", recordID);
#endif
            return false;
        } else if(status == 8) {
#if BNO_DEBUG
            _debugPort->printf("Error: FRS reports flash memory device unavailable!\n");
#endif
            return false;
        }
 
        // check data length
        if(dataLength == 0) {
#if BNO_DEBUG
            _debugPort->printf("Error: Received FRS packet with 0 data length!\n");
#endif
            return false;
        } else if(dataLength == 1) {
            if(readOffset + 1 != readLength) {
#if BNO_DEBUG
                _debugPort->printf("Error: Received 1 length packet but more than 1 byte remains to be be read!\n");
#endif
                return false;
            }
        }
 
        // now, _finally_, read the dang words
        readBuffer[readOffset] = (shtpData[7] << 24) | (shtpData[6] << 16) | (shtpData[5] << 8) | (shtpData[4]);
 
        // check if we only wanted the first word
        ++readOffset;
        if(readOffset == readLength) {
            break;
        }
 
        readBuffer[readOffset] = (shtpData[11] << 24) | (shtpData[10] << 16) | (shtpData[9] << 8) | (shtpData[8]);
        readOffset++;
    }
 
    // read successful
    return true;
 
}
 
//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
bool BNO080::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
    
    uint16_t totalLength = dataLength + 4; //Add four bytes for the header
    packetLength = dataLength;
 
    shtpHeader[0] = totalLength & 0xFF;
    shtpHeader[1] = totalLength >> 8;
    shtpHeader[2] = channelNumber;
    shtpHeader[3] = sequenceNumber[channelNumber]++;
#if BNO_DEBUG
 
    _debugPort->printf("Transmitting packet: ----------------\n");
    printPacket();
#endif
    
    readBuffer[0] = shtpHeader[0];
    readBuffer[1] = shtpHeader[1];
    readBuffer[2] = shtpHeader[2];
    readBuffer[3] = shtpHeader[3];
    
    for(size_t index = 0; index < dataLength; ++index)
    {
        readBuffer[index + 4] = shtpData[index];
    }
    
    int writeRetval = _i2cPort.write(
        _i2cAddress << 1,
        reinterpret_cast<char*>(readBuffer),
        totalLength);
        
    if(writeRetval < 0) 
    {
        _debugPort->printf("BNO I2C body write failed!\n");
        return false;
    }
    
   
    
    return (true);
}
 
//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
bool BNO080::receivePacket(float timeout)
{
    Timer waitStartTime;
    waitStartTime.start();
 
    while(_int.read() != 0) {
        if(waitStartTime.read() > timeout) {
            _debugPort->printf("BNO I2C wait timeout\n");
            return false;
        }
    }
    
    const size_t headerLen = 4;
    uint8_t headerData[headerLen];
    int readRetval = _i2cPort.read(
        (_i2cAddress << 1) | 0x1,
        reinterpret_cast<char*>(headerData),
        headerLen);
        
    if(readRetval < 0) 
    {
        _debugPort->printf("BNO I2C header read failed!\n");
        return false;
    }
 
 
    //Get the first four bytes, aka the packet header
    uint8_t packetLSB = headerData[0];
    uint8_t packetMSB = headerData[1];
    uint8_t channelNumber = headerData[2];
    uint8_t sequenceNum = headerData[3]; //Not sure if we need to store this or not
 
    //Store the header info
    shtpHeader[0] = packetLSB;
    shtpHeader[1] = packetMSB;
    shtpHeader[2] = channelNumber;
    shtpHeader[3] = sequenceNum;
 
    if(shtpHeader[0] == 0xFF && shtpHeader[1] == 0xFF) {
        // invalid according to BNO080 datasheet section 1.4.1
 
        _debugPort->printf("Received 0xFFFF packet length, protocol error!\n");
        return false;
    }
 
    //Calculate the number of data bytes in this packet
    packetLength = (static_cast<uint16_t>(packetMSB) << 8 | packetLSB);
 
    // Clear the MSbit.
    // This bit indicates if this package is a continuation of the last. TBH, I don't really know what this means (it's not really explained in the datasheet)
    // but we don't actually care about any of the advertisement packets
    // that use this, so we can just cut off the rest of the packet by releasing chip select.
    packetLength &= ~(1 << 15);
 
    if (packetLength == 0) {
        // Packet is empty
        return (false); //All done
    }
    else if(packetLength > READ_BUFFER_SIZE)
    {
        return false; // read buffer too small
    }
 
    packetLength -= headerLen; //Remove the header bytes from the data count
    wait(.003);
    readRetval = _i2cPort.read(
        (_i2cAddress << 1) | 0x1,
        reinterpret_cast<char*>(readBuffer),
        packetLength + headerLen,
        false);
        
    if(readRetval < 0) 
    {
        _debugPort->printf("BNO I2C body read failed!\n");
        return false;
    }
 
    //Read incoming data into the shtpData array
    for (uint16_t dataSpot = 0 ; dataSpot < packetLength ; dataSpot++) {
 
        if (dataSpot < STORED_PACKET_SIZE) //BNO080 can respond with upto 270 bytes, avoid overflow
            shtpData[dataSpot] = readBuffer[dataSpot + headerLen]; //Store data into the shtpData array
    }
 
#if BNO_DEBUG
    _debugPort->printf("Received packet: ----------------\n");
    printPacket(); // note: add 4 for the header length
#endif
    return (true); //We're done!
}
 
//Pretty prints the contents of the current shtp header and data packets
void BNO080::printPacket()
{
#if BNO_DEBUG
    //Print the four byte header
    _debugPort->printf("Header:");
    for (uint8_t x = 0 ; x < 4 ; x++) {
        _debugPort->printf(" ");
        if (shtpHeader[x] < 0x10) _debugPort->printf("0");
        _debugPort->printf("%hhx", shtpHeader[x]);
    }
 
    uint16_t printLength = packetLength;
    if (printLength > 40) printLength = 40; //Artificial limit. We don't want the phone book.
 
    _debugPort->printf(" Body:");
    for (uint16_t x = 0 ; x < printLength ; x++) {
        _debugPort->printf(" ");
        if (shtpData[x] < 0x10) _debugPort->printf("0");
        _debugPort->printf("%hhx", shtpData[x]);
    }
 
    _debugPort->printf(", Length:");
    _debugPort->printf("%hhu", packetLength + SHTP_HEADER_SIZE);
 
    if(shtpHeader[1] >> 7) {
        _debugPort->printf("[C]");
    }
 
    _debugPort->printf(", SeqNum: %hhu", shtpHeader[3]);
 
    _debugPort->printf(", Channel:");
    if (shtpHeader[2] == 0) _debugPort->printf("Command");
    else if (shtpHeader[2] == 1) _debugPort->printf("Executable");
    else if (shtpHeader[2] == 2) _debugPort->printf("Control");
    else if (shtpHeader[2] == 3) _debugPort->printf("Sensor-report");
    else if (shtpHeader[2] == 4) _debugPort->printf("Wake-report");
    else if (shtpHeader[2] == 5) _debugPort->printf("Gyro-vector");
    else _debugPort->printf("%hhu", shtpHeader[2]);
 
    _debugPort->printf("\n");
#endif
}
 
 
void BNO080::zeroBuffer()
{
    memset(shtpHeader, 0, SHTP_HEADER_SIZE);
    memset(shtpData, 0, STORED_PACKET_SIZE);
    packetLength = 0;
}
 
bool BNO080::loadReportMetadata(BNO080::Report report)
{
    uint16_t reportMetaRecord;
    
    // first, convert the report into the correct FRS record ID for that report's metadata
    // data from SH-2 section 5.1
    switch(report) {
        case TOTAL_ACCELERATION:
            reportMetaRecord = 0xE301;
            break;
        case LINEAR_ACCELERATION:
            reportMetaRecord = 0xE303;
            break;
        case GRAVITY_ACCELERATION:
            reportMetaRecord = 0xE304;
            break;
        case GYROSCOPE:
            reportMetaRecord = 0xE306;
            break;
        case MAG_FIELD:
            reportMetaRecord = 0xE309;
            break;
        case MAG_FIELD_UNCALIBRATED:
            reportMetaRecord = 0xE30A;
            break;
        case ROTATION:
            reportMetaRecord = 0xE30B;
            break;
        case GEOMAGNETIC_ROTATION:
            reportMetaRecord = 0xE30D;
            break;
        case GAME_ROTATION:
            reportMetaRecord = 0xE30C;
            break;
        case TAP_DETECTOR:
            reportMetaRecord = 0xE313;
            break;
        case STABILITY_CLASSIFIER:
            reportMetaRecord = 0xE317;
            break;
        case STEP_DETECTOR:
            reportMetaRecord = 0xE314;
            break;
        case STEP_COUNTER:
            reportMetaRecord = 0xE315;
            break;
        case SIGNIFICANT_MOTION:
            reportMetaRecord = 0xE316;
            break;
        case SHAKE_DETECTOR:
            reportMetaRecord = 0xE318;
            break;
    }
 
    // if we already have that data stored, everything's OK
    if(bufferMetadataRecord == reportMetaRecord) {
        return true;
    }
 
    // now, load the metadata into the buffer
    if(!readFRSRecord(reportMetaRecord, metadataRecord, METADATA_BUFFER_LEN)) {
        // clear this so future calls won't try to use the cached version
        bufferMetadataRecord = 0;
 
        return false;
    }
 
    bufferMetadataRecord = reportMetaRecord;
 
    return true;
}
