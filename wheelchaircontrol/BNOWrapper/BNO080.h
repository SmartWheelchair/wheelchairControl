/*
 * This is USC RPL's ARM MBed BNO080 IMU driver, by Jamie Smith.
 *
 * It is based on SparkFun and Nathan Seidle's Arduino driver for this chip, but is substantially rewritten and adapted.
 * It also supports some extra features, such as setting the mounting orientation and
 * enabling some additional data reports.
 *
 * This driver uses no dynamic allocation, but does allocate a couple hundred bytes of class variables as buffers.
 * This should allow you to monitor its memory usage using MBed's size printout.
 *
 * The BNO080 is a very complex chip; it's capable of monitoring and controlling other sensors and making
 * intelligent decisions and calculations using its data.  Accordingly, the protocol for communicating with it
 * is quite complex, and it took me quite a while to wrap my head around it.  If you need to modify or debug
 * this driver, look at the CPP file for an overview of the chip's communication protocol.
 *
 * Note: this driver only supports I2C.  I attempted to create an SPI version, but as far as I can tell,
 * the BNO's SPI interface has a bug that causes you to be unable to wake the chip from sleep in some conditions.
 * Until this is fixed, SPI on it is virtually unusable.
 */

#ifndef HAMSTER_BNO080_H
#define HAMSTER_BNO080_H

#include <mbed.h>
#include <quaternion.h>


#include "BNO080Constants.h"

// useful define when working with orientation quaternions
#define SQRT_2 1.414213562f

/**
  Class to drive the BNO080 9-axis IMU.
  
  There should be one instance of this class per IMU chip. I2C address and pin assignments are passed in the constructor.
*/
class BNO080
{
	/**
	 * Serial stream to print debug info to.  Used for errors, and debugging output if debugging is enabled.
	 */
	Serial * _debugPort;

	/**
	 * I2C port object.  Provides physical layer communications with the chip.
	 */
	 
	I2C _i2cPort;
	//SoftI2C _i2cPort;
	
	/// user defined port speed
	int  _i2cPortSpeed;

	/// i2c address of IMU (7 bits)
	uint8_t _i2cAddress;

	/// Interrupt pin -- signals to the host that the IMU has data to send
	DigitalIn _int;
	
	// Reset pin -- resets IMU when held low.
	DigitalOut _rst;

	// packet storage
	//-----------------------------------------------------------------------------------------------------------------

#define SHTP_HEADER_SIZE 4

	// Arbitrarily chosen, but should hopefully be large enough for all packets we need.
	// If you enable lots of sensor reports and get an error, you might need to increase this.
#define STORED_PACKET_SIZE 128 

	/// Each SHTP packet has a header of 4 uint8_ts
	uint8_t shtpHeader[SHTP_HEADER_SIZE];

	/// Stores data contained in each packet.  Packets can contain an arbitrary amount of data, but 
	/// rarely get over a hundred bytes unless you have a million sensor reports enabled.
	/// The only long packets we actually care about are batched sensor data packets.
	uint8_t shtpData[STORED_PACKET_SIZE];
	
	#define READ_BUFFER_SIZE 512
	uint8_t readBuffer[READ_BUFFER_SIZE];

	/// Length of packet that was received into buffer.  Does NOT include header bytes.
	uint16_t packetLength;

	/// Current sequence number for each channel, incremented after transmission. 
	uint8_t sequenceNumber[6];

	/// Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
	uint8_t commandSequenceNumber;


	// frs metadata
	//-----------------------------------------------------------------------------------------------------------------

	/// Record ID of the metadata record currently stored in the metadataRecord[] buffer.
	/// Used so that we can avoid requerying the FRS record if we need to make multiple metadata reads
	/// in succession.
	uint16_t bufferMetadataRecord;

	/// currently we only need the first 10 words of the metadata
#define METADATA_BUFFER_LEN 10

	/// Buffer for current metadata record.
	uint32_t metadataRecord[METADATA_BUFFER_LEN];

	// data storage
	//-----------------------------------------------------------------------------------------------------------------

	// 1 larger than the largest sensor report ID
#define STATUS_ARRAY_LEN MAX_SENSOR_REPORTID + 1

	/// stores status of each sensor, indexed by report ID
	uint8_t reportStatus[STATUS_ARRAY_LEN];

	/// stores whether a sensor has been updated since the last call to hasNewData()
	bool reportHasBeenUpdated[STATUS_ARRAY_LEN];

public:

	// list of reports
	//-----------------------------------------------------------------------------------------------------------------

	/// List of all sensor reports that the IMU supports.
	enum Report
	{
		/**
		 * Total acceleration of the IMU in world space.
		 * See BNO datasheet section 2.1.1
		 */
		TOTAL_ACCELERATION = SENSOR_REPORTID_ACCELEROMETER,

		/**
		 * Acceleration of the IMU not including the acceleration of gravity.
		 * See BNO datasheet section 2.1.1
		 */
		LINEAR_ACCELERATION = SENSOR_REPORTID_LINEAR_ACCELERATION,

		/**
		 * Acceleration of gravity felt by the IMU.
		 * See BNO datasheet section 2.1.1
		 */
		GRAVITY_ACCELERATION = SENSOR_REPORTID_GRAVITY,

		/**
		 * (calibrated) gyroscope reading of the rotational speed of the IMU.
		 * See BNO datasheet section 2.1.2
		 */
		GYROSCOPE = SENSOR_REPORTID_GYROSCOPE_CALIBRATED,

		/**
		 * (calibrated) reading of Earth's magnetic field levels.
		 * See BNO datasheet section 2.1.3
		 */
		MAG_FIELD = SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED,

		/**
		* Uncalibrated reading of magnetic field levels, without any hard iron offsets applied
		* See BNO datasheet section 2.1.3
		*/
		MAG_FIELD_UNCALIBRATED = SENSOR_REPORTID_MAGNETIC_FIELD_UNCALIBRATED,

		/**
		 * Fused reading of the IMU's rotation in space using all three sensors.  This is the most accurate reading
		 * of absolute orientation that the IMU can provide.
		 * See BNO datasheet section 2.2.4
		 */
		ROTATION = SENSOR_REPORTID_ROTATION_VECTOR,

		/**
		 * Fused reading of rotation from accelerometer and magnetometer readings.  This report is designed to decrease
		 * power consumption (by turning off the gyroscope) in exchange for reduced responsiveness.
		 */
		GEOMAGNETIC_ROTATION = SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR,

		/**
		 * Fused reading of the IMU's rotation in space.  Unlike the regular rotation vector, the Game Rotation Vector
	 	 * is not referenced against the magnetic field and the "zero yaw" point is arbitrary.
	 	 * See BNO datasheet section 2.2.2
		 */
		GAME_ROTATION = SENSOR_REPORTID_GAME_ROTATION_VECTOR,

		/**
		 * Detects a user tapping on the device containing the IMU.
		 * See BNO datasheet section 2.4.2
		 */
		TAP_DETECTOR = SENSOR_REPORTID_TAP_DETECTOR,

		/**
		 * Detects whether the device is on a table, being held stably, or being moved.
		 * See BNO datasheet section 2.4.1
		 */
		STABILITY_CLASSIFIER = SENSOR_REPORTID_STABILITY_CLASSIFIER,

		/**
		 * Detects a user taking a step with the IMU worn on their person.
		 * See BNO datasheet section 2.4.3
		 */
		STEP_DETECTOR = SENSOR_REPORTID_STEP_DETECTOR,

		/**
		 * Detects how many steps a user has taken.
		 * See BNO datasheet section 2.4.4
		 */
		STEP_COUNTER = SENSOR_REPORTID_STEP_COUNTER,

		/**
		 * Detects when the IMU has made a "significant" motion, defined as moving a few steps and/or accelerating significantly.
		 *
		 * NOTE: this report automatically disables itself after sending a report, so you'll have to reenable it each time a motion i s detected.
		 * See BNO datasheet section 2.4.6
		 */
	 	SIGNIFICANT_MOTION = SENSOR_REPORTID_SIGNIFICANT_MOTION,

		/**
		 * Detects when the IMU is being shaken.
		 * See BNO datasheet section 2.4.7
		 */
		SHAKE_DETECTOR = SENSOR_REPORTID_SHAKE_DETECTOR
	};

	// data variables to read reports from
	//-----------------------------------------------------------------------------------------------------------------

	// @{
	/// Version info read from the IMU when it starts up
	uint8_t majorSoftwareVersion;
	uint8_t minorSoftwareVersion;
	uint16_t patchSoftwareVersion;
	uint32_t partNumber;
	uint32_t buildNumber;
	// @}


	/**
	 * Readout from Accleration report.
	 * Represents total acceleration in m/s^2 felt by the BNO's accelerometer.
	 */
	TVector3 totalAcceleration;

	/**
	 * Readout from Linear Acceleration report.
	 * Represents acceleration felt in m/s^2 by the BNO's accelerometer not including the force of gravity.
	 */
	TVector3 linearAcceleration;

	/**
	 * Readout from Gravity report.
	 * Represents the force of gravity in m/s^2 felt by the BNO's accelerometer.
	 */
	TVector3 gravityAcceleration;

	/**
	 * Readout from Calibrated Gyroscope report
	 * Represents the angular velocities of the chip in rad/s in the X, Y, and Z axes
	 */
	TVector3 gyroRotation;

	/**
	 * Readout from the Magnetic Field Calibrated report.
	 * Represents the magnetic field read by the chip in uT in the X, Y, and Z axes
	 */
	TVector3 magField;

	/**
	 * Readout from the Magnetic Field Uncalibrated report.
	 * Represents the magnetic field read by the chip in uT in the X, Y, and Z axes, without hard iron offsets applied
	 */
	TVector3 magFieldUncalibrated;

	/**
	 * Auxiliary readout from the Magnetic Field Uncalibrated report.
	 * Represents the hard iron offsets that the chip is using in each axis in uT.
	 */
	TVector3 hardIronOffset;

	/**
	 * Readout from the Rotation Vector report.
	 * Represents the rotation of the IMU (relative to magnetic north) in radians.
	 */
	Quaternion rotationVector;

	/**
	 * Auxiliary accuracy readout from the Rotation Vector report.
	 * Represents the estimated accuracy of the rotation vector in radians.
	 */
	float rotationAccuracy;

	/**
	 * Readout from the Game Rotation Vector report.
	 * Represents the rotation of the IMU in radians.  Unlike the regular rotation vector, the Game Rotation Vector
	 * is not referenced against the magnetic field and the "zero yaw" point is arbitrary.
	 */
	Quaternion gameRotationVector;

	/**
	 * Readout from the Geomagnetic Rotation Vector report.
	 * Represents the geomagnetic rotation of the IMU (relative to magnetic north) in radians.
	 */
	Quaternion geomagneticRotationVector;

	/**
	 * Auxiliary accuracy readout from the Geomagnetic Rotation Vector report.
	 * Represents the estimated accuracy of the rotation vector in radians.
	 */
	float geomagneticRotationAccuracy;

	/**
	 * Tap readout from the Tap Detector report.  This flag is set to true whenever a tap is detected, and you should
	 * manually clear it when you have processed the tap.
	 */
	bool tapDetected;

	/**
	 * Whether the last tap detected was a single or double tap.
	 */
	bool doubleTap;

	/**
	 * Enum to represent the different stability types.
	 *
	 * See BNO datasheet section 2.4.1 and SH-2 section 6.5.31.2 for details.
	 */
	enum Stability
	{
		/// Unknown stability type.
		UNKNOWN = 0,

		/// At rest on a stable surface with very little motion
		ON_TABLE = 1,

		/// Motion is stable, but the duration requirement for stability has not been met.
		/// Can only occur during gyroscope calibration (why? beats me!)
		STATIONARY = 2,

		/// Stable (has been below the acceleration threshold for the required duration)
		STABLE = 3,

		/// IMU is moving.
		MOTION = 4
	};

	/**
	 * Readout from the stability classifier.
	 * Current stability status of the IMU.
	 */
	Stability stability;

	/**
	 * Readout from the Step Detector report.  This flag is set to true whenever a step is detected, and you should
	 * manually clear it when you have processed the step.
	 */
	bool stepDetected;

	/**
	 * Readout from the Step Counter report.  This count increases as the user takes steps, but can also decrease
	 * if the IMU decides that a motion was not a step.
	 */
	uint16_t stepCount;

	/**
	 * Readout from the Significant Motion Detector report.  This flag is set to true whenever significant motion is detected, and you should
	 * manually clear it when you have processed the event.
	 */
	bool significantMotionDetected;

	/**
	 * Readout from the Shake Detector report.  This flag is set to true whenever shaking is detected, and you should
	 * manually clear it when you have processed the event.
	 */
	bool shakeDetected;

	// @{
	/// The axis/axes that shaking was detected in in the latest shaking report.
	bool xAxisShake;
	bool yAxisShake;
	bool zAxisShake;
	// @}

	// Management functions
	//-----------------------------------------------------------------------------------------------------------------

	/**
	 * Construct a BNO080, providing pins and parameters.
	 *
	 * This doesn't actally initialize the chip, you will need to call begin() for that.
	 *
	 * NOTE: while some schematics tell you to connect the BOOTN pin to the processor, this driver does not use or require it.
	 * Just tie it to VCC per the datasheet.
	 *
	 * @param debugPort Serial port to write output to.  Cannot be nullptr.
	 * @param user_SDApin Hardware I2C SDA pin connected to the IMU
	 * @param user_SCLpin Hardware I2C SCL pin connected to the IMU
	 * @param user_INTPin Input pin connected to HINTN
	 * @param user_RSTPin Output pin connected to NRST
	 * @param i2cAddress I2C address.  The BNO defaults to 0x4a, but can also be set to 0x4b via a pin.
	 * @param i2cPortSpeed I2C frequency.  The BNO's max is 400kHz.
	 */
	BNO080(Serial *debugPort, 
	       PinName user_SDApin, 
		   PinName user_SCLpin, 
		   PinName user_INTPin, 
		   PinName user_RSTPin,
		   uint8_t i2cAddress=0x4a, 
		   int i2cPortSpeed=100000);

	/**
	 * Resets and connects to the IMU.  Verifies that it's connected, and reads out its version
	 * info into the class variables above.
	 *
	 * If this function is failing, it would be a good idea to turn on BNO_DEBUG in the cpp file to get detailed output.
	 *
	 * @return whether or not initialization was successful
	 */
	bool begin();

	/**
	 * Tells the IMU to use its current rotation vector as the "zero" rotation vector and to reorient
	 * all outputs accordingly.
	 *
	 * @param zOnly If true, only the rotation about the Z axis (the heading) will be tared.
	 */
	void tare(bool zOnly = false);

	/**
	 * Tells the IMU to begin a dynamic sensor calibration.  To calibrate the IMU, call this function and move
	 * the IMU according to the instructions in the "BNO080 Sensor Calibration Procedure" app note
	 * (http://www.hillcrestlabs.com/download/59de9014566d0727bd002ae7).
	 *
	 * To tell when the calibration is complete, look at the status bits for Game Rotation Vector (for accel and gyro)
	 * and Magnetic Field (for the magnetometer).
	 *
	 * The gyro and accelerometer should only need to be calibrated once, but the magnetometer will need to be recalibrated
	 * every time the orientation of ferrous metals around the IMU changes (e.g. when it is put into a new enclosure).
	 *
	 * The new calibration will not be saved in flash until you call saveCalibration().
	 *
	 * NOTE: calling this with all false values will cancel any calibration in progress.  However, the calibration data being created will
	 * remain in use until the next chip reset (I think!)
	 *
	 * @param calibrateAccel Whether to calibrate the accelerometer.
	 * @param calibrateGyro Whether to calibrate the gyro.
	 * @param calibrateMag Whether to calibrate the magnetometer.
	 *
	 * @return whether the operation succeeded
	 */
	bool enableCalibration(bool calibrateAccel, bool calibrateGyro, bool calibrateMag);

	/**
	 * Saves the calibration started with startCalibration() and ends the calibration.
	 * You will want to call this once the status bits read as "accuracy high".
	 *
	 * WARNING: if you paid for a factory calibrated IMU, then this WILL OVERWRITE THE FACTORY CALIBRATION in whatever sensors
	 * are being calibrated.  Use with caution!
	 *
	 * @return whether the operation succeeded
	 */
	bool saveCalibration();

	/**
	 * Sets the orientation quaternion, telling the sensor how it's mounted
	 * in relation to world space.
	 * See page 40 of the BNO080 datasheet.
	 *
	 * NOTE: this driver provides the macro SQRT_2 to help with entering values from that table.
	 *
	 * NOTE 2: this setting does not persist and will have to be re-applied every time the chip is reset.
	 * Use setPermanentOrientation() for that.
	 *
	 * @param orientation quaternion mapping from IMU space to world space.
	 */
	void setSensorOrientation(Quaternion orientation);

	/**
	 * Sets the orientation quaternion, telling the sensor how it's mounted
	 * in relation to world space. See page 40 of the BNO080 datasheet.
	 *
	 * Unlike setSensorOrientation(), this setting will persist across sensor restarts.
	 * However, it will also take a few hundred milliseconds to write.
	 *
	 * @param orientation quaternion mapping from IMU space to world space.
	 *
	 * @return true if the operation succeeded, false if it failed.
 	*/
	bool setPermanentOrientation(Quaternion orientation);
    
	// Report functions
	//-----------------------------------------------------------------------------------------------------------------

	/**
	 * Checks for new data packets queued on the IMU.
	 * If there are packets queued, receives all of them and updates
	 * the class variables with the results.
	 *
	 * @return True iff new data packets of any kind were received.  If you need more fine-grained data change reporting,
	 * check out hasNewData().
	 */
	bool updateData();


	/**
	 * Gets the status of a report as a 2 bit number.
	 * per SH-2 section 6.5.1, this is interpreted as: <br>
	 * 0 - unreliable <br>
	 * 1 - accuracy low <br>
	 * 2 - accuracy medium <br>
	 * 3 - accuracy high <br>
	 * of course, these are only updated if a given report is enabled.
	 * @param report
	 * @return
	 */
	uint8_t getReportStatus(Report report);

	/**
	 * Get a string for printout describing the status of a sensor.
	 * @return
	 */
	const char* getReportStatusString(Report report);

	/**
	 * Checks if a specific report has gotten new data since the last call to this function.
	 * @param report The report to check.
	 * @return Whether the report has received new data.
	 */
	bool hasNewData(Report report);

	/**
	 * Enable a data report from the IMU.  Look at the comments above to see what the reports do.
	 * This function checks your polling period against the report's max speed in the IMU's metadata,
	 * and reports an error if you're trying to poll too fast.
	 *
	 * @param timeBetweenReports time in milliseconds between data updates.
	 */
	void enableReport(Report report, uint16_t timeBetweenReports);

	/**
	 * Disable a data report from the IMU.
	 *
	 * @param report The report to disable.
	 */
	void disableReport(Report report);

	/**
	 * Gets the serial number (used to uniquely identify each individual device).
	 *
	 * NOTE: this function should work according to the datasheet, but the device I was testing with appears to have
	 * an empty serial number record as shipped, and I could never get anything out of it. Your mileage may vary.
	 *
	 * @return The serial number, or 0 on error.
	 */
	uint32_t getSerialNumber();

	// Metadata functions
	//-----------------------------------------------------------------------------------------------------------------

	/**
	 * Gets the range of a report as reported by the IMU.   Units are the same as the report's output data.
	 * @return
	 */
	float getRange(Report report);

	/**
	 * Gets the resolution of a report as reported by the IMU.  Units are the same as the report's output data.
	 * @param report
	 * @return
	 */
	float getResolution(Report report);

	/**
	 * Get the power used by a report when it's operating, according to the IMU.
	 * @param report
	 * @return Power used in mA.
	 */
	float getPower(Report report);

	/**
	 * Gets the smallest polling period that a report supports.
	 * @return Period in seconds.
	 */
	float getMinPeriod(Report report);

	/**
	 * Gets the larges polling period that a report supports.
	 * Some reports don't have a max period, in which case this function will return -1.0.
	 *
	 * @return Period in seconds, or -1.0 on error.
	 */
	float getMaxPeriod(Report report);

	/**
	 * Prints a summary of a report's metadata to the
	 * debug stream.  Should be useful for debugging and setting up reports since lots of this data
	 * isn't given in the datasheets.
	 *
	 * Note: to save string constant space, this function is only available when BNO_DEBUG is 1.
	 */
	void printMetadataSummary(Report report);

private:

	// Internal metadata functions
	//-----------------------------------------------------------------------------------------------------------------

	/**
	 * Gets the version of the metadata stored in the buffer.
	 * We might see version 3 and 4 records, and they have different layouts.
	 * @return
	 */
	uint16_t getMetaVersion() {return static_cast<uint16_t>(metadataRecord[3] >> 16);}

	// @{
	/**
	 * Gets the Q point from a report's metadata, which essentially defines where the decimal point goes in the sensor's output.
	 * The 1/2/3 Q values are used in different places in the metadata, see SH-2 section 5.1 for details.
	 * @param report
	 * @return
	 */
	int16_t getQ1(Report report);
	int16_t getQ2(Report report);
	int16_t getQ3(Report report);
	// @}

	// internal utility functions
	//-----------------------------------------------------------------------------------------------------------------

	/**
	 * Processes the packet currently stored in the buffer, and updates class variables to reflect the data it contains
	 */
	void processPacket();

	/**
	 * Processes the sensor data packet currently stored in the buffer.
	 * Only called from processPacket()
	 */
	void parseSensorDataPacket();

	/**
	 * Call to wait for a packet with the given parameters to come in.
	 *
	 * @param channel Channel of the packet
	 * @param reportID Report ID (first data byte) of the packet
	 * @param timeout how long to wait for the packet
	 * @return true if the packet has been received, false if it timed out
	 */
	bool waitForPacket(int channel, uint8_t reportID, float timeout = .125f);

	/**
	 * Given a Q value, converts fixed point floating to regular floating point number.
	 * @param fixedPointValue
	 * @param qPoint
	 * @return
	 */
	float qToFloat(int16_t fixedPointValue, uint8_t qPoint);

	/**
	 * Given a Q value, converts fixed point floating to regular floating point number.
	 * This version is used for the unsigned 32-bit values in metadata records.
	 * @param fixedPointValue
	 * @param qPoint
	 * @return
	 */
	float qToFloat_dword(uint32_t fixedPointValue, int16_t qPoint);

	/**
	 * Given a floating point value and a Q point, convert to Q
 	 * See https://en.wikipedia.org/wiki/Q_(number_format)
	 * @param qFloat
	 * @param qPoint
	 * @return
	 */
	int16_t floatToQ(float qFloat, uint8_t qPoint);

	/**
	 * Given a floating point value and a Q point, convert to Q
	 * See https://en.wikipedia.org/wiki/Q_(number_format)
	 *
	 * This version is used for the signed 32-bit values in metadata records.
     *
	 * @param qFloat
	 * @param qPoint
	 * @return
	 */
	int32_t floatToQ_dword(float qFloat, uint16_t qPoint);

	/**
	 * 	Tell the sensor to do a command.
	 * 	See SH-2 Reference Manual section 6.3.8 page 42, Command request
	 * 	The caller is expected to set shtpData 3 though 11 prior to calling
	 */
	void sendCommand(uint8_t command);

	/**
	 * Given a sensor's report ID, this tells the BNO080 to begin reporting the values.
	 *
	 * @param reportID
	 * @param timeBetweenReports
	 * @param specificConfig the specific config word. Useful for personal activity classifier.
	 */
	void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig = 0);

	/**
	 * Read a record from the FRS (Flash Record System) on the IMU.  FRS records are composed of 32-bit words,
	 * with the size of each record determined by the record type.
	 *
	 * Will block until the entire record has been read.
	 * @param recordID Record ID to read.  See SH-2 figures 28 and 29 for a list of these.  Sometimes also called
	 * the "FRS Type" by the datasheet (???).
	 * @param readBuffer Buffer to read data into.
	 * @param readLength Amount of words to read from the record.  Must be <= the length of the record.
	 *
	 * @return whether the request succeeded
	 */
	bool readFRSRecord(uint16_t recordID, uint32_t* readBuffer, uint16_t readLength);

	/**
	 * Write a record to the FRS (Flash Record System) on the IMU.  FRS records are composed of 32-bit words,
	 * with the size of each record determined by the record type.
	 *
	 * Will block until the entire record has been written.
	 * @param recordID Record ID to write.  See SH-2 figures 28 and 29 for a list of these.  Sometimes also called
	 * the "FRS Type" by the datasheet (???).
	 * @param buffer Buffer to write data into.
	 * @param length Amount of words to write to the record.  Must be <= the length of the record.
	 *
	 * @return whether the request succeeded
	 */
	bool writeFRSRecord(uint16_t recordID, uint32_t* buffer, uint16_t length);

	/**
	 * Reads a packet from the IMU and stores it in the class variables.
	 *
	 * @param timeout how long to wait for there to be a packet
	 *
	 * @return whether a packet was recieved.
	 */
	bool receivePacket(float timeout=.2f);

	/**
	 * Sends the current shtpData contents to the BNO.  It's a good idea to disable interrupts before you call this.
	 *
	 * @param channelNumber the channel to send on
	 * @param dataLength How many bits of shtpData to send
	 * @return
	 */
	bool sendPacket(uint8_t channelNumber, uint8_t dataLength);

	/**
	 * Prints the current shtp packet stored in the buffer.
	 * @param length
	 */
	void printPacket();

	/**
	 * Erases the current SHTP packet buffer so new data can be written
	 */
	 void zeroBuffer();

	 /**
	  * Loads the metadata for this report into the metadata buffer.
	  * @param report
	  * @return Whether the operation succeeded.
	  */
	 bool loadReportMetadata(Report report);

};


#endif //HAMSTER_BNO080_H
