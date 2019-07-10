/**
 * Includes
 */
#include "MPU6050.h"

MPU6050::MPU6050(PinName sda, PinName scl) : connection(sda, scl) {
    this->setSleepMode(false);
    
    //Initializations:
    currentGyroRange = 0;
    currentAcceleroRange=0;
    alpha = ALPHA;
}

//--------------------------------------------------
//-------------------General------------------------
//--------------------------------------------------

void MPU6050::write(char address, char data) {
    char temp[2];
    temp[0]=address;
    temp[1]=data;
    
    connection.write(MPU6050_ADDRESS * 2,temp,2);
}

char MPU6050::read(char address) {
    char retval;
    connection.write(MPU6050_ADDRESS * 2, &address, 1, true);
    connection.read(MPU6050_ADDRESS * 2, &retval, 1);
    return retval;
}

void MPU6050::read(char address, char *data, int length) {
    connection.write(MPU6050_ADDRESS * 2, &address, 1, true);
    connection.read(MPU6050_ADDRESS * 2, data, length);
}

void MPU6050::setSleepMode(bool state) {
    char temp;
    temp = this->read(MPU6050_PWR_MGMT_1_REG);
    if (state == true)
        temp |= 1<<MPU6050_SLP_BIT;
    if (state == false)
        temp &= ~(1<<MPU6050_SLP_BIT);
    this->write(MPU6050_PWR_MGMT_1_REG, temp);
}

bool MPU6050::testConnection( void ) {
    char temp;
    temp = this->read(MPU6050_WHO_AM_I_REG);
    return (temp == (MPU6050_ADDRESS & 0xFE));
}

void MPU6050::setBW(char BW) {
    char temp;
    BW=BW & 0x07;
    temp = this->read(MPU6050_CONFIG_REG);
    temp &= 0xF8;
    temp = temp + BW;
    this->write(MPU6050_CONFIG_REG, temp);
}

void MPU6050::setI2CBypass(bool state) {
    char temp;
    temp = this->read(MPU6050_INT_PIN_CFG);
    if (state == true)
        temp |= 1<<MPU6050_BYPASS_BIT;
    if (state == false)
        temp &= ~(1<<MPU6050_BYPASS_BIT);
    this->write(MPU6050_INT_PIN_CFG, temp);
}

//--------------------------------------------------
//----------------Accelerometer---------------------
//--------------------------------------------------

void MPU6050::setAcceleroRange( char range ) {
    char temp;
    range = range & 0x03;
    currentAcceleroRange = range;
    
    temp = this->read(MPU6050_ACCELERO_CONFIG_REG);
    temp &= ~(3<<3);
    temp = temp + (range<<3);
    this->write(MPU6050_ACCELERO_CONFIG_REG, temp);
}

int MPU6050::getAcceleroRawX( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_ACCEL_XOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}
    
int MPU6050::getAcceleroRawY( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_ACCEL_YOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

int MPU6050::getAcceleroRawZ( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_ACCEL_ZOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

void MPU6050::getAcceleroRaw( int *data ) {
    char temp[6];
    this->read(MPU6050_ACCEL_XOUT_H_REG, temp, 6);
    data[0] = (int)(short)((temp[0]<<8) + temp[1]);
    data[1] = (int)(short)((temp[2]<<8) + temp[3]);
    data[2] = (int)(short)((temp[4]<<8) + temp[5]);
}

void MPU6050::getAccelero( float *data ) {
    int temp[3];
    this->getAcceleroRaw(temp);
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_2G) {
        data[0]=(float)temp[0] / 16384.0 * 9.81;
        data[1]=(float)temp[1] / 16384.0 * 9.81;
        data[2]=(float)temp[2] / 16384.0 * 9.81;
        }
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_4G){
        data[0]=(float)temp[0] / 8192.0 * 9.81;
        data[1]=(float)temp[1] / 8192.0 * 9.81;
        data[2]=(float)temp[2] / 8192.0 * 9.81;
        }
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_8G){
        data[0]=(float)temp[0] / 4096.0 * 9.81;
        data[1]=(float)temp[1] / 4096.0 * 9.81;
        data[2]=(float)temp[2] / 4096.0 * 9.81;
        }
    if (currentAcceleroRange == MPU6050_ACCELERO_RANGE_16G){
        data[0]=(float)temp[0] / 2048.0 * 9.81;
        data[1]=(float)temp[1] / 2048.0 * 9.81;
        data[2]=(float)temp[2] / 2048.0 * 9.81;
        }
    
    #ifdef DOUBLE_ACCELERO
        data[0]*=2;
        data[1]*=2;   
        data[2]*=2;
    #endif   
}
//--------------------------------------------------
//------------------Gyroscope-----------------------
//--------------------------------------------------
void MPU6050::setGyroRange( char range ) {
    char temp;
    currentGyroRange = range;
    range = range & 0x03;
    temp = this->read(MPU6050_GYRO_CONFIG_REG);
    temp &= ~(3<<3);
    temp = temp + range<<3;
    this->write(MPU6050_GYRO_CONFIG_REG, temp);
}

int MPU6050::getGyroRawX( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_GYRO_XOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}
    
int MPU6050::getGyroRawY( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_GYRO_YOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

int MPU6050::getGyroRawZ( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_GYRO_ZOUT_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

void MPU6050::getGyroRaw( int *data ) {
    char temp[6];
    this->read(MPU6050_GYRO_XOUT_H_REG, temp, 6);
    data[0] = (int)(short)((temp[0]<<8) + temp[1]);
    data[1] = (int)(short)((temp[2]<<8) + temp[3]);
    data[2] = (int)(short)((temp[4]<<8) + temp[5]);
}

void MPU6050::getGyro( float *data ) {
    int temp[3];
    this->getGyroRaw(temp);
    if (currentGyroRange == MPU6050_GYRO_RANGE_250) {
        data[0]=(float)temp[0] / 301.0;
        data[1]=(float)temp[1] / 301.0;
        data[2]=(float)temp[2] / 301.0;
        }               //7505.5
    if (currentGyroRange == MPU6050_GYRO_RANGE_500){
        data[0]=(float)temp[0] / 3752.9;
        data[1]=(float)temp[1] / 3752.9;
        data[2]=(float)temp[2] / 3752.9;
        }
    if (currentGyroRange == MPU6050_GYRO_RANGE_1000){
        data[0]=(float)temp[0]/ 1879.3;
        data[1]=(float)temp[1] / 1879.3;
        data[2]=(float)temp[2] / 1879.3;

        }
    if (currentGyroRange == MPU6050_GYRO_RANGE_2000){
        data[0]=(float)temp[0] / 939.7;
        data[1]=(float)temp[1] / 939.7;
        data[2]=(float)temp[2] / 939.7;
        }

}
//--------------------------------------------------
//-------------------Temperature--------------------
//--------------------------------------------------
int MPU6050::getTempRaw( void ) {
    short retval;
    char data[2];
    this->read(MPU6050_TEMP_H_REG, data, 2);
    retval = (data[0]<<8) + data[1];
    return (int)retval;
}

float MPU6050::getTemp( void ) {
    float retval;
    retval=(float)this->getTempRaw();
    retval=(retval+521.0)/340.0+35.0;
    return retval;
}

/**Additional function added by Montvydas Klumbys, which will allow easy offset, angle calculation and much more.
 function for getting angles in degrees from accelerometer
*/
void MPU6050::getAcceleroAngle( float *data ) {
    float temp[3];
    this->getAccelero(temp);
    
    data[X_AXIS] = atan (temp[Y_AXIS]/sqrt(pow(temp[X_AXIS], 2) + pow(temp[Z_AXIS], 2))) * RADIANS_TO_DEGREES; //calculate angle x(pitch/roll?) from accellerometer reading
    data[Y_AXIS] = atan (-1*temp[X_AXIS]/sqrt(pow(temp[Y_AXIS], 2) + pow(temp[Z_AXIS], 2))) * RADIANS_TO_DEGREES; //calculate angle x(pitch/roll?) from accellerometer reading
    data[Z_AXIS] = atan (sqrt(pow(temp[X_AXIS], 2) + pow(temp[Y_AXIS], 2))/temp[Z_AXIS]) * RADIANS_TO_DEGREES; //This one is not used anywhere later on
    
//    data[Y_AXIS] = atan2 (temp[Y_AXIS],temp[Z_AXIS]) * RADIANS_TO_DEGREES;  //This spits out values between -180 to 180 (360 degrees)
//    data[X_AXIS] = atan2 (-1*temp[X_AXIS], temp[Z_AXIS]) * RADIANS_TO_DEGREES;    //but it takes longer and system gets unstable when angles ~90 degrees
}

///function for getting offset values for the gyro & accelerometer
void MPU6050::getOffset(float *accOffset, float *gyroOffset, int sampleSize){
    float gyro[3];      
    float accAngle[3];
    
    for (int i = 0; i < 3; i++) {
        accOffset[i] = 0.0;     //initialise offsets to 0.0
        gyroOffset[i] = 0.0;
    }
    
    for (int i = 0; i < sampleSize; i++){
        this->getGyro(gyro); //take real life measurements  
        this->getAcceleroAngle (accAngle);
        
        for (int j = 0; j < 3; j++){
            *(accOffset+j) += accAngle[j]/sampleSize;    //average measurements
            *(gyroOffset+j) += gyro[j]/sampleSize;
        }
        wait (0.01);    //wait between each reading for accuracy 
    } 
}

///function for computing angles for roll, pitch anf yaw
void MPU6050::computeAngle (float *angle, float *accOffset, float *gyroOffset, float interval){
    float gyro[3];
    float accAngle[3];
    
    this->getGyro(gyro);                //get gyro value in rad/s
    this->getAcceleroAngle(accAngle);   //get angle from accelerometer
    
    for (int i = 0; i < 3; i++){
        gyro[i] -= gyroOffset[i];       //substract offset values
        accAngle[i] -= accOffset[i];    
    }
    
    //apply filters on pitch and roll to get accurate angle values
    angle[X_AXIS] = alpha * (angle[X_AXIS] + GYRO_SCALE*gyro[X_AXIS]*interval) + (1-alpha)*accAngle[X_AXIS]; 
    angle[Y_AXIS] = alpha * (angle[Y_AXIS] + GYRO_SCALE*gyro[Y_AXIS]*interval) + (1-alpha)*accAngle[Y_AXIS];
    
    //calculate Yaw using just the gyroscope values - inaccurate
    angle[Z_AXIS] = angle[Z_AXIS] + GYRO_SCALE*gyro[Z_AXIS]*interval;     
}

///function for setting a different Alpha value, which is used in complemetary filter calculations
void MPU6050::setAlpha(float val){
    alpha = val;   
}

///function for enabling interrupts on MPU6050 INT pin, when the data is ready to take
void MPU6050::enableInt( void ){
    char temp;
    temp = this->read(MPU6050_RA_INT_ENABLE);
    temp |= 0x01;
    this->write(MPU6050_RA_INT_ENABLE, temp);
}

///function for disabling interrupts on MPU6050 INT pin, when the data is ready to take
void MPU6050::disableInt ( void ){
    char temp;
    temp = this->read(MPU6050_RA_INT_ENABLE);
    temp &= 0xFE;
    this->write(MPU6050_RA_INT_ENABLE, temp);
}