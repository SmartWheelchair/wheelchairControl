/*Use #define MPU6050_ES before you include this file if you have an engineering sample (older EVBs will have them), to find out if you have one, check your accelerometer output. 
If it is half of what you expected, and you still are on the correct planet, you got an engineering sample
*/


#ifndef MPU6050_H
#define MPU6050_H

/**
 * Includes
 */
#include "mbed.h"


/**
 * Defines
 */
#ifndef MPU6050_ADDRESS
    #define MPU6050_ADDRESS             0x68 // address pin low (GND), default for InvenSense evaluation board
#endif

#ifdef MPU6050_ES
        #define DOUBLE_ACCELERO
#endif  

/**
 * Registers
 */
 #define MPU6050_CONFIG_REG         0x1A
 #define MPU6050_GYRO_CONFIG_REG    0x1B
 #define MPU6050_ACCELERO_CONFIG_REG    0x1C
  
 #define MPU6050_INT_PIN_CFG        0x37
 
 #define MPU6050_ACCEL_XOUT_H_REG   0x3B
 #define MPU6050_ACCEL_YOUT_H_REG   0x3D
 #define MPU6050_ACCEL_ZOUT_H_REG   0x3F
 
 #define MPU6050_TEMP_H_REG         0x41
 
 #define MPU6050_GYRO_XOUT_H_REG    0x43
 #define MPU6050_GYRO_YOUT_H_REG    0x45
 #define MPU6050_GYRO_ZOUT_H_REG    0x47
 
 
 
 #define MPU6050_PWR_MGMT_1_REG     0x6B
 #define MPU6050_WHO_AM_I_REG       0x75
 
                 
 
 /**
  * Definitions
  */
#define MPU6050_SLP_BIT             6
#define MPU6050_BYPASS_BIT         1

#define MPU6050_BW_256              0
#define MPU6050_BW_188              1
#define MPU6050_BW_98               2
#define MPU6050_BW_42               3
#define MPU6050_BW_20               4
#define MPU6050_BW_10               5
#define MPU6050_BW_5                6

#define MPU6050_ACCELERO_RANGE_2G   0
#define MPU6050_ACCELERO_RANGE_4G   1
#define MPU6050_ACCELERO_RANGE_8G   2
#define MPU6050_ACCELERO_RANGE_16G  3

#define MPU6050_GYRO_RANGE_250      0
#define MPU6050_GYRO_RANGE_500      1
#define MPU6050_GYRO_RANGE_1000     2
#define MPU6050_GYRO_RANGE_2000     3

//interrupt address
#define MPU6050_RA_INT_ENABLE 0x38  
//define how the accelerometer is placed on surface
#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 0
//translation from radians to degrees
#define RADIANS_TO_DEGREES 180/3.1415926536
//constant used for the complementary filter, which ranges usually from 0.9 to 1.0
#define ALPHA 0.96   //filter constant
//scale the gyro
#define GYRO_SCALE 2.7176 

/** MPU6050 IMU library.
  *
  * Example:
  * @code
  * Later, maybe
  * @endcode
  */
class MPU6050 {
    public:
     /**
     * Constructor.
     *
     * Sleep mode of MPU6050 is immediatly disabled
     *
     * @param sda - mbed pin to use for the SDA I2C line.
     * @param scl - mbed pin to use for the SCL I2C line.
     */
     MPU6050(PinName sda, PinName scl);
     

     /**
     * Tests the I2C connection by reading the WHO_AM_I register. 
     *
     * @return True for a working connection, false for an error
     */     
     bool testConnection( void );
     
     /**
     * Sets the bandwidth of the digital low-pass filter 
     *
     * Macros: MPU6050_BW_256 - MPU6050_BW_188 - MPU6050_BW_98 - MPU6050_BW_42 - MPU6050_BW_20 - MPU6050_BW_10 - MPU6050_BW_5
     * Last number is the gyro's BW in Hz (accelero BW is virtually identical)
     *
     * @param BW - The three bits that set the bandwidth (use the predefined macros)
     */     
     void setBW( char BW );
     
     /**
     * Sets the auxiliary I2C bus in bypass mode to read the sensors behind the MPU6050 (useful for eval board, otherwise just connect them to primary I2C bus) 
     *
     * @param state - Enables/disables the I2C bypass mode
     */     
     void setI2CBypass ( bool state );
     
     /**
     * Sets the Accelero full-scale range
     *
     * Macros: MPU6050_ACCELERO_RANGE_2G - MPU6050_ACCELERO_RANGE_4G - MPU6050_ACCELERO_RANGE_8G - MPU6050_ACCELERO_RANGE_16G
     *
     * @param range - The two bits that set the full-scale range (use the predefined macros)
     */
     void setAcceleroRange(char range);
     
     /**
     * Reads the accelero x-axis.
     *
     * @return 16-bit signed integer x-axis accelero data
     */   
     int getAcceleroRawX( void );
     
     /**
     * Reads the accelero y-axis.
     *
     * @return 16-bit signed integer y-axis accelero data
     */   
     int getAcceleroRawY( void );
     
     /**
     * Reads the accelero z-axis.
     *
     * @return 16-bit signed integer z-axis accelero data
     */   
     int getAcceleroRawZ( void );
     
     /**
     * Reads all accelero data.
     *
     * @param data - pointer to signed integer array with length three: data[0] = X, data[1] = Y, data[2] = Z
     */   
     void getAcceleroRaw( int *data );
     
     /**
     * Reads all accelero data, gives the acceleration in m/s2
     *
     * Function uses the last setup value of the full scale range, if you manually set in another range, this won't work.
     *
     * @param data - pointer to float array with length three: data[0] = X, data[1] = Y, data[2] = Z
     */   
     void getAccelero( float *data );
     
     /**
     * Sets the Gyro full-scale range
     *
     * Macros: MPU6050_GYRO_RANGE_250 - MPU6050_GYRO_RANGE_500 - MPU6050_GYRO_RANGE_1000 - MPU6050_GYRO_RANGE_2000
     *
     * @param range - The two bits that set the full-scale range (use the predefined macros)
     */
     void setGyroRange(char range);

     /**
     * Reads the gyro x-axis.
     *
     * @return 16-bit signed integer x-axis gyro data
     */   
     int getGyroRawX( void );
     
     /**
     * Reads the gyro y-axis.
     *
     * @return 16-bit signed integer y-axis gyro data
     */   
     int getGyroRawY( void );
     
     /**
     * Reads the gyro z-axis.
     *
     * @return 16-bit signed integer z-axis gyro data
     */   
     int getGyroRawZ( void );
     
     /**
     * Reads all gyro data.
     *
     * @param data - pointer to signed integer array with length three: data[0] = X, data[1] = Y, data[2] = Z
     */   
     void getGyroRaw( int *data );  
     
     /**
     * Reads all gyro data, gives the gyro in rad/s
     *
     * Function uses the last setup value of the full scale range, if you manually set in another range, this won't work.
     *
     * @param data - pointer to float array with length three: data[0] = X, data[1] = Y, data[2] = Z
     */   
     void getGyro( float *data);     
     
     /**
     * Reads temperature data.
     *
     * @return 16 bit signed integer with the raw temperature register value
     */  
     int getTempRaw( void );
     
     /**
     * Returns current temperature
     *
     * @returns float with the current temperature
     */  
     float getTemp( void );

     /**
     * Sets the sleep mode of the MPU6050 
     *
     * @param state - true for sleeping, false for wake up
     */     
     void setSleepMode( bool state );
     
     
     /**
     * Writes data to the device, could be private, but public is handy so you can transmit directly to the MPU. 
     *
     * @param adress - register address to write to
     * @param data - data to write
     */
     void write( char address, char data);
     
     /**
     * Read data from the device, could be private, but public is handy so you can transmit directly to the MPU. 
     *
     * @param adress - register address to write to
     * @return - data from the register specified by RA
     */
     char read( char adress);
     
     /**
     * Read multtiple regigsters from the device, more efficient than using multiple normal reads. 
     *
     * @param adress - register address to write to
     * @param length - number of bytes to read
     * @param data - pointer where the data needs to be written to 
     */
     void read( char adress, char *data, int length);
     
    /**
    * function for calculating the angle from the accelerometer.
    * it takes 3 values which correspond acceleration in X, Y and Z direction and calculates angles in degrees
    * for pitch, roll and one more direction.. (NOT YAW!)
    *
    * @param data - angle calculated using only accelerometer
    *
    */ 
    void getAcceleroAngle( float *data );
     
     
     /**function which allows to produce the offset values for gyro and accelerometer.
     * offset for gyro is simply a value, which needs to be substracted from original gyro rad/sec speed
     * but offset for accelerometer is calculated in angles... later on might change that
     * function simply takes the number of samples to be taken and calculated the average
     * 
     * @param accOffset - accelerometer offset in angle
     * @param gyroOffset - gyroscope offset in rad/s
     * @param sampleSize - number of samples to be taken for calculating offsets
     *
     */
     void getOffset(float *accOffset, float *gyroOffset, int sampleSize);
     
     /**
     * function for computing the angle, when accelerometer angle offset and gyro offset are both known.
     * we also need to know how much time passed from previous calculation to now
     * it produces the angle in degrees. However angles are produced from -90.0 to 90.0 degrees
     * if anyone need smth different, they can update this library...
     *
     * @param angle - calculated accurate angle from complemetary filter
     * @param accOffset - offset in angle for the accelerometer
     * @param gyroOffset - offset in rad/s for the gyroscope
     * @param interval - time before previous angle calculation and now
     *
     */
     void computeAngle (float *angle, float *accOffset, float *gyroOffset, float interval);
     
     ///function, which enables interrupts on MPU6050 INT pin
     void enableInt( void );
     
     ///disables interrupts
     void disableInt( void );
     
     /**function which sets the alpha value - constant for the complementary filter. default alpha = 0.97
     *
     * @param val - value the alpha (complementary filter constant) should be set to
     *
     */
     void setAlpha(float val);
     
     private:

     I2C connection;
     char currentAcceleroRange;
     char currentGyroRange;
     float alpha;   
     
};



#endif
