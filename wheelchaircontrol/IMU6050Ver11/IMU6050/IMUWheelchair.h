#ifndef IMUWheelchair_H
#define IMUWheelchair_H

#include "filter.h"
//#include  "mbed.h"
#include  "math.h"
#include  <MPU6050.h>

#define PI 3.141593

/*#define SDA D14
#define SCL D15*/
#define SDA PB_9
#define SCL PB_8
#define SAMPLEFREQ 50
#define CAL_TIME 3

class IMUWheelchair {
    public:
        //The constructor for this class
        IMUWheelchair(Serial* out, Timer* time);
        IMUWheelchair(PinName sda_pin, PinName scl_pin, Serial* out, Timer* time);
        
        //Set up the IMU, check if it connects
        void setup();
        
        //Get the x-component of the angular acceleration
        double accel_x();
        
        //Get the y-component of the angular acceleration
        double accel_y();
        
        //Get the z-component of the angular acceleration
        double accel_z();
        
        //Get the x-component of gyro, angular velocity
        double gyro_x();
        
        //Get the y-component of gyro, angular velocity
        double gyro_y();
        
        //Get the z-component of gyro, angular velocity
        double gyro_z();
        
        //Magnometer to find angle relative to North to compare to gyroscope
        //double angle_north();
        
        //Get the YAW, or angle (theta), direction facing
        double yaw();
        
        //Get the pitch, (Up and down component)
        double pitch();
        
        //Get the roll, the tilt
        double roll();
  
        MPU6050* imu; //The IMU we're testing from, MPU6050  
        
    private:
        Serial* usb; //the connection port
        Timer* t;//to calculate the time
        float accelData[3];   // stores the angular acceleration component
        float gyroData[3];    //stores the gyro data x,y,z
        float* accelD;        //Pointer that points to either accelData 
        float* gyroD;         //Ptr to the gyroData array
        
        float angleData[3]; //Contains the pitch, roll, yaw angle
        float* angleD;//Ptr to angleData array
        
        void calibrate_yaw();
        
        bool start;
    
};

#endif