#ifndef BNO080Wheelchair_H
#define BNO080Wheelchair_H

//#include "filter.h"
#include "math.h"
#include "BNO080.h"
#include "BNO080Constants.h"

#define PI 3.141593

#define SDA PB_9
#define SCL PB_8
#define INT_PIN PE_2        // Change once actually connected
#define RST_PIN PE_4        // Change once actually connected

#define SAMPLEFREQ 50
#define CAL_TIME 3

class BNO080Wheelchair {
    public:
        BNO080* imu; //The IMU we're testing from, BNO080
        
        BNO080Wheelchair(Serial *debugPort, PinName sdaPin, 
                                 PinName sclPin, PinName intPin, PinName rstPin,
                                 uint8_t i2cAddress, int i2cPortpeed);
      
        //Set up the IMU, check if it connects
        bool setup();
        
        //Checks if IMU has new data
        bool hasNewData(BNO080::Report report);
        
        //Get the x-component of the linear acceleration (total)
        double accel_x();
        
        //Get the y-component of the linear acceleration (total)
        double accel_y();
        
        //Get the z-component of the linear acceleration (total)
        double accel_z();
        
        //Get the x-component of gyro, angular velocity
        double gyro_x();
        
        //Get the y-component of gyro, angular velocity
        double gyro_y();
        
        //Get the z-component of gyro, angular velocity
        double gyro_z();

        //Get the YAW, or angle (theta), direction facing
        double yaw();
        
        //Get the pitch, (Up and down component)
        double pitch();
        
        //Get the roll, the tilt
        double roll();
        
        //Get x component of mag field vector
        double mag_x();
        
        //Get y component of mag field vector
        double mag_y();
        
        //Get z component of mag field vector
        double mag_z();
        
        //Check if IMU is pointing in one of the 4 cardinal directions (NSWE)
        char compass();
        
        //Get the rotation of the IMU (from magnetic north) in radians
        TVector4 rotation();
        
        double rot_w();
        double rot_x();
        double rot_y();
        double rot_z();

        
        
    private:

        Timer* t;//to calculate the time

};

#endif
