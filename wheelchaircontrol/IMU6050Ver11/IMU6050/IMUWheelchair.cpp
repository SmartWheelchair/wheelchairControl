#include "IMUWheelchair.h"
float total_yaw;


IMUWheelchair::IMUWheelchair(Serial* out, Timer* time)
{
    imu = new MPU6050(SDA, SCL);
    usb = out;
    t = time;
    start = false;
    IMUWheelchair::setup();

}

IMUWheelchair::IMUWheelchair(PinName sda_pin, PinName scl_pin, Serial* out, Timer* time){
    usb = out;
    t = time;
    imu = new MPU6050(sda_pin, scl_pin);
    IMUWheelchair::setup();

}

void IMUWheelchair::setup() {
    imu->setGyroRange(MPU6050_GYRO_RANGE_1000);
    accelD = accelData;
    gyroD = gyroData;
    if(imu->testConnection()== 0)
        printf("not connected\r\n");
    else
        printf("connected\r\n");
    
    //timer
    t->start();
    
}

//Get the x component of the angular acceleration from IMU. Stores the component
//in a float array
//Returns a double, the value of the x-acceleration (m/s^2)
double IMUWheelchair::accel_x() {  
    imu -> getAccelero(accelD); //Change the values in accelerationArray
    return (double)accelD[0];
}

//Get the y component of the angular acceleration from IMU. Stores the component
//in a float array
//Returns a double, the value of the y-acceleration (m/s^2)
double IMUWheelchair::accel_y() {
    imu -> getAccelero(accelD); //Change the values in accelerationArray
    return (double)accelD[1];   
}

//Get the z component of the angular acceleration from IMU. Stores the component
//in a float array
//Returns a double, the value of the z-acceleration (m/s^2)
double IMUWheelchair::accel_z() {
    imu -> getAccelero(accelD); //Change the values in accelerationArray
    return (double)accelD[2];
}

//Get the x component of the angular velocity from IMU's gyroscope. Stores the 
//component in a float array
//Returns a double, the value of the x-angular velocity (rad/s)
double IMUWheelchair::gyro_x() {
    imu->getGyro(gyroD); //Change the values in gyroArray
    return (double)gyroD[0];
    
}

//Get the y component of the angular velocity from IMU's gyroscope. Stores the 
//component in a float array
//Returns a double, the value of the y-angular velocity (rad/s)
double IMUWheelchair::gyro_y() {
    imu -> getGyro(gyroD); //Change the values in gyroArray
    return (double)gyroD[1];
    
}

//Get the z component of the angular velocity from IMU's gyroscope. Stores the 
//component in a float array
//Returns a double, the value of the z-angular velocity (rad/s)
double IMUWheelchair::gyro_z() {
    imu -> getGyro(gyroD); //Change the values in gyroArray
    return (double)gyroD[2];
}



//Get the yaw, or the angle turned at a certain time interval
//Return double, the angle or yaw, (degree)
double IMUWheelchair::yaw() {

    float gyroZ = .4+(IMUWheelchair::gyro_x())*180/3.141593;
    if(abs(gyroZ) >= .5) {
     //printf("t->read(): %lf, gyroscope %lf, change %lf\r\n", t->read(), gyroZ, t->read()*gyroZ*2.25);
        total_yaw = total_yaw - t->read()*gyroZ;
     //printf("total_yaw: %lf, gyroZ: %f \r\n", total_yaw, gyroZ);
    }
    t->reset();
    if(total_yaw > 360)
        total_yaw -= 360;
    if(total_yaw < 0)
        total_yaw += 360;
    return (double)total_yaw;
}


double IMUWheelchair::pitch()
 {
    imu->getAccelero(accelD);
    float pitch = atan2 (-accelD[1] ,( sqrt (accelD[0] * accelD[0]) +(accelD[2]  *accelD[2])));
    pitch = pitch*57.3;
    return (double)pitch;
}

double IMUWheelchair::roll() {
    imu->getAccelero(accelD);
    float roll = atan2(-accelD[0]  ,( sqrt((accelD[1] *accelD[1] ) + 
 (accelD[2] * accelD[2]))));
    roll = roll*57.3;
    t->reset();
    return (double)roll;
}
