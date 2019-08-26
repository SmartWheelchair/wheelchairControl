#include "BNO080Wheelchair.h"
float total_yaw;

//The constructor for the BNO080 imu. Needs 7 parameters
BNO080Wheelchair::BNO080Wheelchair(Serial* debugPort, Timer* time) {
    imu = new BNO080(debugPort, SDA, SCL, INT_PIN,RST_PIN,0x4b, 100000);
    //setUp
    t = time;
}
//Check if all the
bool BNO080Wheelchair::setup() {
    bool setup = imu -> begin();
    //Tell the IMU to report every 100ms
    imu -> enableReport(BNO080::TOTAL_ACCELERATION, 200);
    imu -> enableReport(BNO080::LINEAR_ACCELERATION, 200);
    imu -> enableReport(BNO080::GRAVITY_ACCELERATION, 200);
    imu -> enableReport(BNO080::GYROSCOPE, 200);
    imu -> enableReport(BNO080::MAG_FIELD, 200);
//    imu -> enableReport(BNO080::MAG_FIELD_UNCALIBRATED, 100);    
//    imu -> enableReport(BNO080::ROTATION, 100);
//    imu -> enableReport(BNO080::GEOMAGNETIC_ROTATION, 100);    
//    imu -> enableReport(BNO080::GAME_ROTATION, 100);    
//    imu -> enableReport(BNO080::TAP_DETECTOR, 100);   
//    imu -> enableReport(BNO080::STABILITY_CLASSIFIER, 100);    
//    imu -> enableReport(BNO080::STEP_DETECTOR, 100);    
//    imu -> enableReport(BNO080::STEP_COUNTER, 100);
//    imu -> enableReport(BNO080::SIGNIFICANT_MOTION, 100);    
//    imu -> enableReport(BNO080::SHAKE_DETECTOR, 100);
    t->start();
    return setup;
}

bool BNO080Wheelchair::hasNewData(BNO080::Report report) {
    return imu -> hasNewData(report);
}

//Get the x component of the angular velocity from IMU. Stores the component
//in a float array
//Returns a double, the value of the x-acceleration (m/s^2)
double BNO080Wheelchair::gyro_x() {
    wait(0.05);
    imu -> updateData();
    return (double)imu -> gyroRotation[0];
}

//Get the y component of the angular velocity from IMU. Stores the component
//in a float array
//Returns a double, the value of the y-acceleration (m/s^2)
double BNO080Wheelchair::gyro_y() {
    wait(0.05);
    imu -> updateData();
    return (double)imu -> gyroRotation[1];
}

//Get the z component of the angular velocity from IMU. Stores the component
//in a float array
//Returns a double, the value of the z-acceleration (m/s^2)
double BNO080Wheelchair::gyro_z() {
    wait(0.05);
    imu -> updateData();
    return (double)imu -> gyroRotation[2];
}

//Get the x component of the linear acceleration from IMU. Stores the component
//in a float array
//Returns a double, the value of the x-acceleration (m/s^2)
double BNO080Wheelchair::accel_x() {
    wait(0.05);
    imu -> updateData();
    return (double)imu -> totalAcceleration[0];
}

//Get the y component of the linear acceleration from IMU. Stores the component
//in a float array
//Returns a double, the value of the y-acceleration (m/s^2)
double BNO080Wheelchair::accel_y() {
    wait(0.05);
    imu -> updateData();
    return (double)imu -> totalAcceleration[1];
}

//Get the z component of the linear acceleration from IMU. Stores the component
//in a float array
//Returns a double, the value of the z-acceleration (m/s^2)
double BNO080Wheelchair::accel_z() {
    wait(0.05);
    imu -> updateData();
    return (double)imu -> totalAcceleration[2];
}

//Get yaw
double BNO080Wheelchair::yaw() {

    float gyroZ = (BNO080Wheelchair::gyro_z())*180/3.141593;
    if(abs(gyroZ) >= .3) {
     //printf("t->read(): %lf, gyroscope %lf, change %lf\r\n", t->read(), gyroZ, t->read()*gyroZ*2.25);
        total_yaw = total_yaw - t->read()*gyroZ;
     //printf("total_yaw: %lf, gyroZ: %f \r\n", total_yaw, curr_yaw_rawgyroZ);
    }
    t->reset();
    if(total_yaw > 360)
        total_yaw -= 360;
    if(total_yaw < 0)
        total_yaw += 360;
    return (double)total_yaw;
}

//Get x component of magnetic field vector
double BNO080Wheelchair::mag_x() {
    wait(1);
    imu -> updateData();
    return (double)imu -> magField[0];
}

//Get y component of magnetic field vector
double BNO080Wheelchair::mag_y() {
    wait(1);
    imu -> updateData();
    return (double)imu -> magField[1];
}

//Get z component of magnetic field vector
double BNO080Wheelchair::mag_z() {
    wait(1);
    imu -> updateData();
    return (double)imu -> magField[2];
}

//Check if IMU is pointing in one of the 4 cardinal directions (NSWE)
char BNO080Wheelchair::compass() {
    imu -> updateData();
    double x = imu -> magField[0];
    double y = imu -> magField[1];
    
    if ( (x>10) && (y<-5) && (y>-10) ) {
        return 'N'; //Facing NORTH
    }
    
    else if ( (x<-8) && (y>-5) ) {
        return 'S'; //Facing SOUTH
    }
    
    else if ( (x>-3) && (x<4) && (y<-15) ) {
        return 'W'; //Facing WEST
    }

    else if ( (x>4) && (x<8) && (y>1) ) {
        return 'E'; //Facing EAST
    }
    
    else if ( (x==0) && (y==0) ) {
        return 'O'; //IMU reports not enabled correctly, zeros received
    }   

    else {
        return 'I'; //Direction indeterminate
    }
}

//Get the rotation of the IMU (from magnetic north) in radians
TVector4 BNO080Wheelchair::rotation() {
    wait(0.05);
    //printf("Update Data GYRO X: %d \n", imu -> updateData());            // hasNewData()?
    imu -> updateData();
    //wait(0.05);
    return imu -> rotationVector.vector();
}

bool BNO080Wheelchair::calibrate(){
	wait(0.02);
	if(imu -> enableCalibration(true, true, false)){
		wait(0.02);
		if(imu ->saveCalibration()){
			printf("Calibration is saved\r\n");
			return true;
		}
		printf("Calibration is enabled, but not saved\r\n");
		return true;
	}
	else{
		printf("Calibration failed\r\n");
		return false;
	}
}

/*
//Returns Qw component of rotation vector
double BNO080Wheelchair::rot_w() {
    wait(0.05);
    imu -> updateData();
    return (double)imu -> rotationVector[0];
}

//Returns Qx component of rotation vector
double BNO080Wheelchair::rot_x() {
    wait(0.05);
    imu -> updateData();
    return (double)imu -> rotationVector[1];
}

//Returns Qy component of rotation vector
double BNO080Wheelchair::rot_y() {
    wait(0.05);
    imu -> updateData();
    return (double)imu -> rotationVector[2];
}

//Returns Qz component of rotation vector
double BNO080Wheelchair::rot_z() {
    wait(0.05);
    imu -> updateData();
    return (double)imu -> rotationVector[2];
}
*/
