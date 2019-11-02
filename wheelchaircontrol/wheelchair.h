#ifndef wheelchair
#define wheelchair
/*************************************************************************
*            Importing libraries into wheelchair.h                       *
**************************************************************************/
#include "BNO080Wheelchair.h"
#include "PID.h"
#include "QEI.h"
#include "VL53L1X.h"
#include "Statistics.h"
#include "Watchdog.h"
/*
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
*/
/*************************************************************************
* Joystick has analog out of 200-700, scale values between 1.3 and 3.3   *
* Here are some global constants for joystick                            *
**************************************************************************/
#define def (2.55f/3.3f)           	// Default axis on joystick to stay neutral; used on x and y axis
#define high 3.3f/3.3f            	// High power on joystick; used on x and y axis
#define low (1.7f/3.3f)     		// Low power on joystick; used on x and y axis
#define offset .018f     			// Joystick adjustment to be able to go straight. Chair dependent on manufactoring precision
#define process .1             		// Defines default time delay in seconds

/*************************************************************************
*Pin plug-in for Nucleo-L432KC/Compatible with Nucleo-400 series (F767ZI)*
**************************************************************************/
#define yDir PD_14                       // PWM Pins
#define xDir PD_15
#define Encoder1 D7                     // Digital In Pull Up Pin 
#define Encoder2 D8
#define Diameter 31.75                  // Diameter of encoder wheel
#define maxDecelerationSlow 130
#define maxDecelerationFast 80
#define ToFSensorNum 12

/*************************************************************************
*IMU definitions for turning wheelchair
**************************************************************************/
#define WheelchairRadius 95             //distance from IMU to edge of wheelchair(cm)
#define  maxAngularDeceleration 0.5    //found through testing, max
                                        //acceleration at which chair can 
                                        //stop while turning. In rads per sec
#define minWallLengthLeft 100                // minimum distance from wall to ToF (cm)
#define minWallLengthRight 100                // minimum distance from wall to ToF (cm)
/*************************************************************************
*                                                                        *
*                         Wheelchair class                               *
*           Used for controlling the Smart Wheelchair                    *
*                                                                        *
**************************************************************************/
class Wheelchair
{
public:
    /*************************************************************************
    *   Create Wheelchair constructor with x,y pin for analog-to-dc output   *
    *   serial for printout, and time. This is also used to initialize some  *
    *   variables for the timer,encoders and time-of-flight sensors          *
    **************************************************************************/
    Wheelchair(PinName xPin, PinName yPin, Serial* pc, Timer* time, QEI* wheel, QEI* wheelS,
               VL53L1X** ToF, DigitalIn* e1_button);

    /*************************************************************************
    *           This method is to move using the joystick                    *
    **************************************************************************/
    void move(float x_coor, float y_coor);

    /*************************************************************************
    *   This method is to drive the wheelchair forward manually (NO PID used)*
    ************************************************************************ */
    void forward();
    /*************************************************************************
    *   This method is to drive the wheelchair backwards (NO PID used)       *
    ************************************************************************ */
    void backward();

    /*************************************************************************
    *   This method is to turn the wheelchair right manually (NO PID used)   *
    ************************************************************************ */
    void right();

    /*************************************************************************
    *   This method is to turn the wheelchair left manually (NO PID used)    *
    ************************************************************************ */
    void left();

    /*************************************************************************
    *   This method stops the wheelchair and reset the joystick position     *
    ************************************************************************ */
    void stop();

    /*************************************************************************
    *   This method is a thread that will obtain the IMU information such    *
    *   as the gyroscope x,y axis. Z-axis is not used.                       *
    ************************************************************************ */
    void compass_thread();

    /*************************************************************************
    *   This method is a thread that will calculate the velocity of the      *
    *   wheechair using the encoder values this is being obatined.           *
    ************************************************************************ */
    void velocity_thread();

    void pid_wall_follower();

    //Currently not in use (for eclipse IDE)
    void rosCom_thread();

    /*************************************************************************
    *   This method is a thread that iterates through all the sensor's       *
    *   values and determines whether or not the wheelchair is about to hit  *
    *   or crash into something. If the sensors detect something close to    *
    *   the chair, then the chair will safely halt and allow movement in the *
    *   direction opposed to where an object is detected.                    *
    ************************************************************************ */
    void ToFSafe_thread();

    /*************************************************************************
    *   This method is a thread that will constantly be checking the value   *
    *   of the emergency button. If the button is pressed, then the chair    *
    *   will stop and the entire system will reset.                          *
    ************************************************************************ */
    void emergencyButton_thread();

    //---------------------------------------
    void imuRead_thread();
    void forwardSafety_thread();
    void rightSideSafety_thread();
    void leftSideSafety_thread();
    void backwardSafety_thread();
    void ledgeSafety_thread();
    //--------------------------------------

    /*************************************************************************
    *   This method gets the encoder values and calculates the distance since*
    *   the last encoder reset.                                              *
    **************************************************************************/
    float getDistance();

    /*************************************************************************
    *   This method resets the encoder value to recalculate distance         *
    **************************************************************************/
    void resetDistance();


    /*************************************************************************
     *                                                                       *
     *                         PID Control Methods                           *
     *                                                                       *
     *************************************************************************/


    /*************************************************************************
    * This method moves the wheelchair x-millimiters forward using PID       *
    **************************************************************************/
    void pid_forward(double mm);

    /*************************************************************************
    * This method turns the chair right a certain amount of degrees using PID*
    **************************************************************************/
    void pid_right(int deg);

    /*************************************************************************
    * This method turns the chair left a certain amount of degrees using PID *
    **************************************************************************/
    void pid_left(int deg);

    /*************************************************************************
    * This method determines whether to turn the wheelchair left or right    *
    **************************************************************************/
    void pid_turn(int deg);


    /*************************************************************************
     *                                                                       *
     *                     ROS-Embed Communication Methods                   *
     *                                                                       *
     *************************************************************************/

    /*************************************************************************
     * This method computes the relative angle for Twist message in ROS      *
     *************************************************************************/
    void pid_twistA();
    
    /*************************************************************************
     * This method computes the relative velocity for Twist message in ROS   *
     *************************************************************************/
    void pid_twistV();
    
    /*************************************************************************
     * This method computes the angular position for Twist message in ROS    *
     *************************************************************************/
    double getTwistZ();
    
    /*************************************************************************
     * This method calculates the relative position of the chair everytime   * 
     * the encoders reset by setting its old position as the origin to       *
     * calculate the new position. Moreover, this function is important to   *
     * be able to publish (send) the information to ROS                      *
     *************************************************************************/
    void odomMsg();
    
    /*************************************************************************
     * This method prints the Odometry message to the serial monitor         *
     *************************************************************************/
    void showOdom();

    /*************************************************************************
     * (not being used) Functions with a predetermined path (demo)           *
     *************************************************************************/
    void desk();
    void kitchen();
    void desk_to_kitchen();

    /*************************************************************************
    *                         Public Variables                               *       
    **************************************************************************/ 
    double x_position;
    double y_position;
    double z_angular;
    double curr_vel;
    double z_twistA;
    double linearV;
    double angularV;
    double vel;
    double test1, test2;
    bool forwardSafety;
    bool backwardSafety;        //Check if can move backward
    bool leftSafety;            //to check if can turn left
    bool rightSafety;           //to check if can turn right
    double curr_yaw, curr_velS; // Variable that contains current relative angle

    int* LFF = &ToFV[11];  	//Left Front Forward
    int* LFS = &ToFV[10]; 	//Left Front Side
	int* LFA = &ToFV[13];  	//Left Front Angle
	int* LFD = &ToFV[9]; 	//Left Front Down

	int* RFF = &ToFV[7];  	//Right Front Forward
	int* RFS = &ToFV[8];  	//Right Front Side
	int* RFA = &ToFV[12]; 	//Right Front Angle
	int* RFD = &ToFV[6];  	//Right Front Down

	int* LBB = &ToFV[5];  	//Left Back Forward
	int* LBS = &ToFV[3];  	//Left Back Side
	int* LBD = &ToFV[4];  	//Left Back Down

	int* RBB = &ToFV[0];  	//Right Back Forward
	int* RBS = &ToFV[1];  	//Right Back Side
	int* RBD = &ToFV[2];  	//Right Back Down

    double currAngularVelocity;
    double angle;
    double arcLength;



private:

    /************************************************************************
     * Expected data used to compare whether of not there is a ledge.       *
     * This serves as a ground base. Array is used for calibrating the      *
     * time of flight sensors, which is used to calculate stdev and mean on * 
     ************************************************************************/
    int runningAverage[4];	//Array to store angled ToF reading averages

    /* Pointers for the joystick speed */
    PwmOut* x;
    PwmOut* y;

    /* Pointers for PCB */
    PwmOut* on;
    PwmOut* off;

    DigitalIn* e_button;                // Pointer to emergency button

    BNO080Wheelchair* imu;           	// Pointer to IMU
    Serial* out;                        // Pointer to Serial Monitor
    Timer* ti;                          // Pointer to timer
    QEI* wheel;                         // Pointer to left encoder
    QEI* wheelS;                        // Pointer to right encoder
    VL53L1X** ToF;                      // Arrays of pointers to ToF sensors
    int ToFV[12] = {0,0,0,0,0,0,0,0,0,0,0,0};						// Array to store ToF readings, iitialize to 0

};
#endif
