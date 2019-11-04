/*************************************************************************
*             Importing header into wheelchair.cpp                       *
**************************************************************************/
#include "wheelchair.h"
//#include <math.h>

/*************************************************************************
*                     Defining global variables                          *
**************************************************************************/
bool manual_drive = false;                                                             // Variable changes between joystick and auto drive
double encoder_distance;                                                               // Keeps distanse due to original position
double curr_yaw_raw;
volatile double Setpoint, Output, Input, Input2;                                       // Variables for PID
volatile double pid_yaw, Distance, Setpoint2, Output2, encoder_distance2;              // Variables for PID
volatile double vIn, vOut, vDesired;                                                   // Variables for PID Velosity
volatile double vInS, vOutS, vDesiredS;                                                // Variables for PID Slave Wheel
volatile double yIn, yOut, yDesired;                                                   // Variables for PID turn velosity
//    int* ToFDataPointer1;
//    int* ToFDataPointer2;

int ledgeArrayLF[150];
int ledgeArrayRF[150];
int* ToFDataPointer1 = ledgeArrayLF;
int* ToFDataPointer2 = ledgeArrayRF;
Statistics LFDStats(ToFDataPointer1, 149, 1);
Statistics RFDStats(ToFDataPointer2, 149, 1);
int k1 = 0;     //Number of samples

int ledgeArrayLB[150];
int ledgeArrayRB[150];
int* ToFDataPointer3 = ledgeArrayLB;
int* ToFDataPointer4 = ledgeArrayRB;
Statistics LBTStats(ToFDataPointer3, 149, 1);
Statistics RBTStats(ToFDataPointer4, 149, 1);
int k2 = 0;     //Number of samples

double dist_old, curr_pos, curr_posS;                                                             // Variables for odometry position
double outlierToF[4];
double sensors3;
volatile double wallDistance = 497;
// Reads from the ToF Sensors

/*************************************************************************
*                        Creating PID objects                            *
**************************************************************************/
PID PIDFollowRight(&sensors3, &vOutS, &wallDistance, 5.5, .002, .002, P_ON_E, DIRECT);
PID myPID(&pid_yaw, &Output, &Setpoint, 5.5, .00, 0.0036, P_ON_E, DIRECT);             // Angle PID object constructor
PID myPIDDistance(&Input, &Output, &Setpoint, 5.5, .00, 0.002, P_ON_E, DIRECT);        // Distance PID object constructor
PID PIDVelosity(&vIn, &vOut, &vDesired, 5.5, .00, .002, P_ON_E, DIRECT);               // Velosity PID Constructor
PID PIDSlaveV(&vInS, &vOutS, &vDesiredS, 5.5, .00, .002, P_ON_E, DIRECT);              // Slave Velosity PID Constructor
PID PIDAngularV(&yIn, &yOut, &yDesired, 5.5, .00, .002, P_ON_E, DIRECT);               // Angular Velosity PID Constructor


/*************************************************************************
*            Thread measures current angular position                    *
**************************************************************************/
void Wheelchair::compass_thread()
{
	curr_yaw = imu->yaw();
	z_angular = curr_yaw;
}

/*************************************************************************
*      Thread measures velocity of wheels and distance traveled          *
**************************************************************************/
void Wheelchair::velocity_thread()
{
	curr_vel = wheel->getVelocity();
	curr_velS = wheelS->getVelocity();
	curr_pos = wheel->getDistance(53.975);
	curr_posS = wheelS->getDistance(53.975);
}

void Wheelchair::emergencyButton_thread()
{
	//out -> printf("I RUN E BUTTON\n");


	if (!(e_button->read())) {//change once button is connected
		out->printf("E-button has been pressed\r\n");
		//Stop wheelchair
		//Wheelchair::stop();
		//out->printf("E-button has been pressed\r\n");
		//off->write(high);                              // Turn off PCB
		//on->write(0);                                  // Make sure PCB not on
		//Reset Board
		NVIC_SystemReset();

	}
	else {
		//out->printf("\r e_button has not been pressed\r\n");
	}


}


void Wheelchair::forwardSafety_thread() {
	//out -> printf("HI forwardSafety is RUNNING\n");
		/*************************************************************************
	 *           				Forward Safety						         *
	 *************************************************************************/

	ToFV[7] = (*(ToF + 7))->readFromOneSensor();      //RFF
	ToFV[11] = (*(ToF + 11))->readFromOneSensor();  //LFF

	 //CASE 1(A)
	if (curr_vel < 0.5 && ((2 * maxDecelerationSlow * (*LFF) < curr_vel * curr_vel * 1000 * 1000 ||
		2 * maxDecelerationSlow * (*RFF) < curr_vel * curr_vel * 1000 * 1000) &&
		(*LFF < 800 || *RFF < 800))) {

		if (x->read() > def) {
			x->write(def);
			forwardSafety = 1;      		// You cannot move forward
			out->printf("Enabled Forward Safety, Case 1 - A (FORMULA)\n");
		}
	}

	// CASE 1(B)
	else if (550 > * LFF || 550 > * RFF) {
		if (x->read() > def) {
			x->write(def);
			forwardSafety = 1;        		// You cannot move forward
			out->printf("Enabled Forward Safety, Case 1 - B (DISTANCE)\n");
		}
	}

	// CASE 2
	else if (curr_vel > 0.5 && curr_vel < 0.9 && ((2 * maxDecelerationFast * (*LFF) < curr_vel * curr_vel * 1000 * 1000 ||
		2 * maxDecelerationFast * (*RFF) < curr_vel * curr_vel * 1000 * 1000) &&
		(*LFF < 1300 || *RFF < 1300)) /*|| 550 > *LFF || 550 > *RFF */) {
		if (x->read() > def) {
			x->write(def);
			forwardSafety = 1;       	// You cannot move forward
			out->printf("Enabled Forward Safety, Case 2\n");
		}
	}

	// CASE 3
	else if ((curr_vel > 0.9) && (*LFF < 1700 || *RFF < 1700)) {
		if (x->read() > def) {
			x->write(def);
			forwardSafety = 1;   	// You cannot move forward
			out->printf("Enabled Forward Safety, Case 3\n");
		}
	}

	else
		forwardSafety = 0;
}

void Wheelchair::backwardSafety_thread() {
	//out -> printf("HI backwardSafety is RUNNING\n");

	/*************************************************************************
	 *           				Backward Safety						         *
	 *************************************************************************/
	 // CASE 1 (A)
	 //out -> printf("I RAN");

	ToFV[5] = (*(ToF + 5))->readFromOneSensor();  //LBB
	ToFV[0] = (*(ToF + 0))->readFromOneSensor();  //RBB

	if (-curr_vel < 0.5 && ((2 * maxDecelerationSlow * (*LBB) < curr_vel * curr_vel * 1000 * 1000 ||
		2 * maxDecelerationSlow * (*RBB) < curr_vel * curr_vel * 1000 * 1000) &&
		(*LBB < 800 || *RBB < 800))) {

		if (x->read() < def) {
			x->write(def);
			backwardSafety = 1;          	// You cannot move backwards
			out->printf("Enabled Backward Safety, Case 1 - A (FORMULA)\n");
		}
	}

	// CASE 1 (B)
	else if (550 > * LBB || 550 > * RBB) {
		if (x->read() < def) {
			x->write(def);
			backwardSafety = 1;         		// You cannot move backwards
			out->printf("Enabled Backward Safety, Case 1 - B (DISTANCE)\n");
		}
	}

	// CASE 2
	else if (-curr_vel > 0.5 && -curr_vel < 0.9 && ((2 * maxDecelerationFast * (*LBB) < curr_vel * curr_vel * 1000 * 1000 ||
		2 * maxDecelerationFast * (*RBB) < curr_vel * curr_vel * 1000 * 1000) &&
		(*LBB < 1300 || *RBB < 1300))) {
		if (x->read() < def) {
			x->write(def);
			backwardSafety = 1;     	// You cannot move backwards
			out->printf("Enabled Backward Safety, Case 2 (MEDIUM SPEED)\n");
		}
	}

	// CASE 3
	else if ((-curr_vel > 0.9) && (*LBB < 1700 || *RBB < 1700)) {
		if (x->read() < def) {
			x->write(def);
			backwardSafety = 1;   	// You cannot move backwards
			out->printf("Enabled Backward Safety, Case 3 (HIGH SPEED)\n");
		}
	}

	else
		backwardSafety = 0;

}

void Wheelchair::leftSideSafety_thread() {

	/*************************************************************************
	 *              			Left Side Safety		                     *
	 *************************************************************************/

	 //out -> printf("x read: %f ,     y read: %f \n", x->read(), y ->read());

	 //out -> printf("LFS: %d \n arcLength: %f \n minWallLengthLeft %d \n currAngularVelocity %f \n", *LFS, arcLength, minWallLengthLeft, currAngularVelocity);

	 //CASE 1 (A): When standing still

	ToFV[10] = (*(ToF + 10))->readFromOneSensor(); //LFS
	ToFV[3] = (*(ToF + 3))->readFromOneSensor(); //LBS
	ToFV[13] = (*(ToF + 13))->readFromOneSensor(); //LFA

	if ((*LFS <= minWallLengthLeft) || (*LBS <= minWallLengthLeft)) {
		if (y->read() > def) {
			y->write(def);
			leftSafety = 1;			//Not safe to turn left
			//out->printf("Detecting wall to the left!\n");
		}
	}

	//CASE 1 (B): Blind spot, obstacle in the middle side of the wheelchair
	else if ((*LFA) <= 100) {
		if (y->read() > def) {
			y->write(def);
			leftSafety = 1;			//Not safe to turn left
			//out->printf("Blindspot on the left side\n");
		}
	}

	//    //CASE EXTRA
	//    else if((*LFF) <= 200) {
	//    	if(y->read() > def) {
	//    	                y->write(def);
	//    	leftSafety = 1;			//Not safe to turn left
	//        out->printf("Footrest probs\n");
	//    	}
	//    }

		//CASE 2 (A):
	else if (/*(currAngularVelocity * currAngularVelocity > 2 *
		maxAngularDeceleration * angle) && */ fabs(currAngularVelocity) > 0.8 && ((*LFS) / 10 <= arcLength + minWallLengthLeft / 10 + 80) || (*LFF) <= 230) {
		if (y->read() > def) {
			y->write(def);
			leftSafety = 1; 		//Not safe to turn left
			//out->printf("CASE 2 FAST FAST LEFT\n");
		}
	}

	//CASE 3):
	else if (/*(currAngularVelocity * currAngularVelocity > 2 *
		maxAngularDeceleration * angle) && */ ((*LFS) / 10 <= arcLength + minWallLengthLeft / 10 + 13) || (*LFF) <= 230) {
		if (y->read() > def) {
			y->write(def);
			leftSafety = 1; 		//Not safe to turn left
			//out->printf("CASE 3 LEFT\n");
		}
	}



	//    //CASE 2 (B)
	//    else if(/*(currAngularVelocity * currAngularVelocity > 2 *
	//        maxAngularDeceleration * angle) && */((*LBS)/10 <= arcLength + minWallLengthLeft/10)) {
	//    	if(y->read() > def) {
	//    	    	                y->write(def);
	//    	leftSafety = 1; 		//Not safe to turn left
	//        out-> printf("Too fast to the left! Back left area danger \n");
	//    	}
	//    }

	//    //CASE 2 (C)
	//    else if(/*(currAngularVelocity * currAngularVelocity > 2 *
	//        maxAngularDeceleration * angle) && */ ((*LFA)/10 <= arcLength + minWallLengthLeft/10)) {
	//    	if(y->read() > def) {
	//    	    	                y->write(def);
	//    	leftSafety = 1; 		//Not safe to turn left
	//        out->printf("Too fast to the left!, blindspot \n");
	//    	}
	//    }

	else {
		leftSafety = 0;
	}

}

void Wheelchair::imuRead_thread() {
	currAngularVelocity = imu->gyro_z(); //Current angular velocity from IMU
	angle = imu->yaw() * 3.14159 / 180; 	//from IMU, in rads
	arcLength = WheelchairRadius * currAngularVelocity *
		currAngularVelocity / (2 * maxAngularDeceleration); //S = r*theta, in cm
															 // Stopping distance from the wall
}

void Wheelchair::rightSideSafety_thread() {

	//out->printf("HI rightSafety is RUNNING\n");

	/*************************************************************************
	 *              Side Time of Flight sensors detection                    *
	 *************************************************************************/

	ToFV[8] = (*(ToF + 8))->readFromOneSensor(); //RFS
	ToFV[1] = (*(ToF + 1))->readFromOneSensor(); //RBS
	ToFV[12] = (*(ToF + 12))->readFromOneSensor(); //RFA

	out -> printf("LFS: %d, LFF %d, LFA %d, LFD %d\n", *LFS, *LFF, *LFA, *LFD);

	if ((*RFS <= minWallLengthRight) || (*RBS <= minWallLengthRight)) {
		if (y->read() < def) {
			y->write(def);
			rightSafety = 1;
			out->printf("CASE 1(A): Detecting wall to the right!\n");
		}
	}

	//CASE 1 (B)
	else if ((*RFA) <= 100) {	//Number` needs to be changed based on testing
		if (y->read() < def) {
			y->write(def);
			rightSafety = 1;
			out->printf("RFA: %d\n", *RFA);
			out->printf("CASE 1(B): Blindspot on the right side\n");
		}
	}

	//CASE 3):
	else if (((*RFS) / 10 <= arcLength + minWallLengthRight / 10 + 13) || (*RFF) <= 230) {
		if (y->read() < def) {
			y->write(def);
			out->printf("CASE 2\n");
			rightSafety = 1; 		//Not safe to turn RIGHT

		}
		out->printf("CASE 3 RIGHT\n");
	}

	// //CASE 2 (A)
	// else if((currAngularVelocity * currAngularVelocity > 2 *
	//     maxAngularDeceleration * angle) && ((*RFS)/10 <= arcLength + minWallLengthRight/10)) {
	// 	if(y->read() < def) {
	// 	y->write(def);
	// 	rightSafety = 1; 		//Not safe to turn right
	//     out->printf("Too fast to the right!\n");
	// 	}
	// }

	// //CASE 2 (B)
	// else if((currAngularVelocity * currAngularVelocity > 2 *
	//     maxAngularDeceleration * angle) && ((*RBS)/10 <= arcLength + minWallLengthRight/10)) {
	// 	if(y->read() < def) {
	// 	    	    	    	                y->write(def);
	// 	rightSafety = 1; 		//Not safe to turn right
	//     out-> printf("Too fast to the right! Back right area danger\n");
	// }
	// }

	// //CASE 2 (C)
	// else if((currAngularVelocity * currAngularVelocity > 2 *
	//     maxAngularDeceleration * angle) && ((*RFA)/10 <= arcLength + minWallLengthRight/10)) {
	// 	if(y->read() < def) {
	// 	    	    	    	                y->write(def);
	// 	rightSafety = 1; 		//Not safe to turn right
	//     out->printf("Too fast to the right!, blindspot \n");
	// 	}
	// }

	else {
		rightSafety = 0;
	}
}


void Wheelchair::ledgeSafety_thread() {
	

	ToFV[9] = (*(ToF + 9))->readFromOneSensor(); //LFD
	ToFV[6] = (*(ToF + 6))->readFromOneSensor(); //RFD
	ToFV[4] = (*(ToF + 4))->readFromOneSensor(); //LBD
	ToFV[2] = (*(ToF + 2))->readFromOneSensor(); //RBD
	/*************************************************************************
   *           Ledge Detection for the Back Time of Flight Sensors         *
   *************************************************************************/

	k2++;

	if (k2 == 150) {
		k2 = 0;
	}

	ledgeArrayLB[k2] = ToFV[4];
	ledgeArrayRB[k2] = ToFV[2];

	outlierToF[2] = LBTStats.mean() + 2 * LBTStats.stdev();
	outlierToF[3] = RBTStats.mean() + 2 * RBTStats.stdev();

	runningAverage[2] = ((runningAverage[2] * (4) + (*LBD)) / 5);
	runningAverage[3] = ((runningAverage[3] * (4) + (*RBD)) / 5);


	//out -> printf("HI ledgeSafety is RUNNING\n");
	k1++;

	if (k1 == 150) {
		k1 = 0;
	}

	/**************************************************************************
	 *         Ledge Detection for the front Time of Flight Sensors           *
	 **************************************************************************/

	ledgeArrayLF[k1] = ToFV[9];
	ledgeArrayRF[k1] = ToFV[6];

	outlierToF[0] = LFDStats.mean() + 2 * LFDStats.stdev();
	outlierToF[1] = RFDStats.mean() + 2 * RFDStats.stdev();

	runningAverage[0] = ((runningAverage[0] * (4) + *LFD) / 5);
	runningAverage[1] = ((runningAverage[1] * (4) + *RFD) / 5);
}


/*************************************************************************
*      Thread checks ToF sensors for safety of wheelchair movement       *
**************************************************************************/
void Wheelchair::ToFSafe_thread()
{
	//out -> printf("I RAM SAFTEY\n");

	//out->printf("%d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\t %d\n", *RBB, *RBS, *RBD, *LBS, *LBD, *LBB, *RFS, *RFF, *RFD, *LFF, *LFD, *LFS, *LFA, *RFA);
	//out->printf("\n");
	//out->printf("Encoder 2 TEST = %f\n", wheel->getDistance(53.975));
	//out->printf("Encoder 1 TEST = %f\n", wheelS->getDistance(53.975));

	/**************************************************************************
	 *                      ToF ARRAY ASSIGNMENTS
	 *           (from the perspective of user seated on wheelchair)
	 *
	 *   Each ToF has a 3 letter name, the first indicates left/right, the
	 *   second front/back, and third, the specific ToF sensor
	 *   L = Left, R = Right
	 *   F = Front, B = Back
	 *   D = Down, F = Forward, Side = S
	 *   eg: (*LBB) means Left side, Back end, Bottom ToF
	 *
	 *   FRONT - LEFT
	 *   ToF 10	- Down (Angle)		LFD
	 *   ToF 9	- Bottom (Front)	LFF
	 *   ToF 11	- Side				LFS */



}

/*************************************************************************
*                   Constructor for Wheelchair class                     *
**************************************************************************/
Wheelchair::Wheelchair(PinName xPin, PinName yPin, Serial* pc, Timer* time, QEI* qei, QEI* qeiS,
	VL53L1X** ToFT, DigitalIn* e1_button) : e_button(e1_button)
{
	x_position = 0;
	y_position = 0;
	forwardSafety = 0;
	backwardSafety = 0;

	/* Initializes X and Y variables to Pins */
	x = new PwmOut(xPin);
	y = new PwmOut(yPin);

	/* Initializes IMU Library */
	out = pc;                                                                           // "out" is called for serial monitor
	out->printf("on\r\n");
	//    imu = new IMUWheelchair(pc, time);
	imu = new BNO080Wheelchair(pc, time);
	Wheelchair::stop();                                                                 // Wheelchair is initially stationary
	wheelS = qeiS;                                                                      // "wheel" is called for encoder
	wheel = qei;
	ToF = ToFT;                                                                         // passes pointer with addresses of ToF sensors

	for (int i = 0; i < 14; i++) {                                                       // initializes the ToF Sensors
		(*(ToF + i))->initReading(0x31 + ((0x02) * i), 50000);
	}

	out->printf("wheelchair setup done \r\n");                                          // Make sure it initialized; prints in serial monitor
	ti = time;
	for (int i = 0; i < 10; i++) {
		(*(ToF + 1))->readFromOneSensor();
		(*(ToF + 1))->readFromOneSensor();
	}
	for (int i = 0; i < 150; i++) {
		ledgeArrayLF[i] = (*(ToF + 10))->readFromOneSensor();
		ledgeArrayRF[i] = (*(ToF + 8))->readFromOneSensor();
	}

	outlierToF[0] = LFDStats.mean() + 2 * LFDStats.stdev();
	outlierToF[1] = RFDStats.mean() + 2 * RFDStats.stdev();
	imu->setup();                                                                       // turns on the IMU

	myPID.SetMode(AUTOMATIC);                                                           // PID mode: Automatic
}

/*************************************************************************
*               Move wheelchair with joystick on manual mode             *
**************************************************************************/
void Wheelchair::move(float x_coor, float y_coor)
{
	/* Scales one joystick measurement to the chair's joystick measurement */
	float scaled_x = ((x_coor * 1.6f) + 1.7f) / 3.3f;
	float scaled_y = (3.3f - (y_coor * 1.6f)) / 3.3f;

	/* Sends the scaled joystic values to the chair */
	x->write(scaled_x);
	y->write(scaled_y);
}


/*************************************************************************
*  Automatic mode: move forward and update x,y coordinate sent to chair  *
**********e****************************************************************/
void Wheelchair::forward()
{
	//printf("current velosity; %f, curr vel S %f\r\n", curr_vel, curr_velS);

	if (forwardSafety == 0) {
		x->write(high);
		y->write(def + 1.5 * offset);

		//double diff = fmod((wheel->getDistance(53.975)) - (wheelS->getDistance(53.975)), 100);
		//y->write(def-(diff/1000));

	}
	//out->printf("%f, %f\r\n", curr_pos, wheelS->getDistance(53.975));
}

/*************************************************************************
* Automatic mode: move in reverse and update x,y coordinate sent to chair*
**************************************************************************/
void Wheelchair::backward()
{
	if (backwardSafety == 0) {
		x->write(low);
		y->write(def + 1.5 * offset);
	}
	//out->printf("%f, %f\r\n", curr_vel, curr_velS);
}
/*************************************************************************
*   Automatic mode: move right and update x,y coordinate sent to chair   *
**************************************************************************/
void Wheelchair::right()
{
	//if safe to move, from ToFSafety
	if (rightSafety == 0) {
		x->write(def);
		y->write(low);
	}
}
/*************************************************************************
*   Automatic mode: move left and update x,y coordinate sent to chair   *
**************************************************************************/
void Wheelchair::left()
{
	//if safe to move, from ToFSafety
	if (leftSafety == 0) {
		x->write(def);
		y->write(high);
	}
}
/*************************************************************************
*                           Stop the wheelchair                          *
**************************************************************************/
void Wheelchair::stop()
{
	x->write(def);
	y->write(def);
}
/*************************************************************************
 * Counter-clockwise is ( - )                                            *
 * Clockwise is ( + )                                                    *
 * Range of deg: 0 to 360                                                *
 * This method takes in an angle from user and adjusts for turning right *
**************************************************************************/
void Wheelchair::pid_right(int deg)
{
	bool overturn = false;                                                              // Boolean if angle over 360

	out->printf("pid right\r\r\n");
	x->write(def);                                                                      // Update x sent to chair to be stationary
	Setpoint = curr_yaw + deg;                                                          // Relative angle we want to turn
	pid_yaw = curr_yaw;                                                                 // Sets pid_yaw to angle input from user

	/* Turns on overturn boolean if setpoint over 360 */
	if (Setpoint > 360) {
		overturn = true;
	}

	myPID.SetTunings(5.5, 0, 0.0035);                                                    // Sets the constants for P and D
	myPID.SetOutputLimits(0, def - low - .15);                                              // Limit is set to the differnce between def and low
	myPID.SetControllerDirection(DIRECT);                                               // PID mode: Direct

	/* PID stops when approaching a litte less than desired angle */
	while (pid_yaw < Setpoint - 3) {
		/* PID is set to correct angle range if angle greater than 360*/
		if (overturn && curr_yaw < Setpoint - deg - 1) {
			pid_yaw = curr_yaw + 360;
		}
		else {
			pid_yaw = curr_yaw;
		}

		myPID.Compute();                                                                // Does PID calculations
		double tempor = -Output + def;                                                    // Temporary value with the voltage output
		y->write(tempor);                                                               // Update y sent to chair

		/* Prints to serial monitor the current angle and setpoint */
		out->printf("curr_yaw %f\r\r\n", curr_yaw);
		out->printf("Setpoint = %f \r\n", Setpoint);

		//wait(.05);                                                                      // Small delay (milliseconds)
	}

	/* Saftey stop for wheelchair */
	Wheelchair::stop();
	out->printf("done \r\n");
}
/*************************************************************************
* Counter-clockwise is ( - )                                            *
* Clockwise is ( + )                                                    *
* Range of deg: 0 to 360                                                *
* This method takes in an angle from user and adjusts for turning left  *
**************************************************************************/
void Wheelchair::pid_left(int deg)
{
	bool overturn = false;                                                              //Boolean if angle under 0

	out->printf("pid Left\r\r\n");
	x->write(def);                                                                      // Update x sent to chair to be stationary
	Setpoint = curr_yaw - deg;                                                          // Relative angle we want to turn
	pid_yaw = curr_yaw;                                                                 // Sets pid_yaw to angle input from user

	/* Turns on overturn boolean if setpoint less than 0 */
	if (Setpoint < 0) {
		overturn = true;
	}

	myPID.SetTunings(5, 0, 0.004);                                                       // Sets the constants for P and D
	myPID.SetOutputLimits(0, high - def - .12);                                              // Limit is set to the differnce between def and low
	myPID.SetControllerDirection(REVERSE);                                              // PID mode: Reverse

	/* PID stops when approaching a litte more than desired angle */
	while (pid_yaw > Setpoint + 3) {
		/* PID is set to correct angle range if angle less than 0 */
		if (overturn && curr_yaw > Setpoint + deg + 1) {
			pid_yaw = curr_yaw - 360;
		}
		else {
			pid_yaw = curr_yaw;
		}

		myPID.Compute();                                                                // Does PID calculations
		double tempor = Output + def;                                                     // Temporary value with the voltage output
		y->write(tempor);                                                               // Update y sent to chair

		/* Prints to serial monitor the current angle and setpoint */
		out->printf("curr_yaw %f\r\n", curr_yaw);
		out->printf("Setpoint = %f \r\n", Setpoint);

		wait(.05);                                                                      // Small delay (milliseconds)
	}

	/* Saftey stop for wheelchair */
	Wheelchair::stop();
	out->printf("done \r\n");

}

/*************************************************************************
*          This method determines whether to turn left or right          *
**************************************************************************/
void Wheelchair::pid_turn(int deg)
{

	/*****************************************************************
	 * Sets angle to coterminal angle for left turn if deg > 180     *
	 *  Sets angle to coterminal angle for right turn if deg < -180  *
	 *****************************************************************/
	if (deg > 180) {
		deg -= 360;
	}
	else if (deg < -180) {
		deg += 360;
	}

	/* Makes sure angle inputted to function is positive */
	int turnAmt = abs(deg);

	/* Calls PID_right if deg > 0, else calls PID_left if deg < 0 */
	if (deg >= 0) {
		Wheelchair::pid_right(turnAmt);
	}
	else {
		Wheelchair::pid_left(turnAmt);
	}

}

/*************************************************************************
*   This method takes in distance to travel and adjust to move forward   *
**************************************************************************/
void Wheelchair::pid_forward(double mm)
{
	mm -= 20;                                                                           // Makes sure distance does not overshoot
	Input = 0;                                                                          // Initializes input to zero: Test latter w/o
	wheel->reset();                                                                     // Resets encoders so that they start at 0

	out->printf("pid foward\r\n");

	double tempor;                                                                      // Initializes Temporary variable for x input
	Setpoint = mm;                                                                      // Initializes the setpoint to desired value

	myPIDDistance.SetTunings(5.5, 0, 0.0015);                                            // Sets constants for P and D
	myPIDDistance.SetOutputLimits(0, high - def - .15);                                      // Limit set to difference between high and def
	myPIDDistance.SetControllerDirection(DIRECT);                                       // PID mode: Direct

	y->write(def + offset);                                                               // Update y to make chair stationary

	/* Chair stops moving when Setpoint is reached */
	while (Input < Setpoint) {

		if (out->readable()) {                                                           // Emergency Break
			break;
		}

		Input = wheel->getDistance(53.975);                                             // Gets distance from Encoder into PID
		out->printf("Encoder = %f\n", Input);
		//out->printf(Input);
		wait(.05);                                                                      // Slight Delay: *****Test without


		myPIDDistance.Compute();                                                        // Compute distance traveled by chair

		tempor = Output + def;                                                          // Temporary output variable
		x->write(tempor);                                                               // Update x sent to chair

		/* Prints to serial monitor the distance traveled by chair */
		out->printf("distance %f\r\n", Input);
	}

}

/*************************************************************************
*        This method returns the relative angular position of chair      *
**************************************************************************/
double Wheelchair::getTwistZ()
{
	return 0;
	//  return imu->gyro_z();
}

/*************************************************************************
*    This method computes the relative angle for Twist message in ROS    *
**************************************************************************/
void Wheelchair::pid_twistA()
{
	/* Initialize variables for angle and update x,y sent to chair */
	char c;
	double temporA = def;
	y->write(def);
	x->write(def);

	PIDAngularV.SetTunings(.00015, 0, 0.00);                                             // Sets the constants for P and D
	PIDAngularV.SetOutputLimits(-.1, .1);                                               // Limit set to be in range specified
	PIDAngularV.SetControllerDirection(DIRECT);                                         // PID mode: Direct

	/* Computes angular position of wheelchair while turning */
	while (1) {
		yDesired = 180; //angularV;

		/* Update and set all variable so that the chair is stationary
		 * if the desired angle is zero
		 */
		if (out->readable())
			return;
		if (yDesired == 0) {
			x->write(def);
			y->write(def);
			yDesired = 0;
			return;
		}

		/* Continuously updates with current angle measured by IMU */
		yIn = imu->gyro_z() * (180 / 3.1415926);
		out->printf("%lf\r\n", yIn);
		PIDAngularV.Compute();
		temporA += yOut;                                                                // Temporary value with the voltage output
		y->write(temporA);                                                              // Update y sent to chair

		//out->printf("temporA: %f, yDesired %f, angle: %f\r\n", temporA, yDesired, imu->gyro_z());
		wait(.05);                                                                      // Small delay (milliseconds)
	}

}

/*************************************************************************
*   This method computes the relative velocity for Twist message in ROS  *
**************************************************************************/
void Wheelchair::pid_twistV()
{
	/* Initializes variables as default */
	double temporV = def;
	double temporS = def + 1.5 * offset;
	vDesiredS = 0;
	double initAngle = curr_yaw;
	x->write(def);
	y->write(def + 1.5 * offset);            //added 1.5*offset
	wheel->reset();
	wheelS->reset();// Resets the encoders
	/* Sets the constants for P and D */
	//PIDVelosity.SetTunings(.0005,0, 0.00);			//0.0005
	//PIDSlaveV.SetTunings(.005,0.000001, 0.000001); //0.005

	PIDVelosity.SetTunings(.0002, 0, 0.00);			//0.0005
	PIDSlaveV.SetTunings(.000002, 0.000001, 0.001); 	//0.005

	/* Limits to the range specified */
	PIDVelosity.SetOutputLimits(-.005, .005);
	PIDSlaveV.SetOutputLimits(-.0002, .0002);

	/* PID mode: Direct */
	PIDVelosity.SetControllerDirection(DIRECT);
	PIDSlaveV.SetControllerDirection(DIRECT);

	while (1) {
		linearV = .8;
		vel = curr_vel;
		vDesired = linearV * 100;
		if (out->readable())
			return;
		/* Update and set all variable so that the chair is stationary
		* if the velocity is zero
		*/
		if (linearV == 0) {
			x->write(def);
			y->write(def);

			vel = 0;
			vDesired = 0;
			dist_old = 0;
			return;
		}

		if (vDesired >= 0) {
			PIDVelosity.SetTunings(.000004, 0, 0.00);                                    //0.000004 Sets the constants for P and D
			PIDVelosity.SetOutputLimits(-.002, .002);                                   // Limits to the range specified
		}
		else {
			PIDVelosity.SetTunings(.000015, 0, 0.00);                                    // Sets the constants for P and D
			PIDVelosity.SetOutputLimits(-.0005, .0005);                                 // Limits to range specified
		}

		/* Sets maximum value of variable to 1 */
		if (temporV >= 1.5) {
			temporV = 1.5;
		}

		/* Scales and makes some adjustments to velocity */

		vIn = curr_vel * 100;
		vInS = (-curr_yaw + initAngle) / 5000;//(curr_pos-curr_posS)/1000;
		//vIn = curr_pos*100;

		PIDVelosity.Compute();
		PIDSlaveV.Compute();
		//  if(forwardSafety == 0) {
		temporV += vOut;
		temporS += vOutS;

		vInS = (curr_pos - curr_posS) / 1000;
		PIDSlaveV.Compute();
		temporS += vOutS;

		/* Updates x,y sent to Wheelchair and for Odometry message in ROS */
		x->write(temporV);
		test2 = temporV;
		y->write(temporS);
		/* } else {
			 x->write(def);
			 y->write(def);
		 }*/
		 //  out->printf("Velosity: %f, Velosity2: %f, temporV %f, temporS %f, posM %f, posS %f \r\n", curr_vel, curr_velS, temporV, temporS, vOut, curr_posS);
		Wheelchair::odomMsg();
		wait(.005);                                                                      // Small delay (milliseconds)
	}
}
void Wheelchair::pid_wall_follower()
{
	//    out->printf("Inside pid_wall_follower()\n");
	//
	//    /* Initializes variables as default */
	//    double temporV = def;
	//    double temporS = def+offset;
	//    wallDistance = 450;
	//    x->write(def);
	//    y->write(def);
	//    /* Sets the constants for P and D */
	//    PIDVelosity.SetTunings(.0005,0, 0.00);
	//    PIDFollowRight.SetTunings(.005,0.000001, 0.000001);
	//
	//    /* Limits to the range specified */
	//    PIDVelosity.SetOutputLimits(-.005, .005);
	//    PIDFollowRight.SetOutputLimits(-.002, .002);
	//
	//    /* PID mode: Direct */
	//    PIDVelosity.SetControllerDirection(DIRECT);
	//    PIDFollowRight.SetControllerDirection(DIRECT);
	//
	//    while(1) {
	//        linearV = .5;
	//        vel = curr_vel;
	//        vDesired = linearV*100;
	//        if(out->readable())
	//            return;
	//        /* Update and set all variable so that the chair is stationary
	//        * if the velocity is zero
	//        */
	//
	//        if(vDesired >= 0) {
	//            PIDVelosity.SetTunings(.000004,0, 0.00);                                    // Sets the constants for P and D
	//            PIDVelosity.SetOutputLimits(-.002, .002);                                   // Limits to the range specified
	//        } else {
	//            PIDVelosity.SetTunings(.000015,0, 0.00);                                    // Sets the constants for P and D
	//            PIDVelosity.SetOutputLimits(-.0005, .0005);                                 // Limits to range specified
	//        }
	//
	//        /* Sets maximum value of variable to 1 */
	//        if(temporV >= 1) {
	//            temporV = 1;
	//        }
	//        /* Scales and makes some adjustments to velocity */
	//        vIn = curr_vel*100;
	//        PIDVelosity.Compute();
	//        PIDFollowRight.Compute();
	//        if(forwardSafety == 0) {
	//            temporV += vOut;
	//            temporS += vOutS;
	//
	//            /* Updates x,y sent to Wheelchair and for Odometry message in ROS */
	//            x->write(temporV);
	//            y->write(temporS);
	//        } else {
	//            x->write(def);
	//            y->write(def);
	//        }
	//        //out->printf("Velosity: %f, Velosity2: %f, temporV %f, temporS %f\r\n", curr_vel, curr_velS, temporV, temporS);
	//        wait(.01);                                                                      // Small delay (milliseconds)
	//    }
	//

}
/*************************************************************************
* This method calculates the relative position of the chair everytime the*
* encoders reset by setting its old position as the origin to calculate  *
* the new position                                                       *
**************************************************************************/
void Wheelchair::odomMsg()
{
	double dist_new = curr_pos;
	double dist = dist_new - dist_old;
	double temp_x = dist * sin(z_angular * 3.14159 / 180);
	double temp_y = dist * cos(z_angular * 3.14159 / 180);

	x_position += temp_x;
	y_position += temp_y;

	dist_old = dist_new;
}

/**************************************************************************
*     This method prints the Odometry message to the serial monitor       *
***************************************************************************/
void Wheelchair::showOdom()
{
	out->printf("x %f, y %f, angle %f", x_position, y_position, z_angular);
}

/**************************************************************************
* This method returns the approximate distance based on the wheel diameter*
***************************************************************************/
float Wheelchair::getDistance()
{
	return wheel->getDistance(Diameter);
}

/**************************************************************************
*                   This method resets the Encoder's                      *
***************************************************************************/
void Wheelchair::resetDistance()
{
	wheel->reset();
}

/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/

/*Predetermined paths For Demmo*/
void Wheelchair::desk()
{
	imu->calibrate();
	/*
		Wheelchair::pid_forward(5461);
		Wheelchair::pid_right(87);
		Wheelchair::pid_forward(3658);
		Wheelchair::pid_right(87);
		Wheelchair::pid_forward(3658);
		*/
}

void Wheelchair::kitchen()
{
	Wheelchair::pid_forward(5461);
	Wheelchair::pid_right(87);
	Wheelchair::pid_forward(3658);
	Wheelchair::pid_left(90);
	Wheelchair::pid_forward(305);
}

void Wheelchair::desk_to_kitchen()
{
	Wheelchair::pid_right(180);
	Wheelchair::pid_forward(3700);
}
