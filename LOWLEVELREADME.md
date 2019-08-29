![alt text](https://github.com/SmartWheelchair/Systems/blob/master/Wheelchair%203D%20Part%20Images/UCSD_Wheelchair_Team_Logo.png "Logo")


# The Embedded System Team

## Low-level safety features

### Time of Flight (ToF) Safety Feature

We have used a total of 14 Time of Flight sensors for our kit, meant to be placed on strategic points on the wheelchair such that 
they can serve as obstacle detectors at critical points and stop the wheelchair when higher-level controls fail. Specifically, these
sensors can notify if the chair will run into an obstacle directly in front of it, directly behind it, and while turning, as well as
when the wheelchair is dangerously close to a ledge.

#### 1. Forward & Backward Safety
  
  Four sensors in total are used to implement the Forward and Backward Safety parts of the code.
  Two sensors each are placed on the front and rear (on both the left and right sides), facing frontwards. The front two sensors are specifically used for Forward Safety, while the two rear sensors are used for Backward Safety. The reading from the time of flight sensors is combined with the current velocity readings from the encoders to implement Forward & Backward Safety.
  
  **Forward Safety** is implemented using an if-else statement command, with 4 if/else if cases, apart from the default else case.
  If any of the if/else if cases are triggered, the variable forwardSafety is set to 1, which prevents the wheelchair from moving
  forward. If none of these cases are triggered, forwardSafety is set to 0 by default, which allows the wheelchair to move forward.
  In each of the following cases, obstacle detection can be done by either one or both of the two front time of flight sensors.
  
  **Case 1 (A)** is used when the wheelchair is travelling at low speeds. For the case to be triggered, three conditions must be met:
  1. The wheelchair is travelling at low speeds.
  2. The wheelchair is at a certain "reasonably close" distance from an obstacle.
  3. The velocity of the wheelchair is just within the range to stop it safely.
  
  The speed of the chair is determined from the reading from the encoders (in m/s). The distance from the obstacle is measured using the time of flight sensors (in mm). To determine if the velocity of the wheelchair is within the safe range, the following kinematic equation is used:
  
  <img src="https://latex.codecogs.com/gif.latex?2as%20%3D%20v%5E2%20-%20u%5E2">
  
  where a is the acceleration of the wheelchair (which is a fixed negative value, since the chair must decelerate to stop), v is the final velocity (which is 0 for the chair to stop), u is the initial velocity (which is the velocity at which the wheelchair is travelling before it begins to stop), and s is the stopping distance.
  
  Conditions 1 & 2 are determined based on testing. Condition 3 is met when, using the above equation,
  
  <img src="https://latex.codecogs.com/gif.latex?2as%20%3E%20-u%5E2">   or, 2 × maxDecelerationSlow × (Time of Flight Reading) < curr_vel × curr_vel × 1000 × 1000
  
  Note that maxDeceleration is a positive number, which is why the comparator is flipped.
  
  **Case 1 (B)** is used when the wheelchair is very close to an obstacle. This case depends solely on readings from the time of flight
  sensors, and enables forward safety when an obstacle is too close (based on testing).
  
  **Case 2** is used when the wheelchair is travelling at medium speeds. This works exactly like Case 1 (A), except the deceleration value is lower (called maxDecelerationFast) and the "reasonably close" distance value is determined based on testing the wheelchair at medium speeds.
  
  **Case 3** is used when the wheelchair is travelling at high speeds. This case depends solely on readings from the time of flight
  sensors, and enables forward safety when an obstacle is too close (based on testing at high speeds).
  
  **Backward Safety** is implemented exactly like Forward Safety, with symmetrical cases for the rear time of flight sensors. One key
  difference is that the current velocity is negative when the wheelchair is moving backwards; therefore, the velocity condition in any
  of the cases has an inverted comparator.

#### 2. Side Safety
  
  Six sensors in total are used to implement the Side Safety part of the code, which has two parts: Left Safety, and Right Safety.
  Two sensors each are placed on the front and rear (on both the left and right sides), facing sideways. Additionally, a sensor is      placed on each side on the front, facing backwards, into the area between the front and rear wheels of the wheelchair (called the blindspot sensor).
 The three sensors on the left side of the wheelchair are used for Left Safety, and the remaining three sensors on the right side of the wheelchair are used for Right Safety.
 The reading from the time of flight sensors is combined with the angular velocity, angle, and arclength readings from the encoders to implement Side Safety.
 
 ///Some description of these variables
 
 **Left Safety** is implemented using an if-else statement command, with 5 if/else if cases, apart from the default else case.
  If any of the if/else if cases are triggered, the variable leftSafety is set to 1, which prevents the wheelchair from turning
  left. If none of these cases are triggered, leftSafety is set to 0 by default, which allows the wheelchair to turn left.

**Case 1 (A)** is used when the wheelchair detects a closeby obstacle in its turning path. This case solely depends on readings from the time of flight sensors, specifically either one or both of the sideways facing sensors on the left side of the wheelchair.

When either/both of these sensors detect an obstacle closer than a fixed distance (called minWallLength), leftSafety is set to 1, preventing turning to the left.

**Case 1 (B)** is used when the wheelchair detects an obstacle in its left "blindspot" area, i.e., the area between the front and rear wheels of the wheelchair on its left side. 

When the blindspot sensor detects an obstacle closer than a fixed distance, leftSafety is set to 1, preventing turning to the left.

**Case 2 (A)** is used when the wheelchair is turning at an unsafe speed towards the left, using the left sensor on the front, facing sideways. 

For this case to be triggered, two conditions must be met:
1. The wheelchair detects an obstacle in its path that can potentially obstruct the movement of the wheelchair
2. The wheelchair is turning with angular velocity just within the range to stop it safely

The first condition is met when the reading from the side-facing time of flight sensor is smaller than the arcLength (plus an offset, which can be determined by testing), which indicates potential obstruction in the wheelchair's turning path.

The second condition is met when the velocity of the wheelchair is determined to be within the range to stop it before it hits the obstacle, as obtained from the following angular kinematics equation:

<img src="https://latex.codecogs.com/gif.latex?2%5Calpha%20%5Cleft%20%28%20%5CDelta%20%5Ctheta%20%5Cright%20%29%20%3D%20%5Comega%20%5E2%20-%20%5Comega%20_0%20%5E2">

where α is the angular acceleration of the wheelchair (which is a fixed negative value, since the chair must decelerate to stop), ω is the final angular velocity (which is 0 for the chair to stop turning), ω0 is the initial angular velocity (which is the angular velocity at which the wheelchair is travelling before it begins to stop turning), and Δθ is the stopping angle.

To ensure the angular velocity is within the range, we use the following comparison:

<img src="https://latex.codecogs.com/gif.latex?%5Comega%20%5E2%20%3C%202%5Calpha%28%20%5CDelta%20%5Ctheta%20%29"> or, currAngularVelocity × currAngularVelocity > 2 × maxAngularDeceleration × angle

  Note that maxAngularDeceleration is a positive number, which is why the comparator is flipped.

**Case 2 (B)** works exactly like Case 2 (A), except it uses side-facing sensor in the rear for obstacle detection.

**Case 2 (C)** works exactly like Case 2 (A), except it uses the left blindspot sensor for obstacle detection.

**Right Safety** works in an identical fashion as Left Safety, with symmetric cases using the sensors on the right side of the wheelchair.
 
### IMU (BNO080) Wrapper Function

A wrapper is simply a function that exists to call another function. Meaning we do not have to change the functionality of the main function we are calling from, if there are any changes needed we will make the necessary changes in the wrapper function. In the BNO wrapper function we call the necessary functions needed while making the code alot more clearer and easier to read. 

In our code BNO080.cpp and BNO080.h is the main function and BNO080Wheelchair.cpp and BNO080Wheelchair.h is the wrapper function. The wrapper function is about 600 lines shorter than the main function. As stated earlier, this makes it a lot easier to read and to make the necessary adjustments based on the team needs.

Parameters that we are able to receive from the IMU are accelerometer, gyroscope and magnetometers. The main two that we are using are the accelerometer and the gyroscope. We are using gyro_z  which is currAngularVelocity. We then place that variable in a kinematic equation in the wheelchair.cpp file.

#### BNO080Wheelchair::yaw() 

This method returns the total yaw.

We call the gyro_z() from the BNO080 and convert it from radians to degrees
We then calculate the total yaw based on the running time.

If the rotation is more than 360 deg then we go back to zero. In this sense, we wrap around from 0 to 360. Similarly, if rotation is less than -360 deg, we wrap around from -360 to 0.
  
### Watchdog Timer

TBA

### Emergency Stop Button
  
TBA

## Contact
**Website:** http://smartwheelchair.eng.ucsd.edu/  
**Phone:** (619) 836-8052  
**Email:** smartwheelchair.eng.ucsd.edu  
