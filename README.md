![alt text](https://github.com/SmartWheelchair/Systems/blob/master/Wheelchair%203D%20Part%20Images/UCSD_Wheelchair_Team_Logo.png "Logo")


# The Embedded System Team

## Our Mission
  
  Our goal is to use latest affordable, efficient sensors to create power kits that can be mounted on electric wheelchairs to give our
  users assistive autonomy. Moreover, we aim to deliver a fully functional sensor-integrated affordable modular design that can be replicated by others in a feasable manner. Our kit will allow users to have a low-level autonomous system feature to help navigate through tight spaces such as a narrow hallway or a narrow entrance. Moreover, with our safety feature implementations, we make sure our users stay safe and have full control of their electric chair while using our system.
  
## Implementation
  
  We create our autonomous system by using an STM32 Nucleo-F767ZI board for real time operation in conjunction with distance sensors, an
  intertial measurement unit, encoders, and emergency stop button. 
  
  <img src="https://github.com/SmartWheelchair/Systems/blob/master/Wheelchair%203D%20Part%20Images/STM32_Nucleo_F767ZI.PNG" width="500">

  We are currently using the VL53L1X time of flight distance sensors to aid with safety features. The sensors return the distance that an object is from the wheelchair and based on this information, the wheelchair slows down and halts. Moreover, it is used to help our users navigate in a straight line if they are in a narrow hallway, to avoid running over a ledge, and to stop the chair from colliding with a wall or object in front of it. 

<p align="center">
  <img src="https://github.com/SmartWheelchair/Systems/blob/master/Wheelchair%203D%20Part%20Images/ToF_Sensors_VL53L1X_Image.PNG" width="500">
  <img src="https://github.com/SmartWheelchair/Systems/blob/master/Wheelchair%203D%20Part%20Images/ToF_Sensors_Working.PNG" width="1000">
</p>

## Contact
**Website:** http://smartwheelchair.eng.ucsd.edu/  
**Phone:** (619) 836-8052  
**Email:** smartwheelchair.eng.ucsd.edu  
