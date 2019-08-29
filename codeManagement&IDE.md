![alt text](https://github.com/SmartWheelchair/Systems/blob/master/Wheelchair%203D%20Part%20Images/UCSD_Wheelchair_Team_Logo.png "Logo")

This document will walk you through how we share our code and how we use the development environment and debugger.

Code management:
Embedded systems is keeping the code on the “wheelchairControl” repository under the SmartWheelchair github.
https://github.com/SmartWheelchair/wheelchairControl

This is a public repository so anyone has access to download libraries from it. To get access as a contributor contact an Affordable Smart Wheelchair manager:  
Jesus Fausto: jvfausto@ucsd.edu  
Isabella Gomez: ilgomezt@ucsd.edu  
Jesi Miranda: j7mirand@ucsd.edu  
Richart To: rlto@ucsd.edu  
  
There is code that does not work on the offline compiler; it only works on the mbed online compiler. The Ros-mbed library is not working in the offline compiler. For now if you want a copy the ros-mbed wheelchair code please contact Jesi Miranda who has access to a working program so he could forward it to you. Later on we will include it as a branch in the github account.

Most of the libraries you might add to the program will come from the mbed repository system. When you download and add a library to the program, it will appear with a “.lib” reference. Once you download that library delete the “.lib” reference of program before uploading it to github so that any changes you make get saved on the team repository rather than on your personal one. Add a link to the original repository on top of the .h and the .cpp file to give credit to the writer of the library.

Development Environment:

You can use any compiler that works to compile, work on, and debug your code. We recommend you use an offline compiler to make github repository sharing easier. 
We have created tutorials to install and use the Mbed Command Line(CLI) as well as several useful links. We emphasise, at first it will be hard and take some time, but stick to using the offline compiler and over time it will be easier than the online compiler(1).
I recommend you to practice in the CLI environment for a while so you have practice with file movement and transferring, VIM, and Git command line.
After you are comfortable with working on command line, install the eclipse IDE and follow the debugging setup instructions(2). We recommend you to try out eclipse not only as an IDE, but also as a debugger.
If you run into issues on any installation for mbed CLI or the eclipse debugger contact Jesus Fausto.


References
1. Github, Vim, and mbedCLI tutorials
https://docs.google.com/presentation/d/1ZzLTbSPLkaWmC_LvMOWpK6OrA_3AaveXz78WuPfxXF4/edit#slide=id.p

2. Eclipse debugger setup tutorial
https://docs.google.com/presentation/d/1-Ee_TDCPF9NF6DgjkxxXaFKLO6FSHQmupHrXi9HXvZI/edit#slide=id.p
