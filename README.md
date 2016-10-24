# DIP UAV 

##About this project:
This project is mainly about coding our Quadcopter to fulfill the collaborative mission between UAV (Unmanned Aerial Vehicle) and USV (Unmanned Surface Vehicle)

##Main Hardware Used (exclude the cables,motors,etc) :
- Naze32 ~ flight controller with Cleanflight firmware
- Turnigy 6x ~ radio controller
- Turnigy XR7000 ~ receiver
- Arduino UNO ~ as the brain of our quadcopter
- Pixy Camera ~ for our On-board object detector

## Programming Language : C with Arduino IDE
## What are we doing ? 

We are using Arduino as an intermediary between the radio controller and our quadcopter. 
Our quadcopter have two modes :
###1. Autoland mode
When switch channel 5 is turned on, our copter will detect the landing platform by itself and hover to it as well as landing. Any external input from us (from radio controller) is ignored, except the switch channel 5 mode.
###2. Manual mode
When switch channel 5 is turned off, our copter will be manually controlled by our radio controller.

## When will your project be finished ?
Since this is a school project, we will have to complete this by 9th November 2016. We will put a video on youtube when we've finished testing and put it here ! And a more proper documentation from scratch on how to build a low-cost drones will be done after my final exam (1 December 2016).

So Stay Tune !!

## FAQ
1. Why do you use Pixy detection instead of using openCV + Python ?
At first, we wanted to use openCV with python as it is generally easier. However, openCV is a very big library which need a high-level computation (at least Raspberry PI), and Arduino won't be able to run the openCV without the help of any computer computation. Other reason includes, other team members are not familiar with Raspberry Pi and are more comfortable with Arduino
2. How can we help you for this project?
Simply help us to make the code more efficient (but not decreasing so much readability)

## Current Problem
###We are thinking of an efficient landing algorithm
Problem can be simplified into; 
Calling all drones hobbyist/hackers/makers/etc

###Given 
####Input : x,y, height

####Input info :
#####1. x,y is the cartesian coordinates in x-y plane with x = 0, y =0 in the middle
#####2.assume height = 200 means object is within 10 cm of Target, while height = 50 means object is within 160 cm from object)

###Output : value of Throttle, Roll, Yaw and Pitch 
(such that the Drone will land in the center of Target (assume |x,y| = |10,10| )

####Output info :
#####1. Basic drones control
http://www.robolink.com/basics-of-codrone-control/
#####2. Throttle, Roll, Yaw and Pitch value is a PWM (Pulse width modulation) value ranging between 900 to 2000 (with 900 = low, 1500 = default, 2000 = HIGH)
Example : A roll PWM Value of 2000 will means that the drone will tilt to the right aggresively
