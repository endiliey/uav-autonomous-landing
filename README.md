# DIP UAV 

##About this project :
Collaborative mission between UAV (Unmanned Aerial Vehicle) and USV (Unmanned Surface Vehicle)

## Programming Language : C with Arduino IDE

## Landing System (Autonomous Landing)
When miniature UAV is within 4 meters of the landing station and have entered *Autonomous Landing Mode* -> UAV will be guided by USV on-board controller to land

- Using four ultrasonics sensor (HC-SR04) placed in each edge of a square
- (within 4 meters height of the landing spot) USV board will communicate with UAV through a bluetooth module
- Each Ultrasonics sensors can calculate the distance of the drones from the sensor itself by generating high-frequency sound waves and evaluate the echo which is received back by the sensor, measuring the time interval between sending the signal and receiving the echo to determine the distance to an object
- distance = (duration/2) / 29.1 (Sound travels 340 m/s or around 29.1 microsecond per centimeter)
- USV will then send signal to UAV through bluetooth when specific (Sonar 1,Sonar 2,Sonar 3, Sonar 4) distance is already within 100cm
- UAV will calibrate its position after getting all Sonar distance input
- UAV will then autoland

## Pilot System (Human- Controlled)

-to be described later-


