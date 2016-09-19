# DIP UAV sub-project - GPS-guided Autonomous Landing

##About this project :
Collaborative mission between UAV (Unmanned Aerial Vehicle) and USV (Unmanned Surface Vehicle) to rescue survivors

##Project Objective :
To design either on-board or off-board system that can help the miniature UAV to land autonomously in a specific location with high accuracy, minimum error tolerance and fast execution

## Programming Language : C with Arduino IDE

## Initial Proposed Solution Decription
When miniature UAV is low on battery, it will enter *Autonomous Mode* -> with GPS-guidance, UAV will find shortest pathway to hover around a specific landing spot at USV -> UAV will be guided by USV on-board controller to land

- Using three ultrasonics sensor (HC-SR04) placed in each edge of an equilateral triangle (Triangulation Method)
- When UAV is within 4 meters height of the landing spot, USV board will communicate with UAV through a bluetooth module
- Each Ultrasonics sensors can calculate the distance of the drones from the sensor itself by generating high-frequency sound waves and evaluate the echo which is received back by the sensor, measuring the time interval between sending the signal and receiving the echo to determine the distance to an object
- distance = (duration/2) / 29.1 (Sound travels 340 m/s or around 29.1 microsecond per centimeter)
- USV will then send signal to UAV through bluetooth when specific (Sonar 1,Sonar 2,Sonar 3) distance is already within 100cm
- UAV will autoland after getting USV command


## Initial Proposed Solution Problem
- Slower algorithm (performance problem)
- Lot of offset problems in the future. Especially if the USV is moving (accuracyproblem)
- There is a need to have 3 sonar sensors (space problem)

#How can collaborators help?
Propose an autonomous landing algorithm for drones
