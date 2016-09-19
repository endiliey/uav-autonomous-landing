# DIP UAV sub-project - GPS-guided Autonomous Landing

##About this project :
Collaborative mission between UAV (Unmanned Aerial Vehicle) and USV (Unmanned Surface Vehicle) to rescue survivors

##Project Objective :
To design either on-board or off-board system that can help the miniature UAV to land autonomously in a specific location with high accuracy, minimum error tolerance and fast execution

## Programming Language : C with Arduino IDE

## Initial Proposed Solution Decription
When miniature UAV is low on battery, it will enter *Autonomous Mode* -> with GPS-guidance, UAV will find shortest pathway to hover around a specific landing spot at USV -> UAV will be guided by USV on-board controller to land

- Using three ultrasonics sensor placed in each edge of an equilateral triangle (Triangulation Method)
- When UAV is within 4 meters height of the landing spot, USV board will communicate with UAV through a bluetooth module
- UAV will autoland by itself with USV Guidance

## Initial Proposed Solution Problem
- Slower algorithm
- Lot of offset problems in the future
- There is a need to have 3 sonar sensors,

#How can collaborators help?
Propose an autonomous landing algorithm for drones
