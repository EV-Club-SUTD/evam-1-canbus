# EVAM'S CAN BUS
Contains the arduino code for the different nodes in the canbus


### Folders
`Notes\` contains some of my notes and also `CAN Bus Messages.xlsx` which describes the structure of the CAN Bus (namely the nodes and messages).


`Nodes\` contains the arduino sketches for the nodes:
 - BMS (Battery Management System) (*tested, working*)
 - ECU (Engine Control Unit) (*done, but not fully tested*)
-- not tested with all 4 motors connected
 - Wheel Nodes (*roughly done*)
-- Dual motor controller has been tested with 2 motor controllers, but single hasn't been developed. Should just be a case of copying the code over, changing the pins assigned, and disabling the second motor controller functionality.
 - SAS (Steering Angle Sensor) (*done*)
 - TPS (Throttle (and Brake) Position Sensor) (*done, but brakes untested*)
 - Front and Rear Lights (*rear light done, front to be done*)
 -  ~~IMU (Inertial Measurement Unit) (*will probably not be implemented*)~~
 
 
 > Refer to `Notes\CAN Bus Messages.xlsx` for more information about each node.
 
 **Not all the nodes have been developed yet though**
 
 The code for the dashboard node can be found at [the evam-dashboard repo](https://github.com/thespacemanatee/evam-dashboard)