# EVAM'S CAN BUS
Contains the arduino code for the different nodes in the canbus


### Folders
`Notes\` contains some of my notes and also `CAN Bus Messages.xlsx` which describes the structure of the CAN Bus (namely the nodes and messages).


`Nodes\` contains the arduino sketches for the nodes:
 - BMS (Battery Management System) (*tested, working*)
 - ECU (Engine Control Unit) (*haven't begun*)
 - Wheel Nodes (*in progress*)
 - SAS (Steering Angle Sensor) (*early stages*)
 - TPS (Throttle (and Brake) Position Sensor) (*in progress*)
 - Front and Rear Lights (*In progress*)
 - IMU (Inertial Measurement Unit) (*haven't begun, low priority*)
 
 
 > Refer to `Notes\CAN Bus Messages.xlsx` for more information about each node.
 
 **Not all the nodes have been developed yet though**
 
 The code for the dashboard node can be found at [the evam-dashboard repo](https://github.com/thespacemanatee/evam-dashboard)