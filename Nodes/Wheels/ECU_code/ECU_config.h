#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

//debug mode: to print info to terminal
#define DEBUG

//timing config
#define THROTTLE_TIMEOUT 300    //timeout for not recieving throttle messages before an error is raised
#define WHEEL_SPEED_TIMEOUT 300    //timeout for not recieving wheel speed messages before an error is raised

#define VEHICLE_SPEED_MSG_INTERVAL 100 //interval in ms for the vehicle speed message
#define WHEEL_THROTTLES_MSG_INTERVAL 10 //interval in ms for the individual wheel throttles message


//otherNodeStatus masks
#define BMS_STATUS_MASK 0b01000000
#define TPS_STATUS_MASK 0b00100000
#define SAS_STATUS_MASK 0b00010000
#define IMU_STATUS_MASK 0b00001000
#define FW_STATUS_MASK  0b00000100
#define RLW_STATUS_MASK 0b00000010
#define RRW_STATUS_MASK 0b00000001

//wheel spin masks
#define flWheelSpin 0b0001
#define frWheelSpin 0b0010
#define rlWheelSpin 0b0100
#define rrWheelSpin 0b1000

#define WHEEL_CIRCUMFERENCE 160 //TODO (CALCULATE CORRECTLY): circumference of wheel in cm. THIS NUMBER IS A PLACEHOLDER AND IS NOT ACCURATE. PLEASE GET A MORE ACCURATE NUMBER SOON

//Wiring Connections
//TODO: decide on pin and plan circuit
#define E_STOP_SENSE_PIN 3 //fill in
//#define ENABLE_E_STOP_RELAY   //if the ECU has a relay to turn off the estop
    #ifdef ENABLE_E_STOP_RELAY
    #define E_STOP_RELAY_PIN 2 //not sure if we wanna use
    #endif

#endif