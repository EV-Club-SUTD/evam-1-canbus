#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

//debug mode: to print info to terminal
#define DEBUG

//timings
#define THROTTLE_TIMEOUT 300    //timeout for not recieving throttle messages before an error is raised
#define WHEEL_SPEED_TIMEOUT 300    //timeout for not recieving wheel speed messages before an error is raised

#define VEHICLE_SPEED_MSG_INTERVAL 100 //interval in ms for the vehicle speed message
#define WHEEL_THROTTLES_MSG_INTERVAL 10 //interval in ms for the individual wheel throttles message
#define ERROR_MSG_INTERVAL 50 //interval in ms between error messages (to avoid clogging up canbus)
#define ESTOP_MSG_INTERVAL 1000 //interval in ms between e-stop messages

//ERRORS

//status of the node
enum nodeErrorType {
    GENERIC_ERROR           = 0,
    OK                      = 1,
    ESTOP_PRESSED           = 2,
    THROTTLE_TIMED_OUT      = 3,
    STEERING_TIMED_OUT      = 4,
    WHEEL_SPEED_TIMED_OUT   = 5,
    OFFLINE                 = 255
};



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

#define WHEEL_CIRCUMFERENCE 160 //TODO (CALCULATE CORRECTLY): circumference of wheel in cm. THIS NUMBER IS A PLACEHOLDER AND IS NOT ACCURATE. PLEASE GET A MORE ACCURATE NUMBER SOON ONCE THE CIRCUMFERENCE IS MEASURED

//Wiring Connections
//TODO: decide on pin and plan circuit
#define E_STOP_SENSE_PIN 3
//#define ENABLE_E_STOP_RELAY   //if the ECU has a relay to turn off the estop
    #ifdef ENABLE_E_STOP_RELAY
    #define E_STOP_RELAY_PIN 4
    #endif

#endif