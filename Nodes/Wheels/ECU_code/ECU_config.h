#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>  //arduino-mcp2515 by autowp: https://github.com/autowp/arduino-mcp2515/
#include <EEPROM.h>
#include "can_ids.h"

//debug mode: to print info to terminal
#define DEBUG

//timings
#define THROTTLE_TIMEOUT 300    //timeout for not recieving throttle messages before an error is raised
#define WHEEL_SPEED_TIMEOUT 300    //timeout for not recieving wheel speed messages before an error is raised

#define VEHICLE_SPEED_MSG_INTERVAL 100 //interval in ms for the vehicle speed message
#define WHEEL_THROTTLES_MSG_INTERVAL 10 //interval in ms for the individual wheel throttles message
#define ERROR_MSG_INTERVAL 100 //interval in ms between error messages (to avoid clogging up canbus)
#define ESTOP_MSG_INTERVAL 1000 //interval in ms between e-stop messages
#define PRINT_INTERVAL 100      //interval in ms between speed and accelerator messages printed to serial port


//ERRORS

//status of the node
enum nodeErrorType {
    GENERIC_ERROR           = 0,
    OK                      = 1,
    OFFLINE                 = 255
};

enum ecuErrorType {

    NO_ERROR                    = 0,
    THROTTLE_TIMED_OUT      = 3,
    STEERING_TIMED_OUT      = 4,
    WHEEL_SPEED_TIMED_OUT   = 5
};


//CAN Bus
MCP2515 mcp2515(10);
struct can_frame canMsg; //generic CAN message for recieving data
struct can_frame canStatusMsg;  //status of the node
struct can_frame eStopMsg; //e-Stop message
struct can_frame indivWheelThrottlesMsg; // individual wheel throttles
struct can_frame vehicleSpeedMsg; // speed message

//otherNodeStatus masks
#define BMS_STATUS_MASK 0b01000000
#define TPS_STATUS_MASK 0b00100000
#define SAS_STATUS_MASK 0b00010000
#define IMU_STATUS_MASK 0b00001000
#define FW_STATUS_MASK  0b00000100
#define RLW_STATUS_MASK 0b00000010
#define RRW_STATUS_MASK 0b00000001

/* ABS AND TRACTION CONTROL */
//wheel masks
#define flWheelMask 0b0001
#define frWheelMask 0b0010
#define rlWheelMask 0b0100
#define rrWheelMask 0b1000

//differential power balance settings
uint8_t lrPowerBalance;      //Left-Right Differential Power Balance. 0: equal distribution between inner and outer wheel, 255: more power to outer wheel
uint8_t lrPowerScale;        //working it out
uint8_t frPowerBalance;    //Front-Rear Differential Balance. 0: equal distribution between front and rear wheels, 255: most power to rear wheels (scaled based on acceleration)
uint8_t frPowerScale;        //working it out

//EEPROM Addresses for differential power balance settings
#define LR_POWER_BALANCE_ADDR 0
#define LR_POWER_SCALE_ADDR 1
#define FR_POWER_BALANCE_ADDR 2
#define FR_POWER_SCALE_ADDR 3

#define WHEEL_CIRCUMFERENCE 160 //TODO (CALCULATE CORRECTLY): circumference of wheel in cm. THIS NUMBER IS A PLACEHOLDER AND IS NOT ACCURATE. PLEASE GET A MORE ACCURATE NUMBER SOON ONCE THE CIRCUMFERENCE IS MEASURED

//Wiring Connections
//TODO: decide on pin and plan circuit
#define E_STOP_SENSE_PIN 3
//#define ENABLE_E_STOP_RELAY   //if the ECU has a relay to turn off the estop
    #ifdef ENABLE_E_STOP_RELAY
    #define E_STOP_RELAY_PIN 4
    #endif

#endif  //ECU_CONFIG_H
