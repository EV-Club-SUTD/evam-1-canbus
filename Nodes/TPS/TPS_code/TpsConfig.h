#ifndef TPS_CONFIG_H
#define TPS_CONFIG_H

//#define FILTER_VALUES //seems like there is no need to filter the values. they are very stable
#ifdef FILTER_VALUES
//#define FILTER_THROTTLE
//#define FILTER_BRAKE
#include <EwmaT.h>
#endif //FILTER_VLAUES

#include <SPI.h>
#include <mcp2515.h>  //arduino-mcp2515 by autowp: https://github.com/autowp/arduino-mcp2515/
#include "can_ids.h"

#define DEBUG

//throttle & brake 
//#define DUAL_TPS //for use with dual TPS (for redundancy), disabled for now
#ifdef DUAL_TPS
  #define TPS2_REVERSE_VOLTAGE  //TPS 2 voltage decreases as the pedal is pressed more
#endif

//#define TWO_BRAKE_SENSORS

#define ACC_PIN A0  //analog pin that the accelerator is connected to
#ifdef DUAL_TPS
  #define ACC_PIN2 A1
#endif  //DUAL_TPS
#define BRAKE_PIN A2  //analog pin that the brake pressure sensor is connected to
#ifdef TWO_BRAKE_SENSORS  //for front and rear brake circuit pressure monitoring
  #define BRAKE_PIN2 A3
#endif  //TWO_BRAKE_SENSORS



//TPS Callibration
#define TPS1_MIN_VAL 0    //dual TPS: 128-129
#define TPS1_MAX_VAL 1006    //dual TPS: 927

#ifdef DUAL_TPS
#define TPS2_MIN_VAL 550    //is actually 550
#define TPS2_MAX_VAL 956    //is actually 956-958
#define THROTTLE_DIFFERENCE_LIMIT 600 //the maximum allowed mismatch between calibrated values of TPS 1 and 2
#endif

#ifdef FILTER_VALUES
//Exponential Weighted Moving Average Filter
EwmaT <uint32_t> throttleFilter(3, 100);
EwmaT <uint32_t> brakeFilter(3, 100);
#ifdef TWO_BRAKE_SENSORS
EwmaT <uint32_t> brakeFilter2(3, 100);
#endif  //TWO_BRAKE_SENSORS
#endif  //FILTER_VALUES

//timing stuff
#define MSG_INTERVAL 10  //timing delay in ms between messages sent by node
unsigned long lastMessageTime = 0;  //keeps track of the timestamp of the last message sent

#define PRINT_INTERVAL 300
unsigned long lastPrintMillis = 0;  //keeps track of the timestamp of the last accelerator message printed


//node status
uint8_t errorState = 255;  //state of the node. 0: error, 1: ok, 255: offline

/***CAN BUS STUFF***/
MCP2515 mcp2515(10);
struct can_frame canStatusMsg;  //status of the node
struct can_frame canAccMsg; //main accelerator/brake message
struct can_frame canMsg; //generic CAN message for recieving messages

//Functions

void readFilterThrottle();
void readFilterBrake();
uint16_t checkForErroneousValues(uint16_t _rawVal, uint16_t minVal, uint16_t maxVal);
uint8_t calcBrakePercent(uint8_t brakeRaw);
void sendStatus(uint8_t status = 0);
void setupCan();

#endif  //TPS_CONFOG_H
