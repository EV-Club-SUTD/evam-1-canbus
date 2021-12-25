#ifndef _FW_CONFIG_H_
#define _FW_CONFIG_H_

#define DEBUG   //debug mode prints imformation to terminal

//Connections to ESC
//Updated based on PCB
//Numbers are based on underlying integer pin numbers
#define LEFT_THROTTLE_PIN   5 
#define RIGHT_THROTTLE_PIN  6 
#define LEFT_REVERSE_PIN    7 
#define RIGHT_REVERSE_PIN   8 
#define LEFT_BOOST_PIN      17      //A3
#define RIGHT_BOOST_PIN     16      //A2
#define LEFT_ECO_PIN        19      //A5
#define RIGHT_ECO_PIN       18      //A4
#define LEFT_REGEN_PIN      15      //A1, was previously motor lock pin
#define RIGHT_REGEN_PIN     14      //A0, was previously motor lock pin
//for motor pulse detection
#define LEFT_PULSE_PIN      2
#define RIGHT_PULSE_PIN     3

//timings
#define THROTTLE_TIMEOUT    150 //timeout for not recieving throttle messages before an error is raised
#define MSG_INTERVAL        10  //interval in ms for the wheel speed messages
#define ERROR_MSG_INTERVAL  100 //interval in ms for error messages

#define PRINT_INTERVAL 50      //interval in ms to print speed and throttle message to serial port
unsigned long lastPrintMillis = 0; 


//status of the node
enum nodeErrorType {
    GENERIC_ERROR           = 0,
    OK                      = 1,
    //ESTOP_PRESSED           = 2,
    //THROTTLE_TIMED_OUT      = 3,
    //STEERING_TIMED_OUT      = 4,
    //WHEEL_SPEED_TIMED_OUT   = 5,
    OFFLINE                 = 255
};

#endif //_FW_CONFIG_H_