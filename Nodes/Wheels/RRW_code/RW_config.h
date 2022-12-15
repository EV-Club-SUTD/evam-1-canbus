#ifndef _RW_CONFIG_H_
#define _RW_CONFIG_H_

#define DEBUG   //debug mode prints imformation to terminal

//Connections to ESC
//Updated based on PCB
//Numbers are based on underlying integer pin numbers
#define THROTTLE_PIN   6 //WAS ORIGINALLY 7 (swapped with reverse), BUT 7 DOESN'T HAVE PWM!! 
#define REVERSE_PIN    7 //WAS ORIGINALLY 6 (swapped with throttle)
#define BOOST_PIN      4      
#define ECO_PIN        5      
#define REGEN_PIN      8      //IS THIS REGEN OR LOCK??
//for motor pulse detection
#define PULSE_PIN      2

//timings
#define THROTTLE_TIMEOUT    150 //timeout for not recieving throttle messages before an error is raised
#define MSG_INTERVAL        10  //interval in ms for the wheel speed messages
#define ERROR_MSG_INTERVAL  100 //interval in ms for error messages

#define PRINT_INTERVAL 100      //interval in ms to print speed and throttle message to serial port
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

#endif //_RW_CONFIG_H_