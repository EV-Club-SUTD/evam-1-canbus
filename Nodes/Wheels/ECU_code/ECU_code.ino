/*
ECU (Engine Control Unit) CODE FOR EVAM

FUNCTIONS:
- Read individual wheel speeds from wheel nodes and calcluate vehicle speed; publish to CAN Bus
- Read accelerator, steering (and brake) values (and other settings, like reverse, boost, etc) and calculate individual throttle values for each wheel; publish to CAN Bus
- Check if E stop button is pressed (100V rail); publish to CAN Bus
- Check if the battery current is too high and lower the throttle amount if it's overloaded for too long

Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)

Code is still under development

*/
#include <SPI.h>
#include <mcp2515.h>  //arduino-mcp2515 by autowp: https://github.com/autowp/arduino-mcp2515/
#define DEBUG

//timing
#define THROTTLE_TIMEOUT 300    //timeout for not recieving throttle messages before an error is raised
#define WHEEL_SPEED_TIMEOUT 300    //timeout for not recieving wheel speed messages before an error is raised

#define VEHICLE_SPEED_MSG_INTERVAL 100 //interval in ms for the vehicle speed message
#define WHEEL_THROTTLES_MSG_INTERVAL 10 //interval in ms for the individual wheel throttles message

unsigned long lastRcvThrottleMillis = 0;    //time last throttle message was received
unsigned long lastRcvSteeringMillis = 0;    //time last throttle message was received
unsigned long lastRcvWheelSpeedMillis = 0;    //time last throttle message was received
unsigned long lastSendWheelThrottlesMillis = 0;   //time last wheel speed message was sent
unsigned long lastSendVehicleSpeedMillis = 0;   //time last wheel speed message was sent

//Errors
uint8_t errorState = 255;   //status of the node
uint8_t overCurrent = 0;
uint8_t eStop = 0; //e-Stop Pressed



/***CAN BUS DATA***/
MCP2515 mcp2515(10);
struct can_frame canMsg; //generic CAN message for recieving data

//status message
struct can_frame canStatusMsg;  //status of the node
canStatusMsg.can_id  = 0x08;
canStatusMsg.can_dlc = 1;
canStatusMsg.data[0] = errorState;

//e-Stop Message
struct can_frame eStopMsg; //e-Stop message
eStopMsg.can_id  = 0x04;
eStopMsg.can_dlc = 1;
eStopMsg.data[0] = 0x00;

//Individual wheel throttles Message
struct can_frame indivWheelThrottlesMsg; // individual wheel throttles
indivWheelThrottlesMsg.can_id  = 0x30;
indivWheelThrottlesMsg.can_dlc = 4;
indivWheelThrottlesMsg.data[0] = 0x00;
indivWheelThrottlesMsg.data[1] = 0x00;
indivWheelThrottlesMsg.data[2] = 0x00;
indivWheelThrottlesMsg.data[3] = 0x00;

//Vehicle Speed Message
struct can_frame vehicleSpeedMsg; // speed message
vehicleSpeedMsg.can_id  = 0x37;
vehicleSpeedMsg.can_dlc = 3;
vehicleSpeedMsg.data[0] = 0x00;
vehicleSpeedMsg.data[1] = 0x00;
vehicleSpeedMsg.data[2] = 0x00;


/****WHEEL & SPEED DATA****/
//common
#define WHEEL_CIRCUMFERENCE 160 //TODO (CALCULATE CORRECTLY): circumference of wheel in cm. THIS NUMBER IS A PLACEHOLDER AND IS NOT ACCURATE. PLEASE GET A MORE ACCURATE NUMBER SOON
uint8_t throttle = 0;
uint8_t brakePos = 0;
int16_t steeringAngle = 0; //divide by 10 to get actual steering angle
bool throttleRev = 0;   //reverse. 0: forward, 1: reverse
uint8_t boostMode = 0;  //0: normal, 1: eco, 2: boost
uint16_t vehicleSpeed = 0;  //vehicle speed. Divide by 256 to get actual speed

/*  Individual wheel settings. Format:
 * [0] = front left
 * [1] = front right
 * [2] = rear left
 * [3] = rear right
 */
uint8_t wheelDirs[4];       //direction of rotation of the wheel. Forward = 0, reverse = 1
uint16_t wheelSpeeds[4];    //wheel speeds. Multiply by 0.03 to get value in RPM
uint8_t wheelThrottles[4];  // wheel throttle values to send to nodes

/***OTHER FUNCTIONS***/
//current control
uint16_t battCurrent = 0;   //Battery Current(A) = -320 + battCurrent*0.1	
uint8_t battTemp = 0; //subtract 40 from this for the actual temperature

///abs and traction control
uint8_t wheelSpin = 0b0000;  //Set 1 for the respective bit if a wheel is spinning. order of bits: fl fr rl rr


//Wiring Connections
//TODO: decide on pin and plan circuit
#define E_STOP_SENSE_PIN 1 //fill in
//#define ENABLE_E_STOP_RELAY   //if the ECU has a relay to turn off the estop
    #ifdef ENABLE_E_STOP_RELAY
    #define E_STOP_RELAY_PIN 2 //not sure if we wanna use
    #endif


void absTractionControl(){ //calculates if any of the wheel speeds are a lot more than the rest and lowers the throttle for it significantly/turns off throttle for it
    //TODO
    wheelSpin = 0b0000;
    uint16_t avgWheelRpm = calcAvgRpm();
    //idk what to do 
    
} 

void calculateWheelThrottles(){
    
}

void checkOverCurrent()[    //checks if battery current is too high for prolonged time
    //TODO
]

void calculateSendCarSpeedMsg(){
    //check if wheel directions are all the same. Is there a better way to do this??
    bool wheelsSameDir = true;
    uint16_t avgWheelRpm  =0;
    if (wheelsSameDir){
        avgWheelRpm = calcAvgRpm();
    }
    else{
        avgWheelRpm = wheelSpeeds[0]
    }
    float vehicleSpeedFloat = (float)avgWheelRpm * 0.03 * 60 * WHEEL_CIRCUMFERENCE / (1000 * 100);
    vehicleSpeed = int(vehicleSpeedFloat * 256);

    vehicleSpeedMsg.data[0] = vehicleSpeed & 0xFF;
    vehicleSpeedMsg.data[1] = vehicleSpeed >> 8;
    //TODO: reverse or forward
}

uint16_t calcAvgRpm(uint8_t excludeWheel = 5){
    uint32_t rpmSum = 0;
    for (int i = 0; i < 4; i++){
        if (i == excludeWheel) {    //
          continue;
        rpmSum += wheelSpeeds[i]
    }
    uint16_t avgWheelRpm = (uint16_t)(rpmSum / 4);
    return avgWheelRpm
}

//to update CANBus on the status of the node
void sendStatus(uint8_t status = 0){
    errorState = status;
    #ifdef DEBUG
    Serial.print("Node status: ");
    Serial.println(errorState);
    #endif //DEBUG
    canStatusMsg.data[0] = status;
    mcp2515.sendMessage(&canStatusMsg);
}

//to update CANBus on the status of the node
void sendEStopMsg(uint8_t eStop = 1){
    errorState = status;
    #ifdef DEBUG
    Serial.print("Node status: ");
    Serial.println(errorState);
    #endif //DEBUG
    canStatusMsg.data[0] = status;
    mcp2515.sendMessage(&canStatusMsg);
}

void sendIndivThrottlesMsg(){
    for(int i = 0; i <4; i++){
        indivWheelThrottlesMsg.data[i] = wheelThrottles[i];  
    }
    mcp2515.sendMessage(&indivWheelThrottlesMsg);

    #ifdef DEBUG  //print brake and throttle values
    Serial.print("Wheel Throttles: ");
    Serial.print(flWheelSpeed*0.03);
    Serial.print(" | Front Right Wheel Speed = ");
    Serial.print(frWheelSpeed*0.03);
    #endif
}


void setup() {
    //set up e stop pin
    pinMode(E_STOP_SENSE_PIN, INPUT);
    #ifdef ENABLE_E_STOP_RELAY
    pinMode(E_STOP_RELAY_PIN, OUTPUT);
    #endif

    #ifdef DEBUG  //debug mode
    Serial.begin(115200);
    Serial.println("ECU");
    #ifndef ARDUINO_AVR_NANO
    Serial.print("WARNING: This sketch was designed for an arduino Nano");
    #endif //#ifndef ARDUINO_AVR_NANO
    #endif //#ifdef DEBUG
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    sendStatus(1);  //TODO: only come online once all the other nodes are online
}

void loop() {   //TODO: NOT DONE AT ALL
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        if(canMsg.can_id == 0x20){ //Throttle n Brake Position
            throttle = canMsg.data[0];
            brakePos = canMsg.data[4];
            lastRcvThrottleMillis = millis();
            #ifdef DEBUG
            //Serial.println("Throttle: " + String(throttle));
            #endif  //DEBUG
        }
        else if(canMsg.can_id == 0x24){ //Battery Stats
            battCurrent = canMsg.data[2] + (canMsg.data[3]<<8);
            battTemp = canMsg.data[2];
            // #ifdef DEBUG
            // //Serial.println("Throttle: " + String(throttle));
            // #endif  //DEBUG
        }
        else if(canMsg.can_id == 0x2D){ //Battery Stats
            steeringAngle = canMsg.data[0] + (canMsg.data[1]<<8) - 1800;
            lastRcvSteeringMillis = millis();
            // #ifdef DEBUG
            // //Serial.println("Throttle: " + String(throttle));
            // #endif  //DEBUG
        }
        //TODO : read wheel speeds
        //TODO: read differential balances
    } //if message available


    if(millis() - lastSendMillis >= MSG_INTERVAL){
        sendCanMessage();
        lastSendMillis = millis();
    }

    //timeouts
    if (millis() - lastRcvThrottleMillis > THROTTLE_TIMEOUT){
    sendStatus(0);  //raise error
    }
    if (millis() - lastRcvSteeringMillis > THROTTLE_TIMEOUT){
    sendStatus(0);  //raise error
    }
    if (millis() - lastRcvWheelSpeedMillis > WHEEL_SPEED_TIMEOUT){
    sendStatus(0);  //raise error
    }
}