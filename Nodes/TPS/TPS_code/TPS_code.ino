/*
TPS CODE FOR EVAM


FUNCTIONS:
-Read throttle positions sensors and publish to CAN Bus
--Possibly average a few readings for stability
-If 2 TPS are connected, check between both sensors to identify throttle faults

-Read brake pressure sensor(s) and publish [pressure inforamtion to CAN Bus
-Convert pressure information to equivalent brake pedal position and publish to CANB us


Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)
Connect A0 to the throttle 
Connect A1 to the brake

!This code is not millis() overflow protected!
*/

//TODO:  Add in dual TPS and BPS code

#include <Ewma.h>
#include <SPI.h>
#include <mcp2515.h>  //arduino-mcp2515 by autowp: https://github.com/autowp/arduino-mcp2515/
#define DEBUG
//#define DUAL_TPS_SENSOR //for use with dual TPS (for redundancy)
  #ifdef DUAL_TPS_SENSOR
  //#define TPS2_REVERSE_VOLTAGE  //TPS 2 voltage decreases as the pedal is pressed more
  #endif
#endif

//timing stuff
#define MSG_INTERVAL 10  //timing delay in ms between messages sent by node
unsigned long lastMessageTime = 0;  //keeps track of the timestamp of the last message sent

//node status
uint8_t errorState = 255;  //state of the node. 0: error, 1: ok, 255: offline

//brake
#define BRAKE_PIN A1  //analog pin that the brake pressure sensor is connected to
uint16_t brakeRaw = 0;  //0-1023, raw value from Arduino ADC
uint8_t brakePressureByte = 0; //0-255, for CAN message. Brake Pressure = BrakePressureByte * 4. This is just a test value and needs to be calibrated
uint8_t brakePercentByte = 0; //0-255, for CAN message. Brake % = BrakePercentByte * 0.4

//throttle
#define ACC_PIN A0  //analog pin that the accelerator is connected to
#ifdef DUAL_TPS_SENSOR
#define ACC_PIN2 A1
#endif
uint16_t throttleRaw = 0;  //0-1023, raw value from Arduino ADC
uint16_t throttleRaw2 = 0;
uint8_t throttlePercentByte = 0;  //0-255, for CAN message. Throttle % = ThrottleByte * 0.4

//filtering
Ewma throttleFilter(0.1);
Ewma brakeFilter(0.1);


/***CAN BUS STUFF***/
MCP2515 mcp2515(10);

//status message
struct can_frame canStatusMsg;  //status of the node
canStatusMsg.can_id  = 0x0A;
canStatusMsg.can_dlc = 1;
canStatusMsg.data[0] = errorState;

//main message
struct can_frame canAccMsg; //main accelerator/brake message
canAccMsg.can_id  = 0x020;
canAccMsg.can_dlc = 6;
canAccMsg.data[0] = 0x00;
canAccMsg.data[1] = 0x00;
canAccMsg.data[2] = 0x00;
canAccMsg.data[3] = 0x00;
canAccMsg.data[4] = 0x00;
canAccMsg.data[5] = 0x00;


void sendCanMessage(){
  throttlePercentByte = map(throttleRaw, 0, 1023, 0, 255);  //maps the 10bit ADC value to 1 byte to be sent on the CANBus
  uint16_t brakePressure16bit = map(brakeRaw, 0, 1023, 0, 255); //maps the 10bit ADC value to 1 byte to be sent on the CANBus. this should be changed once we can test the range of the sensor
  brakePressureByte = constrain(brakePressure16bit, 0 , 255);   //constains in case there is an overflow
  uint16_t brakePercent16bit = map(brakeRaw, 0, 1023, 0, 255);  //needs to be calibrated
  brakePercentByte = constrain(brakePercent16bit, 0 , 255);     //constains in case there is an overflow
  canAccMsg.data[0] = throttlePercentByte;  //accelerator percentage
  canAccMsg.data[2] = brakePressureByte;    //brake pressure
  canAccMsg.data[4] = brakePercentByte;     //brake percentage
  mcp2515.sendMessage(&canAccMsg);
  
  #ifdef DEBUG  //print brake and throttle values
  float throttlePercent = throttlePercentByte * 0.4;
  float brakePressure = brakePressureByte * 4.0;
  float brakePercent = brakePercentByte * 0.4;
  Serial.print("Throttle = ");
  Serial.print(throttlePercent);
  Serial.print("| Brake Pressure = ");
  Serial.print(brakePressure);
  Serial.print("| Brake Percent = ");
  Serial.println(brakePercent);
  #endif
}

void readThrottle(){
  throttleRaw = analogRead(ACC_PIN);  //reads 10bit ADC value
  #ifdef DUAL_TPS_SENSOR
  throttleRaw2 = analogRead(ACC_PIN2);
  #endif
  //TODO: implement dual TPS checking code
}

void readBrake(){
  //brakeRaw = analogRead(BRAKE_PIN);  //reads 10bit ADC value
  brakeRaw = 0;  //from sensor
}

void average(){
  float throttleAvg = throttleFilter.filter(throttleRaw);
  float brakeAvg = brakeFilter.filter(brakeRaw);
  throttleRaw = (int)throttleAvg;
  brakeRaw = (int)brakeAvg;
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
void setup() {
  #ifdef DEBUG  //debug mode
  //while (!Serial);
  Serial.begin(115200);
  Serial.println("TPS");
  #ifndef ARDUINO_AVR_NANO
  Serial.print("WARNING: This sketch was designed for an arduino Nano");
  #endif //#ifndef ARDUINO_AVR_NANO
  #endif //#ifdef DEBUG
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  sendStatus(1);  
}

void loop() {
  readThrottle();
  readBrake();
  average();
  if(millis() - lastMessageTime >= MSG_INTERVAL){
    sendCanMessage();
  }
}
