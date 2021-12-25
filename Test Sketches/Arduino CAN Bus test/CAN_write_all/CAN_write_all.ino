/*
 *  CAN Bus Spoofer
 *  For EVAM
 *  Sends messages as though various nodes are online on the CAN Bus
 */
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <can_ids.h>
#define POT_PIN A0


unsigned long lastThrottleMillis = 0;
unsigned long lastSteeringMillis = 0;
bool motorsLocked = false;


void sendStatuses(){
  mcp2515.sendMessage(&ecu);
  delay(30);
  mcp2515.sendMessage(&bms);
  delay(30);
  mcp2515.sendMessage(&tps);
  delay(30);
}

void sendThrottleAndBrake(){
  mcp2515.sendMessage(&throttleBrakeMsg);
}
void sendSteering(){
  mcp2515.sendMessage(&steeringMsg);
}
void sendRevBoostMsg(){
  mcp2515.sendMessage(&revBoostMsg);
}

void setUpCanMsgs(){
  ecu.can_id = ECU_STATUS_MSG_ID;
  ecu.can_dlc = 1;
  ecu.data[0] = 1;

  bms.can_id = BMS_STATUS_MSG_ID;
  bms.can_dlc = 1;
  bms.data[0] = 1;

  tps.can_id = TPS_STATUS_MSG_ID;
  tps.can_dlc = 1;
  tps.data[0] = 1;

  throttleBrakeMsg.can_id = THROTTLE_BRAKE_MSG_ID;
  throttleBrakeMsg.can_dlc = 5;
  for(uint8_t i = 0; i<5;i++){
    throttleBrakeMsg.data[i] = 0;
  }

  revBoostMsg.can_id = REV_BOOST_MSG_ID;
  revBoostMsg.can_dlc = 2;
  for(uint8_t i = 0; i<2;i++){
    revBoostMsg.data[i] = 0;
  }
  
  steeringMsg.can_id = STEERING_MSG_ID;
  steeringMsg.can_dlc = 2;
  steeringMsg.data[0] = 0;
  steeringMsg.data[1] = 8;
  
}


struct can_frame ecu;
struct can_frame bms;
struct can_frame tps;
/*
struct can_frame sas;
struct can_frame imu;
struct can_frame fw;
struct can_frame rlw;
struct can_frame rrw;
struct can_frame fl;
struct can_frame rl;
struct can_frame int;
*/

struct can_frame throttleBrakeMsg;
struct can_frame revBoostMsg;
struct can_frame battStatsMsg;
struct can_frame steeringMsg;
struct can_frame speedMsg;
struct can_frame rx_Msg;


//struct can_frame canMsg2;
MCP2515 mcp2515(10);
uint8_t potVal = 0;
int rawPot = 0;
void setup() {
  setUpCanMsgs();
  
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz 
  mcp2515.setNormalMode();
 
  Serial.println("CAN: Spoofer for EVAM");
  sendStatuses();
  delay(10);
  sendRevBoostMsg();
}

void loop() {
  unsigned long currentMillis - millis();
  if(currentMillis - lastThrottleMillis > 10){
    sendThrottleAndBrake();
    lastThrottleMillis = currentMillis;
  }

    if(currentMillis - lastSteeringMillis > 10){
    sendSteering();
    lastSteeringMillis = currentMillis;
  }
  
  //read from CANBUS
   if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK){
     

     if(canMsg.can_id==FRONT_LIGHT_MSG_ID){
       Serial.print("Front Lights set to: ")
        Serial.print(string(canMsg.data[0])+" "+string(canMsg.data[1])+" "+string(canMsg.data[2]))
     } else if(canMsg.can_id==REAR_LIGHT_MSG_ID){
       Serial.print("Rear Lights set to: ")
        Serial.print(string(canMsg.data[0])+" "+string(canMsg.data[1])+" "+string(canMsg.data[2]))
     } else if(canMsg.can_id==INT_LIGHT_MSG_ID){
       Serial.print("Interior Lights set to: ")
        Serial.print(string(canMsg.data[0])+" "+string(canMsg.data[1])+" "+string(canMsg.data[2]))
     } else if(canMsg.can_id==MOTOR_LOCKOUT_MSG_ID){
        if(motorsLocked != canMsg.data[0]){
          motorsLocked = canMsg.data[0];
          Serial.print("Motors Locked: ");
          Serial.println(motorsLocked);
        }
     } else{
      Serial.print("ID: ");
      Serial.print(canMsg.can_id); 
      Serial.print(" | Value: ");
      Serial.println(canMsg.data[0]);  
     }
     
}
