#include <SPI.h> //Library for using SPI Communication 
#include <mcp2515.h> //Library for using CAN Communication

struct can_frame canMsg; 
MCP2515 mcp2515(10); // SPI CS Pin 10 

void setup() {
  SPI.begin();   //Begins SPI communication
  Serial.begin(115200); //Begins Serial Communication at 9600 baud rate 
  mcp2515.reset();                          
  mcp2515.setBitrate(CAN_500KBPS); //Sets CAN at speed 500KBPS and Clock 8MHz 
  mcp2515.setNormalMode();  //Sets CAN at normal mode
}

void loop(){
 if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK){
     int x = canMsg.data[0];
     Serial.print("ID: ");
     Serial.print(canMsg.can_id); 
     Serial.print(" | Value: ");
     Serial.println(x);     
   }
}
