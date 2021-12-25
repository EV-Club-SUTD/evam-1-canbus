#include <SPI.h>
#include <mcp2515.h>
#define POT_PIN A0

struct can_frame canMsg1;
//struct can_frame canMsg2;
MCP2515 mcp2515(10);
uint8_t potVal = 0;
int rawPot = 0;
void setup() {
  canMsg1.can_id  = 0x20;
  canMsg1.can_dlc = 4;

  canMsg1.data[0] = 0;
  canMsg1.data[1] = 0;
  canMsg1.data[2] = 0;
  canMsg1.data[3] = 0;
  /* 
  canMsg1.data[4] = 0;
  canMsg1.data[5] = 0;
  canMsg1.data[6] = 0;
  canMsg1.data[7] = 0;
/*
  canMsg2.can_id  = 0x036;
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0x0E;
  canMsg2.data[1] = 0x00;
  canMsg2.data[2] = 0x00;
  canMsg2.data[3] = 0x08;
  canMsg2.data[4] = 0x01;
  canMsg2.data[5] = 0x00;
  canMsg2.data[6] = 0x00;
  canMsg2.data[7] = 0xA0;
  */
  //while (!Serial);
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz 
  mcp2515.setNormalMode();
 
  Serial.println("Example: Write to CAN");
}

void loop() {
  //readPot();
  //rawPot = analogRead(POT_PIN);
  //potVal = map(rawPot, 0, 1023, 0, 255);

  //hardcode 100 as first data byte
  canMsg1.data[0] = 100;
  mcp2515.sendMessage(&canMsg1);
//  mcp2515.sendMessage(&canMsg2);

  Serial.println("Messages sent");
  
  delay(100);
}
