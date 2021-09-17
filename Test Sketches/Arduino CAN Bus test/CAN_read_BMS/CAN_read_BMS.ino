#include <SPI.h>
#include <mcp2515.h>
float V, I;
int SOC, Th, Tl;

struct can_frame canMsg;
MCP2515 mcp2515(10);


void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS);
  mcp2515.setNormalMode();
  
//  Serial.println("------- CAN Read ----------");
//  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    ///*
    if(canMsg.can_id == 2566001651){ //BMS 2

      V = (String(canMsg.data[1],DEC).toFloat() * 256.0 + String(canMsg.data[0],DEC).toFloat()) * 0.1;
      I = (String(canMsg.data[3],DEC).toFloat() * 256.0 + String(canMsg.data[2],DEC).toFloat()) * 0.1 - 320.0;
      SOC = String(canMsg.data[6],DEC).toInt();

      Serial.println("V = " + String(V) + " | " + "I = " + String(I) +  " | " + "SOC = " + String(SOC));

    }
    
    if(canMsg.can_id == 2566002163){ //BMS4


      Th = String(canMsg.data[0],DEC).toInt() - 40 ;
      Tl = String(canMsg.data[2],DEC).toInt() - 40;

      Serial.println("Th = " + String(Th) + " | " + " Tl = " + String(Tl) );
      /*
      Serial.print(canMsg.can_id, HEX); // print ID
      Serial.print(" "); 
      Serial.print(canMsg.can_dlc, HEX); // print DLC
      Serial.print(" ");
    
   // */
    /*for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],DEC);
      Serial.print(" ");
    }*/
    }
          
  }
}
