/* Refer to motor controller document to connect to CAN Bus
 * Probably needs the 120 ohm resistor
 * and hope for the best
 * LOOKS LIKE: THE MOTOR CONTROLLER DOESN'T HAVE A CANBUS :/
 */

#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);

void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    
    if(canMsg.can_id == 284692634){ //Data frame 1
        bool dir = canMsg.data[0];
        uint16_t speed = canMsg.data[1] + canMsg.data[2]<<8;
        uint8_t fault = canMsg.data[3];
        uint8_t lowPower = canMsg.data[4];
        uint16_t mileage = canMsg.data[1] + canMsg.data[2]<<8;

      Serial.println("Direction: " + String(dir>>6) + " | Speed: " + String(speed) + " | Fault: " + String(fault) + " | Low Power?: " +((lowPower==0xAA) ? ("Yes") : ("No ")) + "Mileage: " + String(mileage)/10);

    }
    
    if(canMsg.can_id == 284692621){ //data frame 1
        uint16_t voltage = canMsg.data[0] + canMsg.data[1]<<8;
        uint16_t current = canMsg.data[2] + canMsg.data[3]<<8;

        Serial.println("Voltage = " + String(voltage/10) + " | " + " Current = " + String(current/10) );
    }
          
  }
}