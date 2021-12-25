#include <Arduino.h>
#include <ESP32CAN.h> //ESP32-Arduino-CAN-master, https://github.com/miwagner/ESP32-Arduino-CAN
//there was another CAN library based on the native ESP32 CAN implementation (I THINK!!) but I can't find it now
#include <CAN_config.h>

//Tested while connected to an Arduino MCP node, ESP canbus is able to receive
//10kOhm resistor connected to RS pin on the 230D (slope control) removed, and replaced with direct connection to ground
//(disable slope control - enter high speed mode)
//But ESP CAN bus is still not able to send any messages, or acknowledge the Arduino canbus messages

CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_14;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
}

void loop() {

  CAN_frame_t rx_frame;

  unsigned long currentMillis = millis();

  // Receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {

    if (rx_frame.FIR.B.FF == CAN_frame_std) {
      Serial.println("New standard frame");
    }
    else {
      Serial.println("New extended frame");
    }
    
    if (rx_frame.FIR.B.RTR == CAN_RTR) {
      //printf(" RTR from 0x%08X, DLC %d\r\n", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
    }
    else {
      //printf(" from 0x%08X, DLC %d, Data ", rx_frame.MsgID,  rx_frame.FIR.B.DLC);
      for (int i = 0; i < rx_frame.FIR.B.DLC; i++) {
        Serial.print(rx_frame.data.u8[i]);
        Serial.print("  ");
      }
      Serial.println();
    }
    
  }
  
   /*if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.println("ping");
   }
   */
  // Send CAN Message
  /*
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.MsgID = 0x001;
    tx_frame.FIR.B.DLC = 8;
    tx_frame.data.u8[0] = 0x00;
    tx_frame.data.u8[1] = 0x01;
    tx_frame.data.u8[2] = 0x02;
    tx_frame.data.u8[3] = 0x03;
    tx_frame.data.u8[4] = 0x04;
    tx_frame.data.u8[5] = 0x05;
    tx_frame.data.u8[6] = 0x06;
    tx_frame.data.u8[7] = 0x07;
    ESP32Can.CANWriteFrame(&tx_frame);
    Serial.println("Frame Sent");
  }
  */
}
