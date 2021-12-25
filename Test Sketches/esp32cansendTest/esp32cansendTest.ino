#include <ESP32CAN.h>
#include <CAN_config.h>

CAN_device_t CAN_cfg;

void setup() {
    Serial.begin(115200);
    Serial.println("iotsharing.com CAN demo");
    CAN_cfg.speed=CAN_SPEED_500KBPS;
    CAN_cfg.tx_pin_id = GPIO_NUM_5;
    CAN_cfg.rx_pin_id = GPIO_NUM_14;
    CAN_cfg.rx_queue = xQueueCreate(10,sizeof(CAN_frame_t));
    //initialize CAN Module
    ESP32Can.CANInit();
}

void loop() {
    CAN_frame_t tx_frame;
    tx_frame.FIR.B.FF = CAN_frame_std;
    tx_frame.FIR.B.DLC = 1;
    tx_frame.MsgID = 10;
    tx_frame.data.u8[0] = 100;
    //receive next CAN frame from queue
//    if(xQueueReceive(CAN_cfg.rx_queue,&rx_frame, 3*portTICK_PERIOD_MS)==pdTRUE){
//
//      //do stuff!
//      if(rx_frame.FIR.B.FF==CAN_frame_std)
//        printf("New standard frame");
//      else
//        printf("New extended frame");
//
//      if(rx_frame.FIR.B.RTR==CAN_RTR)
//        printf(" RTR from 0x%08x, DLC %d\r\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);
//      else{
//        printf(" from 0x%08x, DLC %d\n",rx_frame.MsgID,  rx_frame.FIR.B.DLC);
//        /* convert to upper case and respond to sender */
//        for(int i = 0; i < 8; i++){
//          if(rx_frame.data.u8[i] >= 'a' && rx_frame.data.u8[i] <= 'z'){
//            rx_frame.data.u8[i] = rx_frame.data.u8[i] - 32;
//          }
//        }
//      }
    //respond to sender
    ESP32Can.CANWriteFrame(&tx_frame);
    delay(100);

}
