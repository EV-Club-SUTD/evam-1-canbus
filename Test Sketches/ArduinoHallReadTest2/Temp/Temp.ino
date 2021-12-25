/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogReadSerial
*/
#define pulsePin 2
unsigned long pulseMicros = 0;
unsigned long lastPulseMicros = 0;
volatile unsigned long pulseInterval = 1000;
uint16_t motorSpeed = 0;
unsigned long printMillis = 0;
volatile uint8_t hallLvl = 1;
volatile uint8_t prevHallLvl = 0;
volatile uint8_t change = 0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(500000);
  pinMode(pulsePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pulsePin), pulseISR, FALLING);
}

// the loop routine runs over and over again forever:
void loop() {
  /*
  // read the input on analog pin 0:
  //int sensorValue = analogRead(A0);
  uint8_t newSensorVal = digitalRead(pulsePin);
  if (newSensorVal == 1 && sensorVal == 0){  //rising
    pulseInterval = millis()-pulseMillis;
    pulseMillis = millis();
    sensorVal = newSensorVal;
    */
    if (change == 1){
      noInterrupts();
      
      unsigned long pulseInterval = pulseMicros - lastPulseMicros;
      lastPulseMicros = pulseMicros;
      
      interrupts();
      
      float motorSpeedNew = 60.0 * 1000000.0 / (pulseInterval * 16.0);
      motorSpeed = uint16_t(motorSpeedNew); //nofilter
      //motorSpeed = (motorSpeed *3 + uint16_t(motorSpeedNew) * 7)/10;  //exponential filter
      change = 0;
      Serial.println(motorSpeed);
  }
  /*
  else if(newSensorVal == 0 && sensorVal == 1){ //falling
    sensorVal = 0;
  }
  */
  /*
  if (millis()-printMillis > 100){
    Serial.println(motorSpeed);
    printMillis = millis();
  }
  */
  //wheel timeout
  if (micros() - lastPulseMicros > 300000){
    motorSpeed = 0;
  }
}

void pulseISR() {
  if(micros() - lastPulseMicros >1000){
    pulseMicros = micros();
    change = 1;
  }
  /*
    if(micros() - lastPulseMicros >1500){
    hallLvl = digitalRead(pulsePin);
    if(hallLvl == 1){ //check
      change = 1;
      pulseMicros = micros();
    }
    }
    */
  /*
    uint8_t hallLvlTemp = digitalRead(pulsePin);
    if((hallLvlTemp != prevHallLvl)){ //check
      change = 1;
      pulseMicros = micros();
      hallLvl = hallLvlTemp;
      prevHallLvl = hallLvl;
    }
    */
    
}
