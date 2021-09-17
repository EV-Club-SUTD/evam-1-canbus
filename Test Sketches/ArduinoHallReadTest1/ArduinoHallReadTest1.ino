

unsigned long lastHallMillis = 0;
unsigned long interval = 0;
float rpm = 0.0;
unsigned long lastPrintMillis = 0;
#define PRINT_INTERVAL 500
uint8_t buttonState = 0;
uint8_t lastState = 0;

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin


void setup() {
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  Serial.begin(115200);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  //Serial.println(buttonState);
  
  if (buttonState == HIGH && lastState == 0) {
    interval = millis() - lastHallMillis;
    lastHallMillis = millis();
    //Serial.print("Pulse! | ");
    //Serial.println(lastHallMillis);
    //rpm  = 60000.0/(float)interval;  
    lastState = 1;  
  }
  else if (buttonState == LOW){
    lastState=0;
  }
  if (millis() - lastPrintMillis > PRINT_INTERVAL){
    //Serial.print("RPM: ");
    //Serial.println(rpm);
    Serial.print("Interval: ");
    Serial.println(interval);
    lastPrintMillis = millis();
  }
}
