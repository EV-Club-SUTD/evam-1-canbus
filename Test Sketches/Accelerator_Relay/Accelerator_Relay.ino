#define ACC_PIN A0    // select the input pin for the potentiometer
#define ACC_PIN2 A1    // select the input pin for the backup potentiometer
int val = 0;       // variable to store the value coming from the sensor
int val2 = 0;       // variable to store the value coming from the sensor

void setup() {
  pinMode(ACC_PIN, INPUT);
  pinMode(ACC_PIN2, INPUT);
  Serial.begin(115200);
}

int read_write_acc(){
  val = analogRead(ACC_PIN);    // read the value from the sensor
  val = map(val, 129, 928, 0, 250);
  val2 = analogRead(ACC_PIN2);    // read the value from the sensor
  val2 = map(val2, 550, 957, 250, 0);
  int diff = val - val2;
  Serial.println(String(val) + " | " + String(val2) + " | " + String(diff));
}

void loop() {
  read_write_acc();
  delay(50);   
            
}
