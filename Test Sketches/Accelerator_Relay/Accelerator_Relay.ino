int acc_in_pin = 2;    // select the input pin for the potentiometer
int val = 0;       // variable to store the value coming from the sensor
int val_percentage = 0;
int acc_out_pin = 3;

void setup() {
  pinMode(acc_in_pin, INPUT);
  pinMode(acc_out_pin, OUTPUT);
  Serial.begin(9600);
}

int read_write_acc(){
  val = analogRead(acc_in_pin);    // read the value from the sensor
  val_percentage = float(val)/871 * 255;
  if (val_percentage > 255){
    val_percentage = 255;
  }

  analogWrite(acc_out_pin, val_percentage);
  
  return val_percentage;
}

void loop() {
  Serial.println(read_write_acc());
  delay(50);   
            
}
