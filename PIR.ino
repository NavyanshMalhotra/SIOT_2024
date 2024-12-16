int sensor = 3;              // the pin that the sensor is atteched to 3 (right side -> 3)
int state = LOW;             // by default, no motion detected
int val = 0;                 // variable to store the sensor status (value)

void setup() {
  pinMode(sensor, INPUT); 
  Serial.begin(9600);      
}

void loop(){
  val = digitalRead(sensor);   
  Serial.println(val);
  delay(500);
}