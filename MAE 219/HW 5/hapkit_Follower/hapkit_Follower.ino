int mrsensorpin = A2;
int sensorPos = 0; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);

  pinMode(mrsensorpin, INPUT);
  
}

void loop() {
  // nothing
}
