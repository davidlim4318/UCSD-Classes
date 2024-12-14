#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

// Pins
int pina = A0;
int pinb = A1;
int pinpot = A5;
int pin1 = 2;
int pin2 = 3;

// Mode
int val1Ref = 424;
int val2Ref = 628;
float scale = 0.26;  // mm per unit
int mode = 0;  // modes: calibration, 

// Sensor variables
int val1;
int val2;
int pot;

// Virtual object variables
float pressureATM = 900;  // mV
float p = pressureATM;  // mV
float pressureMax;

float w = 40;  // mm
float h = 40;  // mm
float x1;  // mm
float x2;  // mm
float xb; // mm
float pressure;  // mV
float blockSoftness = 0.5;  // strain ratio
float blockPoisson = 1;  // Poisson's ratio

float minDist = w;  // mm
float maxDist;  //mm
float threshold = 5;  // mm
bool isSqueeze;

unsigned long startTime;

unsigned long buttonTime;
int i;
int myArray[180]={450,360,450,240,450,360,450,360,450,270,450,240,450,270,450,330,450,390,450,390,450,240,450,240,450,330,450,240,450,240,450,330,450,420,450,330,450,300,450,420,450,390,450,240,450,300,450,240,450,240,450,240,450,420,450,240,450,240,450,390,450,240,450,270,450,240,450,330,450,240,450,270,450,240,450,390,450,360,450,270,450,240,450,330,450,240,450,270,450,360,450,300,450,420,450,240,450,330,450,360,450,270,450,390,450,240,450,330,450,270,450,300,450,360,450,300,450,420,450,420,450,390,450,420,450,270,450,300,450,420,450,240,450,270,450,330,450,300,450,420,450,390,450,240,450,300,450,330,450,390,450,240,450,240,450,390,450,240,450,240,450,240,450,300,450,240,450,360,450,420,450,240,450,360,450,240,450,300,450,360};


void setup() {
  // Setup
  Serial.begin(9600);
  pinMode(pina,INPUT_PULLUP);
  pinMode(pinb,INPUT_PULLUP);
  pinMode(pinpot,INPUT);
  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
  dac.begin(0x61);
  jam(pressureATM);
  
  if (mode == 0) {
    // User calibration
    calibrateUser();
  }
}

void loop() {
  // Read-in currennt positions
  readIn();
  // Check for collisions
  checkCollision();

  if (millis() > buttonTime + 500) {
    if (digitalRead(pin1) == LOW) {
      i = i + 1;
      buttonTime = millis();
      printData();
    } else if (digitalRead(pin2) == LOW) {
      i = i - 1;
      buttonTime = millis();
      printData();
    }
  }

  p = myArray[i];

  if (isSqueeze) {
    pressure = constrain(p,0,1000);
  }
  else {
    if (millis() - startTime < 200) {
      pressure = pressureATM + 60;
    }
    else{
      pressure = pressureATM;
    }
  }

  // Jam user
  jam(pressure);
}

void readIn() {
  val1 = analogRead(pina);
  val2 = analogRead(pinb);
  x1 = (val1 - val1Ref) * scale;
  x2 = -(val2 - val2Ref) * scale;
  pot = analogRead(pinpot);
}

void checkCollision() {
  if (x2 - x1 <= w) {
    if (minDist > (x2 - x1)) {
      minDist = x2 - x1;
    }
    else if (minDist < (x2 - x1) - threshold - 1) {
      minDist = (x2 - x1) - threshold;
    }
    if ((x2 - x1) - threshold < minDist) {
      isSqueeze = true;
    }
    else {
      if (isSqueeze) {
        startTime = millis();
        isSqueeze = false;
      }
    }
    xb = (x1 + x2) / 2;
  }
  else {
    if (isSqueeze) {
      startTime = millis();
      isSqueeze = false;
    }
    if (xb < x1 + w/2) {
      xb = x1 + w/2;
    }
    if (xb > x2 - w/2) {
      xb = x2 - w/2;
    }
  }
}

void jam(float p) {
    int output = constrain(p,0,1000);
    dac.setVoltage(output, false);
}

void printData () {
  Serial.print(i);
  Serial.print(" ");
  Serial.print(myArray[i]);
  Serial.println(" ");
}

void calibrateUser () {
  Serial.println(" ");
  Serial.println("Please touch your finger tips together.");
  delay(3000);
  readIn();
  val1Ref = val1;
  val2Ref = val2;
  Serial.println("Now, please separate your finger tips by about 8 centimeters.");
  delay(3000);
  readIn();
  scale = 80.0 / (abs(val1 - val1Ref) + abs(val2 - val2Ref));
  // Read out calibration data
  // Serial.print(val1Ref);
  // Serial.print(",");
  // Serial.print(val2Ref);
  // Serial.print(",");
  // Serial.print(scale);
  // Serial.println(" ");
  Serial.println("Pick a mode!");
}