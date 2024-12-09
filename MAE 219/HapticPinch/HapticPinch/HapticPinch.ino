#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

// Pins
int pin1 = A0;
int pin2 = A1;

// Sensor variables
int val1;
int val2;
int val1Ref;
int val2Ref;
float scale;  // mm per unit

// Virtual object variables
float pressure = 900;  // mV
float w = 40;  // mm
float h = 40;  // mm
float x1;  // mm
float x2;  // mm
float xb; // mm

float minDist = w;  // mm
float maxDist;  //mm
float threshold = 5;  // mm
bool isSqueeze = true;

unsigned long startTime;

void setup() {
  // Setup
  Serial.begin(19200);
  pinMode(pin1,INPUT_PULLUP);
  pinMode(pin2,INPUT_PULLUP);
  dac.begin(0x61);
  jam(pressure);

  // User calibration
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
  Serial.println("Running simulation ...");
}

void loop() {
  // Read-in currennt positions
  readIn();

  // Check for collisions
  checkCollision();

  // Print data to Processing
  printData();
}

void readIn() {
  val1 = analogRead(pin1);
  val2 = analogRead(pin2);
  x1 = (val1 - val1Ref) * scale;
  x2 = -(val2 - val2Ref) * scale;
}

void checkCollisionRight() {
  if (x2 - x1 <= w) {
    if (minDist > (x2 - x1)) {
      minDist = x2 - x1;
    }
    else if (minDist < (x2 - x1) - threshold - 1) {
      minDist = (x2 - x1) - threshold;
    }
    if ((x2 - x1) - threshold < minDist) {
      pressure = 0;
    }
    else {
      pressure = 900;
    }
    xb = (x1 + x2) / 2;
  }
  else {
    pressure = 900;
    if (xb < x1 + w/2) {
      xb = x1 + w/2;
    }
    if (xb > x2 - w/2) {
      xb = x2 - w/2;
    }
  }
}

void checkCollision() {
  if (x2 - x1 <= w) {
    if (minDist > (x2 - x1) && isSqueeze) {
      minDist = x2 - x1;
    }
    if (maxDist < (x2 - x1) && !isSqueeze) {
      maxDist = x2 - x1;
    }

    if (((x2 - x1) > (minDist + threshold)) && isSqueeze) {
      startTime = millis();
      isSqueeze = false;
      maxDist = minDist;
    }
    
    if (((x2 - x1) < (maxDist - threshold)) && !isSqueeze) {
      isSqueeze = true;
      minDist = maxDist;
    }
  }
  if (x2 - x1 <= w & isSqueeze) {
    pressure = 0;
    // if (x2 - x1 <= w - threshold) {
    //   w = x2 - x1 + threshold;
    // }
    // pressure = constrain(500 - 18*(w-(x2-x1)),0,950);
    jam(pressure);
    xb = (x1 + x2) / 2;
  }
  else if (x2 - x1 <= w & !isSqueeze) {
    if (millis() - startTime < 200) {
        pressure = 960;
    }
    else{
      pressure = 900;
    }
    jam(pressure);
    xb = (x1 + x2) / 2;
  }
  else {
    maxDist = w + threshold;
    isSqueeze = false;
    pressure = 900;
    jam(pressure);
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
  Serial.print(w);
  Serial.print(",");
  Serial.print(h);
  Serial.print(",");
  Serial.print(x1);
  Serial.print(",");
  Serial.print(x2);
  Serial.print(",");
  Serial.print(xb);
  Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.print(isSqueeze);
  Serial.println(" ");
}