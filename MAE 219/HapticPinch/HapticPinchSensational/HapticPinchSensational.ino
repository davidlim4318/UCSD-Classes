#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

// Pins
int pina = A0;
int pinb = A1;
int pinpot = A5;
int pin1 = 45;
int pin2 = 47;
int pin3 = 49;
int pin4 = 51;
int pin5 = 53;

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

void setup() {
  // Setup
  Serial.begin(19200);
  pinMode(pina,INPUT_PULLUP);
  pinMode(pinb,INPUT_PULLUP);
  pinMode(pinpot,INPUT);
  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
  pinMode(pin3, INPUT_PULLUP);
  pinMode(pin4, INPUT_PULLUP);
  pinMode(pin5, INPUT_PULLUP);
  dac.begin(0x61);
  jam(pressureATM);
  
  if (mode == 0) {
    // User calibration
    calibrateUser();
  }
}

void loop() {

  changeState();
  // Read-in currennt positions
  readIn();
  // Check for collisions
  if (mode > 0) {
    checkCollision();
  }

  switch (mode) {
    case 1:
      blockSoftness = 0.5;
      blockPoisson = 1;
      if (isSqueeze) {
        if (w > x2 - x1 + threshold) {
          h = h + w - (x2 - x1 + threshold);
          w = x2 - x1 + threshold;
        }
      }
      if (x2 - x1 > 75) {
        h = 60;
        w = 60;
      }
      p = map(pot,0,1023,0,900);
      break;
    case 2:
      blockSoftness = 0.5; // map(pot,0,1023,0,0.5);
      blockPoisson = 1;
      w = 40;
      h = 40;
      p = map(pot,0,1023,0,900);
      break;
    case 3:
      blockSoftness = 0.5;
      blockPoisson = 1;
      w = map(pot,0,1023,0,200);
      h = w;
      p = 0;
      break;
    case 4:
      blockSoftness = 0.5;
      blockPoisson = 1;
      w = 40;
      h = 40;
      p = 700 - map(pot,0,1023,0,50)*(w-(x2-x1));
      break;
    case 5:
      blockSoftness = 0.5;
      blockPoisson = 1;
      if (isSqueeze) {
        if (x2 - x1 < 5) {
          h = 0;
          w = 0;
          p = pressureATM;
        }
        else {
          h = 50;
          w = 50;
          p = 0;
        }
      }
      if (x2 - x1 > 75) {
        h = 50;
        w = 50;
      }
      break;
    default:
      Serial.println("Pick a mode!");
      break;
  }

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

  if (mode > 0) {
    // Jam user
    jam(pressure);

    // Print data to Processing
    printData();
  }
}

void changeState() {
  if (digitalRead(pin1) == LOW) {
    mode = 1;
    Serial.println("Playdoh");
  }
  else if (digitalRead(pin2) == LOW) {
    mode = 2;
    Serial.println("Adjustable Stiffness Ball");
  }
  else if (digitalRead(pin3) == LOW) {
    mode = 3;
    Serial.println("Adjustable Size Ball");
  }
  else if (digitalRead(pin4) == LOW) {
    mode = 4;
    Serial.println("Variable Stiffness Ball");
  }
  else if (digitalRead(pin5) == LOW) {
    mode = 5;
    Serial.println("Balloon");
  }
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
  Serial.print(blockSoftness);
  Serial.print(",");
  Serial.print(blockPoisson);
  Serial.print(",");
  Serial.print(mode);
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