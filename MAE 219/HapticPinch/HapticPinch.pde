import processing.serial.*;

Serial myPort;        // The serial port

int textSize = 20;  // pixels

float scaleFactor = 5;  // pixels per mm

float blockSoftness = 0.5;  // strain ratio
float blockPoisson = 1;  // Poisson's ratio

float fingerWidth = 10;  // mm
float fingerHeight = 15; // mm

float w;
float h;
float x1;
float x2;
float xb;
float p;

int isSqueeze;

String message = "Waiting for data...";

void setup() {
  println(Serial.list());
  myPort = new Serial(this, Serial.list()[3], 19200);
  myPort.bufferUntil('\n');
  
  frameRate(30);
  size(720, 480);
}

void draw() {
  float blockWidth = w;
  float blockHeight = h;
  float finger1Position = x1;
  float finger2Position = x2;
  if (x2 - x1 <= w) {
    blockWidth = max(w - blockSoftness*(w - (x2 - x1)),0);
    blockHeight = max(h + blockPoisson*blockSoftness*(w - (x2 - x1)),0);
    finger1Position = xb - blockWidth/2;
    finger2Position = xb + blockWidth/2;
  } 
  
  background(255);
  noStroke();
  
  fill(0, 0, 0);
  textSize(textSize);
  textAlign(LEFT, TOP);
  text("Frame Rate: " + int(frameRate) + " fps", 10, 10);
  text("Penetration: " + min(0,int((x2 - x1)-w)) + " mm", 10, 10+textSize);
  text("Pressure: " + int(0.1*p-90) + " kPa", 10, 10+2*textSize);
  text("isSqueeze: " + isSqueeze, 10, 10+3*textSize);
  textAlign(CENTER, BOTTOM);
  text(message, width/2, height);
  
  translate(width/2, height/2);
  scale(scaleFactor, -scaleFactor);
  
  fill(173, 216, 230);
  ellipse(finger1Position-fingerWidth/2, 0, fingerWidth, fingerHeight);
  ellipse(finger2Position+fingerWidth/2, 0, fingerWidth, fingerHeight);
  
  fill(255, 182, 193);
  rectMode(CENTER);
  ellipse(xb, 0, blockWidth, blockHeight);
}

void serialEvent (Serial myPort) {
  String input = myPort.readStringUntil('\n');
  if (input != null) {
    try {
      input = trim(input);
      String[] inputArray = split(input,',');
      w = float(inputArray[0]);
      h = float(inputArray[1]);
      x1 = float(inputArray[2]);
      x2 = float(inputArray[3]);
      xb = float(inputArray[4]);
      p = float(inputArray[5]);
      blockSoftness = float(inputArray[6]);
      blockPoisson = float(inputArray[7]);
      isSqueeze = int(inputArray[8]);
    } catch (Exception e) {
      message = input;
    }
  }
}
