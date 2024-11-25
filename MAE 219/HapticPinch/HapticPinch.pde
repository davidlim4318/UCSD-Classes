
int textSize = 20;  // pixels

float scaleFactor = 5;  // pixels per mm

float fingerWidth = 10;  // mm
float fingerHeight = 15; // mm

float fingerDistance = 60;  // mm
float fingerForce;
boolean pressed;

float blockPoisson = 0.5;  // Poisson's ratio
float blockWidthDim = 30;  // mm
float blockHeightDim = 30;  // mm
float blockWidth = blockWidthDim;
float blockHeight = blockHeightDim;

void setup() {
  frameRate(30);
  size(720, 480);
  resetBlock();
}

void draw() {
  blockWidth = min(blockWidthDim, fingerDistance);
  blockHeight = blockHeightDim + blockPoisson*(blockWidthDim-blockWidth);
  
  if (fingerDistance < blockWidthDim) {
    if (fingerForce == 0) {
      blockWidthDim = blockWidth;
      blockHeightDim = blockHeight;
    } else {
      pressed = true;
    }
  } else {
    pressed = false;
  }
  
  background(255);
  noStroke();
  
  fill(0, 0, 0);
  textSize(textSize);
  textAlign(LEFT, TOP);
  text("Frame Rate: " + int(frameRate) + " fps", 10, 10);
  text("Distance: " + int(fingerDistance) + " mm", 10, 10+textSize);
  text("Pressed: " + pressed, 10, 10+2*textSize);
  text("Force: " + fingerForce, 10, 10+3*textSize);
  
  translate(width/2, height/2);
  scale(scaleFactor, -scaleFactor);
  
  fill(173, 216, 230);
  ellipse((fingerDistance+fingerWidth)/2, 0, fingerWidth, fingerHeight);
  ellipse(-(fingerDistance+fingerWidth)/2, 0, fingerWidth, fingerHeight);
  
  fill(255, 182, 193);
  rectMode(CENTER);
  rect(0, 0, blockWidth, blockHeight);
}

void keyReleased() {
  resetBlock();
}

void resetBlock(){
  blockWidthDim = int(random(10,50));  // mm
  blockHeightDim = int(random(10,50));  // mm
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount() / scaleFactor;
  fingerDistance += e;
}

void mousePressed(MouseEvent event) {
  fingerForce = 1;
}

void mouseReleased(MouseEvent event) {
  fingerForce = 0;
}
