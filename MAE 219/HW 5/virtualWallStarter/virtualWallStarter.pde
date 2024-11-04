// Pro_Graph2.pde
/*
 Based on the Arduining example which is based on the Tom Igoe example.
 Mofified by Cara Nunez 5/1/2019:
  -A wider line was used. strokeWeight(4);
  -Continuous line instead of vertical lines.
  -Bigger Window size 600x400.
 Last Modified by Tania Morimoto 2/14/2023:
  - more detailed comments/instructions
-------------------------------------------------------------------------------
This program takes ASCII-encoded strings
from the serial port at 38400 baud and graphs them. It expects values in the
range 0 to 1023, followed by a newline, or newline and carriage return


*/

import processing.serial.*;

Serial myPort;        // The serial port

//initialize variables
float inByte = 0;
float lastByte = 0;

String input;
float xh;
float xp;
int r = 20;
int x_wall = 330;

void setup () {
  // set the window size:
  size(600, 400);        

  // List all the available serial ports
  println(Serial.list());
  
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[] below.
  // Note that these are indexed from 0, and you are looking for the same port as your ardunio.
  myPort = new Serial(this, Serial.list()[3], 38400);  //make sure baud rate matches Arduino

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
}
void draw () {
  // everything happens in the serialEvent()
  background(0); //uncomment if you want to control a ball
  strokeWeight(4);        //stroke wider
  stroke(127,34,255);     //stroke color
  
  // START EDITING HERE
  
  // Virtual Wall
  // map the wall position from units of Arduino simulation to the screen width.
  // HINT: use map(myValue, minValueIn, maxValueIn, minValueOut, maxValueOut) to map from units of your Arduino simulation to pixels
  xp = map(xh, -0.05, 0.05, 0, 600);
  if (xp > x_wall) {
    xp = x_wall;
  }
  // draw the wall as a line (use "line" function: line(x1, y1, x2, y2))
  line(x_wall, 0, x_wall, 400);
  // draw an ellipse to represent user position (ellipse(x-coord, y-coord, ellipse width, ellipse height))
  fill(127,34,255);     //fill color
  ellipse(xp-r, 200, 2*r, 2*r);

}

void serialEvent (Serial myPort) {
  // get the ASCII string:

  // read the input string
  // HINT: use myPort.readStringUntil('\n')  (https://processing.org/reference/libraries/serial/Serial_readStringUntil_.html)
  input = myPort.readStringUntil('\n');
  // if: the number is NaN, set current value to previous value
  // otherwise: 
  if (input != null) {
    // 1. trim using trim()
    input =  trim(input);
    // 2. convert string to a number (i.e. a float)
    // 3. update previous value variable
    xh = float(input);
  }
  
  //STOP EDITING HERE
}
