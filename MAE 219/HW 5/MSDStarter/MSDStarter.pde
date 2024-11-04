// MAE 207 Winter 2021
// HW6_P2 Starter code


import processing.serial.*;

Serial myPort;        // The serial port

//initialize all variables
float inByte = 0; //current value of the first variable in the string
float lastByte = 0; //previous value of the first variable in the string
float inByte2 = 0; //current value of the second variable in the string
float lastByte2 = 0; //previous value of the second variable in the string

int x_wall = 550;   // position of wall
int w = 50;   // width of mass
int r = 25;   // radius of proxy

String input1;   // serial input 1
float x1;   // input 1 converted
float xh;   // position of proxy

String input2;   // serial input 2
float x2;   // input 2 converted
float xm;   // position of mass

void setup () {
  // set the window size:
  size(600, 400);        

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  //note you may need to change port number, it is 9 for me
  myPort = new Serial(this, Serial.list()[3], 38400);  // also make sure baud rate matches Arduino
  

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
}
void draw () {
  // everything happens in the serialEvent()
  background(0); //uncomment if you want to control a ball
  stroke(127,34,255);     //stroke color
  strokeWeight(8);        //stroke wider
  
  //START EDITING HERE
  //stroke(r,g,b);     //stroke color
  //strokeWeight(num);        //stroke wider
  
  
  // Mass Spring Damper
  
  //draw the wall
  line(x_wall, 0, x_wall, 400);
  //draw a line from the wall to the xMass
  stroke(127,127,127);
  line(x_wall, 200, xm+w/2, 200);
  //draw an ellipse to represent the mass of the spring-mass-damper
  strokeWeight(0);
  fill(255,255,255);     //fill color
  rect(xm,200-w/2,w,w);
  //draw an ellipse to represent the user
  fill(255,255,255);     //fill color
  ellipse(xh-r, 200, 2*r, 2*r);
  
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  
  //For Mass Spring Damper
  // make sure to match variable names with what has been declared above in lines 10-13 (or change the original variable names if you wish)
  
  // read the first part of the input string
  // HINT: use myPort.readStringUntil(' ') 
  input1 = myPort.readStringUntil(' ');
  // trim and convert string to a number
  x1 = float(trim(input1));
  // if: the number is NaN, set current value to previous value
  // otherwise: map the new value to the screen width
  //           & update previous value variable
  if (x1 == x1) {
    // HINT: use map(myValue, minValueIn, maxValueIn, minValueOut, maxValueOut) to map from units of your Arduino simulation to pixels 
    xh = map(x1, -0.05, 0.05, 0, 600);
  }
  
  // repeat for second part of the input string, this time using myPort.readStringUntil('\n')
  input2 = myPort.readStringUntil('\n');
  x2 = float(trim(input2));
  if (x2 == x2) {
    // HINT: use map(myValue, minValueIn, maxValueIn, minValueOut, maxValueOut) to map from units of your Arduino simulation to pixels 
    xm = map(x2, -0.05, 0.05, 0, 600);
  }
  //STOP EDITING HERE
  
}
