//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// 04.11.14
// Last updated by David Lim 10.18.24
//--------------------------------------------------------------------------

// Includes
#include <math.h>

// Experiment switching
#define parta1
// #define parta2
// #define partb
// #define partc
// #define parte

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;
double OFFSET = 980;
double OFFSET_NEG = 15;

// Kinematics variables
double xh = 0;           // position of the handle [m]

// Time and time step variables
double t;   // time [s]
double tLast;   // time [s] from previous loop
double dt;   // time step [s]

// Velocity estimate variables
double alpha = 0.10;   // IIR filter parameter
double xhLast;   // xh [m] from previous loop
double vh;   // velocity [m/s] of the handle, unfiltered
double vhFilt;   // vh [m/s], IIR filtered

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

#ifdef partc
  int T = 10;   // experiment time
  bool stop;   // true when experiment has ended
  int count;   // number of loops
#endif

// Virtual wall variables
#ifdef parte
  double xWall = 0.005;   // position [m] of the wall
  double k = 600;   // stiffness [N/m] of the wall
#endif

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(115200);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  flipNumber = 0;
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped = false;
  }
   updatedPos = rawPos + flipNumber*OFFSET; // need to update pos based on what most recent offset is 
 
  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************

  // Define kinematic parameters you may need
  double rh = 0.070;   // radius [m] of the handle
  // Step B.1: print updatedPos via serial monitor
  //Serial.print("updatedPos:");
  //Serial.print(updatedPos);
  //Serial.print(" ");
  // Step B.6: double ts = ?; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts = 0.01326 * updatedPos - 7.28;
  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)
  double xh = rh * PI * ts / 180.;
  // Step B.8: print xh via serial monitor
  //Serial.print("xh:");
  //Serial.print(xh,4);
  //Serial.print(" ");
  
  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
 
  // Define kinematic parameters you may need
  double rp = 0.005;   // radius [m] of the pulley
  double rs = 0.074;   // radius [m] of the sector
  // Step C.1: force = ?; // You will generate a force by simply assigning this to a constant number (in Newtons)
  // force = 0.12;
  // Step C.2: Tp = ?;    // Compute the require motor pulley torque (Tp) to generate that force
  // Tp = rh * rp * force / rs;

  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************

  tLast = t;   // store previous time [s]
  t = micros() / 64. / 1000000.;   // store current time (scaled by 1/64 since clock speed is affected by PWM code)
  dt = t - tLast;   // compute time step

  // Velocity estimate and filtering
  vh = (xh - xhLast) / (t - tLast);   // compute the velocity of the handle with numerical differentiation of the handle position (first-order divided difference)
  vhFilt = alpha*vh + (1 - alpha)*vhFilt;   // filter vh using a first-order IIR filter
  xhLast = xh;   // store the previous handle position
  // Serial.print("vh:");
  // Serial.print(vh,4);
  // Serial.print(" ");
  // Serial.print("vhFilt:");
  // Serial.print(vhFilt,4);
  // Serial.print(" ");

  //******************* Part A1: Coulomb friction estimate *******************
  #ifdef parta1
    force = 0.001 * t + 0.25;   // increase the force slowly
    Serial.print("force:");
    Serial.print(force,3);   // print the current force
    Serial.println(" ");
  #endif

  //******************* Part A2: Damping estimate *******************
  #ifdef parta2
  double b = 1.33;
    if (abs(vhFilt) > 0.005) {   // if the filtered velocity magnitude is greater than a threshold
      force = b * vhFilt;   // compute damping force
    }
    else {   // if the filtered velocity magnitude is less than the threshold
      force = 0;   // force is zero
    }
  #endif

  //******************* Part B: Position Quantization Estimation *******************
  #ifdef partb
    Serial.print("updatedPos:");
    Serial.print(updatedPos);   // print the position in counts
    Serial.println(" ");
  #endif

  //******************* Part C: Loop Period Estimation *******************
  #ifdef partc
    if (!stop) {   // if experiment allowed to continue
      count = count + 1;   // increment loop counter
    }
    if (t > T) {   // if time elapsed exceeds experiment time
      force = 0;
      stop = true;   // stop the experiment
      Serial.print("Time:");
      Serial.print(t,6);
      Serial.println(" ");
      Serial.print("Count:");
      Serial.print(count);
      Serial.println(" ");
      Serial.print("Average period:");
      Serial.print(t/count,6);   // print the result
      Serial.println(" ");
      while (1) {   // stop void loop
        delay(1000);
      }
    }
  #endif

  //******************* Part E: Virtual Wall *******************
  #ifdef parte
    if (xh > xWall) {   // if the handle is inside the wall
      force = k * (xWall - xh);   // compute spring force
    }
    else {   // if the handle is outside the wall
      force = 0;   // force is zero
    }
    Serial.println(updatedPos);
  #endif
  
  Tp = rh * rp * force / rs;   // compute the torque required to output the desired force

  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force > 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.0183);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal
}

// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
