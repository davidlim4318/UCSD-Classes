// Includes
#include <math.h>

#define TELEOP

#define E_Value 2.71828     // Value for e

// Pin declares
int pwmPin1 = 5; // PWM output pin for motor 1
int dirPin1 = 8; // direction output pin for motor 1
int pwmPin2 = 6; // PWM output pin for motor 2
int dirPin2 = 7; // direction output pin for motor 2
int sensorPosPin1 = A2; // input pin for MR sensor 1
int sensorPosPin2 = A3; // input pin for MR sensor 2

// **************************************************************
// **************************************************************
// **************************************************************

double rs = 0.073152;   //[m]
double rp = 0.004191;   //[m]

// Position tracking variables
int updatedPos1 = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos1 = 0;         // current raw reading from MR sensor
int lastRawPos1 = 0;     // last raw reading from MR sensor
int lastLastRawPos1 = 0; // last last raw reading from MR sensor
int flipNumber1 = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset1 = 0;
int rawDiff1 = 0;
int lastRawDiff1 = 0;
int rawOffset1 = 0;
int lastRawOffset1 = 0;
boolean flipped1 = false;

// Second set of position tracking variables
int updatedPos2 = 0;     // keeps track of the latest updated value of the MR sensor reading
int rawPos2 = 0;         // current raw reading from MR sensor
int lastRawPos2 = 0;     // last raw reading from MR sensor
int lastLastRawPos2 = 0; // last last raw reading from MR sensor
int flipNumber2 = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset2 = 0;
int rawDiff2 = 0;
int lastRawDiff2 = 0;
int rawOffset2 = 0;
int lastRawOffset2 = 0;
boolean flipped2 = false;

const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
double OFFSET = 980;
double OFFSET_NEG = 15;

// Kinematics variables

// Hapkit 1
double xh1 = 0;           // Position of the handle [m]
double theta_s1 = 0;      // Angle of the sector pulley in deg
double xh_prev1;          // Distance of the handle at previous time step
double dxh1;              // Velocity of the handle
double dxh_prev1;
double dxh_filt1;         // Filtered velocity of the handle
double dxh_filt_prev1;

// Hapkit 2
double xh2 = 0;           // Position of the handle [m]
double theta_s2 = 0;      // Angle of the sector pulley in deg
double xh_prev2;          // Distance of the handle at previous time step
double dxh2;              // Velocity of the handle
double dxh_prev2;
double dxh_filt2;         // Filtered velocity of the handle
double dxh_filt_prev2;

// Force output variables
// Hapkit 1
double force1 = 0;                 // Force at the handle
double Tp1 = 0;                   // Torque of the motor pulley
double duty1 = 0;                 // Duty cylce (between 0 and 255)
unsigned int output1 = 0;         // Output command to the motor

// Hapkit 2
double force2 = 0;                // Force at the handle
double Tp2 = 0;                   // Torque of the motor pulley
double duty2 = 0;                 // Duty cylce (between 0 and 255)
unsigned int output2 = 0;         // Output command to the motor

 //*************************************************************************
  //*** Section 1. ADD CODE HERE: Parameters for teleop controllers ********  
  //************************************************************************


// --------------------------------------------------------------
// Setup function 
// --------------------------------------------------------------
void setup() 
{
  
  // Set up serial communication
  Serial.begin(250000);
  

 //********************************************************************************
  //*** Section 2. ADD CODE HERE: DUPLICATE SETUP FOR MOTOR 2 AND SENSOR 2 ********  
  //*******************************************************************************
  // NOTE: SEE VARIABLES ALREADY DEFINED FOR YOU UP AT THE TOP
 
  // Set PWM frequency 
  setPwmFrequency(pwmPin1,1); 
 
  
  // Input pins
  pinMode(sensorPosPin1, INPUT); // set MR sensor pin to be an input


  // Output pins
  pinMode(pwmPin1, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin1, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin1, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin1, LOW);  // set direction
  
  // Initialize position valiables
  lastLastRawPos1 = analogRead(sensorPosPin1);
  lastRawPos1 = analogRead(sensorPosPin1);

  
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  //*************************************************************
  //*** Compute position in counts for each device ***  
  //*************************************************************

  // HAPKIT 1
  // Get voltage output by MR sensor
  rawPos1 = analogRead(sensorPosPin1);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff1 = rawPos1 - lastRawPos1;          //difference btwn current raw position and last raw position
  lastRawDiff1 = rawPos1 - lastLastRawPos1;  //difference btwn current raw position and last last raw position
  rawOffset1 = abs(rawDiff1);
  lastRawOffset1 = abs(lastRawDiff1);
  
  // Update position record-keeping vairables
  lastLastRawPos1 = lastRawPos1;
  lastRawPos1 = rawPos1;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset1 > flipThresh) && (!flipped1)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff1 > 0) {        // check to see which direction the drive wheel was turning
      flipNumber1--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber1++;              // ccw rotation
    }
    flipped1 = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped1 = false;
  }
   updatedPos1 = rawPos1 + flipNumber1*OFFSET; // need to update pos based on what most recent offset is    
  //  Serial.print(updatedPos1);
  //  Serial.print(' ');

  // HAPKIT 2 
  // Get voltage output by MR sensor
  rawPos2 = analogRead(sensorPosPin2);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff2 = rawPos2 - lastRawPos2;          //difference btwn current raw position and last raw position
  lastRawDiff2 = rawPos2 - lastLastRawPos2;  //difference btwn current raw position and last last raw position
  rawOffset2 = abs(rawDiff2);
  lastRawOffset2 = abs(lastRawDiff2);
  
  // Update position record-keeping vairables
  lastLastRawPos2 = lastRawPos2;
  lastRawPos2 = rawPos2;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset2 > flipThresh) && (!flipped2)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff2 > 0) {        // check to see which direction the drive wheel was turning
      flipNumber2--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber2++;              // ccw rotation
    }
    flipped2 = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    flipped2 = false;
  }
   updatedPos2 = rawPos2 + flipNumber2*OFFSET; // need to update pos based on what most recent offset is 
  //  Serial.print(updatedPos2);
  //  Serial.println(' ');
   
  //*** Compute position in meters *******************
  // Define kinematic parameters you may need
  double rh1 = 0.075;   //[m] 
  double rh2 = 0.075;   //[m]
  
  // **************************************************************
  // ********* SECTION 3: ADD CODE HERE AS INDICATED **************
  // **************************************************************
  // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  theta_s1 = 0.01326 * updatedPos1 - 7.28; // Your hapkit fit goes here
  theta_s2 = 0.01326 * updatedPos2 - 7.28; // Linked hapkit fit goes here
  
  // Compute the position of the handle based on the angle of the sector pulley
  xh1 = -rh1*(theta_s1 * 3.14159 / 180);
  xh2 = -rh2*(theta_s2 * 3.14159 / 180);

  Serial.print(xh1,4);
  Serial.print(' ');
  Serial.print(xh2,4);
  Serial.println(' ');

  // Calculate the velocity of the handle
  dxh1 = (double)(xh1 - xh_prev1) / 0.001;
  dxh2 = (double)(xh2 - xh_prev2) / 0.001;
    
  // Calculate the filtered velocity of the handle using an IIR filter
  double alpha = 0.2; 
  dxh_filt1 = alpha*dxh1 + (1-alpha)*dxh_prev1;
  dxh_filt2 = alpha*dxh2 + (1-alpha)*dxh_prev2;
    
  // Record the position and velocity
  xh_prev2 = xh2;
  xh_prev1 = xh1;
  
  dxh_prev2 = dxh2;
  dxh_prev1 = dxh1; 

  
  //****************************************************************************
  //*** Section 4: ADD CODE HERE: ASSIGN MOTOR OUTPUT FORCES IN NEWTONS ******** 
  //*** (i.e. WRITE YOUR CONTROL LAWS HERE) ************************************ 
  //****************************************************************************
  // Use variables "force1" and "force2"
  float kp1 = 200;
  float kd1 = 0.2;
  float kp2 = 200;
  float kd2 = 0.2;
  float ms2 = 2;   // motion scaling to hapkit 2
  float fs2 = 1;   // force scaling to hapkit 2

  force1 = kp1*(xh2 - ms2*xh1) + kd1*(dxh2 - dxh1);
  force2 = fs2*(-kp2*(ms2*xh1 - xh2) - kd2*(dxh1 - dxh2));
 
  //**********************************************************************
  //*** Section 5: ADD CODE HERE: ADD FORCE OUTPUT FOR HAPKIT 2 **********
  //**********************************************************************
  
  // HAPKIT 1
  // Compute the require motor pulley torque (Tp) to generate that force
  Tp1 = rp/rs * rh1* force1;
  
  // Determine correct direction for motor torque
  if(force1 > 0) 
  { 
    digitalWrite(dirPin1, LOW);
  } 
  else 
  {
    digitalWrite(dirPin1, HIGH);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty1 = sqrt(abs(Tp1)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty1 > 1) 
  {            
    duty1 = 1;
  } 
  else if (duty1 < 0) 
  { 
    duty1 = 0;
  }  
  output1 = (int)(duty1* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin1,output1);  // output the signal    

  // Duplicate for HAPKIT 2
  // YOU NEED TO ADD THIS
  Tp2 = rp/rs * rh2* force2;
  
  // Determine correct direction for motor torque
  if(force2 > 0) 
  { 
    digitalWrite(dirPin2, LOW);
  } 
  else 
  {
    digitalWrite(dirPin2, HIGH);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty2 = sqrt(abs(Tp2)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty2 > 1) 
  {            
    duty2 = 1;
  } 
  else if (duty2 < 0) 
  { 
    duty2 = 0;
  }  
  output2 = (int)(duty2* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin2,output2);  // output the signal  

  
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
