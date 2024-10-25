//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// 04.11.14
// Last updated by David Lim 10.18.24
//--------------------------------------------------------------------------

// Includes
#include <math.h>

// Enviroment switching (uncomment one at a time)
//#define ENABLE_VIRTUAL_WALL
//#define ENABLE_LINEAR_DAMPING
//#define ENABLE_TEXTURE
//#define ENABLE_HARD_SURFACE
#define ENABLE_MASS_SPRING_DAMPER

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
double alpha = 0.2;   // IIR filter parameter
double xhLast;   // xh [m] from previous loop
double vh;   // velocity [m/s] of the handle, unfiltered
double vhFilt;   // vh [m/s], IIR filtered

// A. Virtual wall variables
#ifdef ENABLE_VIRTUAL_WALL
  double xWall = 0.005;   // position [m] of the wall
  double k = 250;   // stiffness [N/m] of the wall
#endif

// B. Linear damping variables
#ifdef ENABLE_LINEAR_DAMPING
  double b = 4;   // damping coefficient [N*s/m]
#endif

// C. Texture variables
#ifdef ENABLE_TEXTURE
  double forceMax = 0.5;   // force amplitude [N] of the bumps
  double wavelength = 0.005;   // wavelength [m] of the bumps
#endif

// D. Hard surface variables
#ifdef ENABLE_HARD_SURFACE
  double xWall = 0.005;   // position [m] of the wall
  double k = 150;   // stiffness [N*s/m] of the wall
  double gamma = 100;   // exponential decay rate [s^-1] of the vibration
  double freq = 30;   // frequency [Hz] of the vibration
  double A = 100;   // force amplitude [N] of the vibration
  bool impact;   // boolean indicating that an impact has happened
  double tImpact;   // time [s] of the impact
  double vhImpact;   // velocity [m/s] of the impact
#endif

// E. Mass-spring-damper variables
#ifdef ENABLE_MASS_SPRING_DAMPER
  double x0 = 0.01;   // equilibrium position [m] of the mass
  double m = 2;   // mass [kg] of the mass
  double b = 1;   // damping coefficient [N*s/m] on the mass
  double k = 300;   // spring constant [N/m] on the mass
  double ku = 1000;   // spring constant [N/m] between the user and the mass
  double xm = x0;   // position [m] of the mass (starts at the equilibrium position)
  double vm;   // velocity [m/s] of the mass
  double am;   // acceleration [m/s^2] of the mass
  double Fu;   // force [N] between the user and the block
  double Fk;   // force [N] of the spring
  double Fb;   // force [N] of the damper
#endif

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor


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
  Serial.print("updatedPos:");
  Serial.print(updatedPos);
  Serial.print(" ");
  // Step B.6: double ts = ?; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  double ts = 0.01326 * updatedPos - 7.28;
  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)
  double xh = rh * PI * ts / 180.;
  // Step B.8: print xh via serial monitor
  Serial.print("xh:");
  Serial.print(xh,4);
  Serial.print(" ");
  
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


  #ifdef ENABLE_VIRTUAL_WALL
    if (xh > xWall) {   // if the handle is inside the wall
      force = k * (xWall - xh);   // compute spring force
    }
    else {   // if the handle is outside the wall
      force = 0;   // force is zero
    }
  #endif


  #ifdef ENABLE_LINEAR_DAMPING
    if (abs(vhFilt) > 0.005) {   // if the filtered velocity magnitude is greater than a threshold
      force = -b * vhFilt;   // compute damping force
    }
    else {   // if the filtered velocity magnitude is less than the threshold
      force = 0;   // force is zero
    }
  #endif


  #ifdef ENABLE_TEXTURE
    force = forceMax*sin(2*PI*xh/wavelength);   // compute force as a sinusoidal function of the handle position
  #endif


  #ifdef ENABLE_HARD_SURFACE
    if (xh > xWall) {   // if the handle is inside the wall
      if (!impact) {   // if an impact was not previously detected
        tImpact = t;   // store the time of the impact
        vhImpact = vhFilt;   // store the velocity of the impact
        impact = true;   // impact has been detected
      }
      // compute force as a spring force plus a decaying sinusoid that starts at the time of the impact
      force = k * (xWall - xh) - A * vhImpact * exp(-gamma * (t - tImpact)) * sin(2 * PI * freq * (t - tImpact));
    }
    else {   // if the handle is outside the wall
      impact = false;   // impact has not been detected
      force = 0;   // force is zero
    }
  #endif


  #ifdef ENABLE_MASS_SPRING_DAMPER
    vm = vm + am * dt;   // compute the current mass velocity
    xm = xm + vm * dt;   // compute the current mass position
    if (xm < xh) {   // if the mass is to the left of the handle (collision detection)
      xm = xh;   // set the mass position equal to the handle position
      vm = 0;   // set the mass velocity equal to zero (inelastic collision)
    }
    if (xm < xh + x0) {   // if the handle is close enough to the mass
      Fu = k * (xh + x0 - xm);   // compute the spring force between the handle and the mass
    }
    else {   // if the handle is not close enough to the mass
      Fu = 0;   // force = 0
    }
    Fk = k * (x0 - xm);   // compute the spring force on the mass
    Fb = -b * vm;   // compute the damping force on the mass
    am = (Fu + Fk + Fb) / m;   // compute the acceleration of the mass
    force = -Fu;   // render the force to the user
    Serial.print("xm:");
    Serial.print(xm,4);
    Serial.print(" ");
  #endif


  Serial.print("force:");
  Serial.print(force);
  Serial.println(" ");
  
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
