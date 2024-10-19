//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// 04.11.14
// Last updated by Tania Morimoto 2.01.23
//--------------------------------------------------------------------------

// Includes
#include <math.h>

// Enviroment switching
//#define ENABLE_VIRTUAL_WALL
//#define ENABLE_LINEAR_DAMPING
//#define ENABLE_TEXTURE
//#define ENABLE_HARD_SURFACE
//#define ENABLE_MASS_SPRING_DAMPER

// Velocity estimate parameters
double alpha = 0.2;
unsigned long timeLast;
double xhLast;
double vh;
double vhFilt;

// A. Virtual wall parameters
#ifdef ENABLE_VIRTUAL_WALL
  double xWall = 0.005;
  double k = 250;
#endif

// B. Linear damping parameters
#ifdef ENABLE_LINEAR_DAMPING
  double b = 4;
#endif

// C. Texture parameters
#ifdef ENABLE_TEXTURE
  double forceMax = 0.5;
  double wavelength = 0.005;
#endif

// D. Hard surface parameters
#ifdef ENABLE_HARD_SURFACE
  double xWall = 0.005;
  double k = 150;
  double gamma = 40;
  double freq = 30;
  double A = 20;
  bool impact;
  unsigned long timeImpact;
  double vhImpact;
#endif

// E. Mass-spring-damper parameters
#ifdef ENABLE_MASS_SPRING_DAMPER
  double x0 = 0.01;
  double m = 2;
  double b = 1;
  double k = 300;
  double ku = 1000;
  double xm = x0;
  double vm;
  double am;
  double Fu;
  double Fk;
  double Fb;
  unsigned long tLast;
  unsigned long dt;
#endif

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

  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  double rh = 0.070;  //[m]
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
 
  // ADD YOUR CODE HERE
  // Define kinematic parameters you may need
  double rp = 0.005;  //[m]
  double rs = 0.074;  //[m]
  // Step C.1: force = ?; // You will generate a force by simply assigning this to a constant number (in Newtons)
    //force = 0.12;
  // Step C.2: Tp = ?;    // Compute the require motor pulley torque (Tp) to generate that force
    //Tp = rh * rp * force / rs;
    //Serial.print("Duty cycle:");
    //Serial.print(sqrt(abs(Tp)/0.0183));
    //Serial.println(" ");

  //*************************************************************
  //******************* Rendering Algorithms ********************
  //*************************************************************
  // ADD YOUR CODE HERE
  // NOTE: good to use #ifdef statements for the various environments

  // Velocity estimate and filtering
  vh = (xh - xhLast) / (getTimeMillis() - timeLast) * 1000;
  vhFilt = alpha*vh + (1 - alpha)*vhFilt;
  xhLast = xh;
  timeLast = getTimeMillis();
  Serial.print("time:");
  Serial.print(timeLast);
  Serial.println(" ");

  #ifdef ENABLE_VIRTUAL_WALL
    if (xh > xWall){
      force = k * (xWall - xh);
    }
    else {
      force = 0;
    }
    Serial.print("force:");
    Serial.print(force);
    Serial.println(" ");
  #endif

  #ifdef ENABLE_LINEAR_DAMPING
    if (abs(vhFilt) > 0.005){
      force = -b * vhFilt;
    }
    else {
      force = 0;
    }
    Serial.print("vh:");
    Serial.print(vh,4);
    Serial.print(" ");
    Serial.print("vhFilt:");
    Serial.print(vhFilt,4);
    Serial.print(" ");
    Serial.print("force:");
    Serial.print(force);
    Serial.println(" ");
  #endif

  #ifdef ENABLE_TEXTURE
    force = forceMax*sin(2*PI*xh/wavelength);
    Serial.print("force:");
    Serial.print(force);
    Serial.println(" ");
  #endif

  #ifdef ENABLE_HARD_SURFACE
    if (xh > xWall){
      if (!impact) {
        timeImpact = getTimeMillis();
        vhImpact = vhFilt;
        impact = true;
      }
      double time = getTimeMillis();
      force = k * (xWall - xh) - A * vhImpact * exp(-gamma * (time - timeImpact) / 1000.) * sin(2 * PI * freq * (time - timeImpact) / 1000.);
    }
    else {
      impact = false;
      force = 0;
    }
    Serial.print("force:");
    Serial.print(force);
    Serial.println(" ");
  #endif

  #ifdef ENABLE_MASS_SPRING_DAMPER
    dt = getTimeMillis() - tLast;
    tLast = getTimeMillis();
    xm = xm + vm * dt / 1000;
    vm = vm + am * dt / 1000;
    if (xm < xh) {
      xm = xh;
      vm = 0;
    }
    if (xm < xh + x0) {
      Fu = k * (xh + x0 - xm);
    }
    else {
      Fu = 0;
    }
    Fk = k * (x0 - xm);
    Fb = -b * vm;
    am = (Fu + Fk + Fb) / m;

    Serial.print("force:");
    Serial.print(-Fu);
    Serial.print(" ");
    Serial.print("xm:");
    Serial.print(xm,4);
    Serial.println(" ");

  #endif

  Tp = rh * rp * force / rs;
 
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
// Function to rescale millis()
// --------------------------------------------------------------
unsigned long getTimeMillis() {
  return millis()/64;
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
