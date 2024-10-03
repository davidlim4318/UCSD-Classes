// Includes
#include <math.h>

// Pin declares
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1

// *************** Parameters for the haptic rendering *******************
// Amplitude control variables
float output = 0.0;     // output to send to the motor (between 0 and 255) --> should be rounded to an int
float maxOutput = 255.0; // output cannot be above 255

float A = 200;        // try range 
float frequency = 25; // try range 
float timeSeconds;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(115200);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  //*************************************************************
  //******** Sine wave input ************
  //*************************************************************

  unsigned long currentMillis = millis();
  timeSeconds = (float)currentMillis/64000; // convert to seconds (note: that there is a conversion needed due to the setPwmFrequency function we use, which changes the timers on the Arduino)
  
  // sine function with amplitude A and frequency in Hz
  output = A*sin(2*PI*frequency*timeSeconds);

  // set the motor direction using the dirPin based on the sign of the output
  if (output > 0) {
    //digitalWrite(dirPin, LOW);  // set positive direction
  }
  else {
    //digitalWrite(dirPin, HIGH);  // set negative direction
  }

  // Add in checks to make sure that the command sent to the motor is not greater than the maximum possible output
  // (or less than the negative of the maximum)
  output = constrain(output,-maxOutput,maxOutput);

  // send output to the motor using the pwmPin --> don't forget that the command needs to be a positive integer!
  output = abs((int)output);
  // analogWrite(pwmPin, output);

  // print output to check
  Serial.print("Time:");
  Serial.print(timeSeconds);
  Serial.print(" ");
  Serial.print("Output:");
  Serial.println(output);
  
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
