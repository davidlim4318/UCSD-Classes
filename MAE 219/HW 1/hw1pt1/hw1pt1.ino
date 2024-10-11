// Includes
#include <math.h>

// Pin declares
int pwmPin = 5;  // PWM output pin for motor 1
int dirPin = 8;  // direction output pin for motor 1

// *************** Parameters for the haptic rendering *******************
// Amplitude control variables
int output = 0;  // output to send to the motor (between 0 and 255) --> should be rounded to an int
int maxOutput = 255;  // output cannot be above 255

float y;  // sinusoidal function value
float A = 150;  // amplitude
float frequency = 20;  // frequency 
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
  analogWrite(pwmPin, 0);  // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction

  delay(3000);  // delay to allow for unplugging while unpowered
  
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
  timeSeconds = (float)currentMillis/64000;  // convert to seconds (note: that there is a conversion needed due to the setPwmFrequency function we use, which changes the timers on the Arduino)
  
  // sine function with amplitude A and frequency in Hz
  y = A*sin(2*PI*frequency*timeSeconds);

  // set the motor direction using the dirPin based on the sign of the output
  if (y > 0) {  // if y is positive
    digitalWrite(dirPin, LOW);  // set direction pin to low
  }
  else {  // if y is 0 or negative
    digitalWrite(dirPin, HIGH);  // set direction pin to high, reversing the direction
  }

  // before sending to board, convert output to positive integer and limit output if greater than maximum allowed
  output = abs(y);
  output = min(output,maxOutput);

  // send output to the motor using the pwmPin
  analogWrite(pwmPin, output);

  // print output to check
  Serial.print("Time:");
  Serial.print(timeSeconds);
  Serial.print(" ");
  Serial.print("y:");
  Serial.print(y);
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
