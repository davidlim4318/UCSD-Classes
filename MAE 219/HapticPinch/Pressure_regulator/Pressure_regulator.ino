/**************************************************************************/
/*! 
    @file     trianglewave.pde
    @author   Adafruit Industries
    @license  BSD (see license.txt)

    This example will generate a triangle wave with the MCP4725 DAC.   

    This is an example sketch for the Adafruit MCP4725 breakout board
    ----> http://www.adafruit.com/products/935
 
    Adafruit invests time and resources providing this open source code, 
    please support Adafruit and open-source hardware by purchasing 
    products from Adafruit!
*/
/**************************************************************************/
#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

void setup(void) {
  Serial.begin(19200);

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x61);
  jam(800);
  
}

int val = 800;

void loop(void) {
  float f = 0.2;
  float t = millis()/1000.;
  jam(900);
  //jam( 950*( 2*ceil(f*t)-ceil(2*f*t) ) );
  //jam(400*cos(2*PI*f*millis()/1000.0)+400);


  // if (Serial.available() > 0) {
  //   char incomingByte = Serial.read();  // Read one byte from serial

  //   // Check if the received byte matches a specific key, e.g., 'a'
  //   if (incomingByte == 'a') {
  //     val = 960;
  //     jam(val);
  //   }
  //   else if (incomingByte == 'x') {
  //     val = 300;    
  //     jam(val);
      
  //   }

  // }
}

void jam(float p) {
    int output = constrain(p,0,1800);
    dac.setVoltage(output, false);
    Serial.print(0);
    Serial.print(",");
    Serial.print(output);
    Serial.print(",");
    Serial.println(1800);
}
