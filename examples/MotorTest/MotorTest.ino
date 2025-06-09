// Patched Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor4809.h>


// Declare motors
AF_DCMotor m1(1);          // Motor M1
AF_DCMotor m2(2);          // Motor M2  


void setup() {
  m1.setSpeed(150);       // Set speed from 0-255
  m2.setSpeed(150);       

}

void loop() {
  m1.run(FORWARD);         
  m2.run(FORWARD);         
  delay(1500);
  m1.run(BACKWARD);       
  m2.run(BACKWARD);        
  delay(1500);
  m1.run(RELEASE);         
  m2.run(RELEASE);         
  delay(1000);
}
