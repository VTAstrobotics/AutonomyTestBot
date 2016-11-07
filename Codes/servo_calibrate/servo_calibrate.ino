/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#include <Pixy.h>
#include <SPI.h>

Pixy pixy;
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
Servo myservo2;
int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(9600);
Serial.print("Starting...\n");
  pixy.init();
  myservo.attach(5);  // attaches the servo on pin 9 to the servo object
   myservo.write(155);
   myservo2.attach(7);
   myservo2.write(121);
   Serial.println("test");
  Serial.println(pixy.getBlocks());
}

void loop() {
   
   Serial.println(pixy.getBlocks());
   delay(50);
//  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'//   delay(15);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
}

