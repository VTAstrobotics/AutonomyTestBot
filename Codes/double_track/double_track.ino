//
// begin license header
//
// This file is part of Pixy CMUcam5 or “Pixy” for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
/*
06.04.2014 v0.1.3 John Leimon
+ Now using pixy.init() to initialize Pixy in setup().
*/
#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>

Pixy pixy;

Servo myservox;  // create servo object to control a servo
// twelve servo objects can be created on most boards
Servo myservoy; 
int posx = 121;    // variable to store the servo position
int posy = 155;   
int temp;
int xloc;
int yloc;
int i;
void setup()
{
Serial.begin(9600);
Serial.print("Starting...\n");
pixy.init();
myservox.attach(7);  // attaches the servo on pin 9 to the servo object
myservox.write(posx); // Center Camera
myservoy.attach(5);
myservoy.write(posy);
//delay(1000);
}
void loop()
{
//This portion tracks the x location of the green block
//
//
//
  pixy.getBlocks();
  xloc = (int) pixy.blocks[0].x;
  yloc = (int) pixy.blocks[0].y;
  //Serial.println(xloc);
  while(xloc>=175)
  {
    // in steps of 1 degree
    pixy.getBlocks();
    xloc = (int) pixy.blocks[0].x;
//    Serial.println(xloc);
//    Serial.println(pixy.blocks[0].signature);
    Serial.println(pixy.getBlocks());
    posx -= 1;
    myservox.write(posx);              // tell servo to go to position in variable 'pos'
    delay(30);

    }
    
    while(xloc<=145)
  {
    // in steps of 1 degree
    pixy.getBlocks();
    xloc = (int) pixy.blocks[0].x;
//    Serial.println(xloc);
//Serial.println(pixy.blocks[0].signature);
    Serial.println(pixy.getBlocks());
    posx +=1;
    myservox.write(posx);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
    
    }



    
//This portion tracks the y location of the green block
//
//
//
  while(yloc>=115)
  {
    // in steps of 1 degree
    pixy.getBlocks();
    yloc = (int) pixy.blocks[0].y;
   // Serial.println(yloc);
//    Serial.println(pixy.blocks[0].signature);
    Serial.println(pixy.getBlocks());
//    Serial.println("Loop 1");
    posy += 1;
    myservoy.write(posy);              // tell servo to go to position in variable 'pos'
    delay(30);

    }
    
    while(yloc<=85)
  {
    // in steps of 1 degree
    pixy.getBlocks();
    yloc = (int) pixy.blocks[0].y;
    //Serial.println(yloc);
//    Serial.println(pixy.blocks[0].signature);
    Serial.println(pixy.getBlocks());
//    Serial.println("Loop 2");
    posy -=1;
    myservoy.write(posy);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 150ms for the servo to reach the position
    
    }
}
