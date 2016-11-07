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

/*
 * sweep_if_lost v1.1
 * 
 * Ian Bean/Turner Bowling
 * bturn95@vt.edu
 * 
 */
#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>
#include <Wire.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

Pixy pixy;

Servo myservox;                         // create servo object to control the xtracking servo
                                        
Servo myservoy;                         // create servo object to control the ytracking servo

const int DELAY = 0;                   // Universal delay, could disappear later for increased operation speed
const int SERVODELAY = 0;               // Delay specifically for servo movement, may be depreciated
const int XUPPERBOUND = 175;            // Set of four axis bounds for keeping camera centered on target board
const int XLOWERBOUND = 145;
const int YUPPERBOUND = 115;
const int YLOWERBOUND = 85;
int xtarg = 180;                        // Used as a goal for our sweep state to get to. Switches between 180 and 0
int xadj = 1;                           // Used to guide our sweep when in SWEEPING state
int posx = 121;                         // variable to store the servo position in the horizontal direction
int posy = 155;                         // variable to store the servo position in the vertical direction
int temp = 0;
int xloc = 0;                               // Variable to store the pixy camera's x-location of signature 1
int yloc = 0;                               // Variable to store the pixy camera's y-location of signature 1
bool blockHist[8] = {0,0,0,0,0,0,0,0};  // Array of bools used for debouncing vision of marker

//Declaration of all of our statemachine states and the variable to track it all 'state'
enum MachineState{
  SWEEPING, TRACKING
} state;

void setup() {
  Wire.begin(); // join i2c bus
  Serial.begin(9600);
  Serial.print("Starting...\n");
  pixy.init();
  myservox.attach(7);                     // attaches the servo on pin 7 to the servo object
  myservox.write(posx);                   // Center Camera in x (depends on how you set the camera up, may need to be calibrated)
  myservoy.attach(5);
  myservoy.write(posy);                   // Center Camera in x (depends on how you set the camera up, may need to be calibrated)
  //delay(1000);
 state = SWEEPING;
}

void loop() {
  pixy.getBlocks();
  xloc = (int) pixy.blocks[0].x;          //Typecast our pixy results into integers.
  yloc = (int) pixy.blocks[0].y;
  delay(DELAY);
  if (blocksSeen()) {
   state = TRACKING;
   Serial.println("Precase 1");  
  } else {
    state = SWEEPING;
    Serial.println("Precase 0");
  }
  switch(state) {
    case SWEEPING:
      Serial.println("SWEEPING");
      if(posx != xtarg) {
        posx += xadj;
        posy = 155 + 25 * xadj * sin((posx * 9.24278) / 180);  // posx 8 2pi / 180 to get one period per sweep. xadj inverts func on return
        if (posx <  0) {
          posx = 0;
        } else if (posx > 180) {
          posx = 180;
        }
        myservoy.write(posy);
        myservox.write(posx);
        if (blocksSeen()) {
          state = TRACKING;
          break;
        }
        delay(SERVODELAY);
      } else {
        xadj = -xadj;
        xtarg = 180 - xtarg;
      }
      break;
    case TRACKING:
      Serial.println("TRACKING");
      pixy.getBlocks();                   // Obtains the current data from the pixy camera.
      xloc = (int) pixy.blocks[0].x;      // Obtains the x-location of signature 1 from the pixy camera  
      yloc = (int) pixy.blocks[0].y;      // Obtains the y-location of signature 1 from the pixy camera
      if (xloc>= XUPPERBOUND ) {         
        posx -= 1;
      } else if (xloc <= XLOWERBOUND ) {
        posx +=1;                    
      }   
      
      if(yloc>= YUPPERBOUND ) {
        posy += 1;
      } else if (yloc<= YLOWERBOUND ) {
        posy -=1;     
      }
      if(posy >180) {
        posy = 180;
      }
      if(posx > 180) {
        posx = 180;
      }
      if(posx < 0) {
        posx = 0;
      }
      if(posy < 0) {
        posy = 0;
      }
      myservox.write(posx); 
      myservoy.write(posy);        
      delay(SERVODELAY); 
      Serial.println(getDistance());
      break;
  }
}

/*
 * Returns boolean true if blocks are in view, 0 if they haven't been in view for 8 cycles
 */
bool blocksSeen() {
  //Shuffle all values to the left
  for (int i = 0; i < 7; i++) {
    blockHist[i] = blockHist[i+1];
  }
  //Get most recent value
  if (pixy.getBlocks() > 0) {
    blockHist[7] = 1;
    return true;  //we can cut out here because it's seen
  } else {        //If not seen now, check if we've seen it recently
    blockHist[7] = 0;
    for (int i = 0; i < 8; i++) {
      if (blockHist[i] == 1) {
        return true;  //If we've seen it recently, cut out and return true
      }
    }
    return false; //If we've not seen it for a while, return false
  }
}

/*
 * Returns the distance read by the lidar
 */
int getDistance(){
  int reading = 0;
  
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)  
  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)  
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
  Wire.endTransmission(); // stop transmitting

  delay(20); // Wait 20ms for transmit

  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite

  if(2 <= Wire.available()) // if two bytes were received
  {
    reading = Wire.read(); // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    return(reading);
  }
}

