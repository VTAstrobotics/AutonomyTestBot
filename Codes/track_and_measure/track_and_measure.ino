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
#include <Wire.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

int reading = 0;

Pixy pixy;

Servo myservox;                         // create servo object to control the xtracking servo
                                        
Servo myservoy;                         // create servo object to control the ytracking servo

int posx = 121;                          // variable to store the servo position in the horizontal direction
int posy = 155;                         // variable to store the servo position in the vertical direction
int temp;
int xloc;                               // Variable to store the pixy camera's x-location of signature 1
int yloc;                               // Variable to store the pixy camera's y-location of signature 1
int i;
static int k;
int cswitch;                            // Variable used to switch between cases.  It changes depending on whether the Pixy returns a true value for
                                        //detecting a calibrated signal (1 for "seeing it" and 0 for "not seeing it")
bool blockHist[8] = {0,0,0,0,0,0,0,0};

int DELAY = 30;

void setup()
{
  Wire.begin(); //join i2c bus
  
Serial.begin(9600);
Serial.print("Starting...\n");
pixy.init();
myservox.attach(7);                     // attaches the servo on pin 7 to the servo object
myservox.write(posx);                   // Center Camera in x (depends on how you set the camera up, may need to be calibrated)
myservoy.attach(5);
myservoy.write(posy);                   // Center Camera in x (depends on how you set the camera up, may need to be calibrated)
//delay(1000);
 cswitch = 0;
}
void loop()
{
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
    Serial.print("Distance: ");
    Serial.println(reading); // print the reading
  }
  
                                        // This bit here attempts to determine the x and y positions of signal 1 (the green block right now).
  pixy.getBlocks();
  xloc = (int) pixy.blocks[0].x;        // x and y when gotten from the pixy library are not integers for some reason.  The (int) before it forces it to become an integer.
  yloc = (int) pixy.blocks[0].y;
  delay(DELAY);
if (blocksSeen())               // This part states that if the camera sees signature one, make cswitch equal 
                                        //to 1.  (I can't directly assign the value for some reason but we need a variable for case statements anyways)
{
 cswitch = 1;
 Serial.println("Precase 1");           // I've been using different serial prints to try to track what loop the algorithm was getting stuck in.  
}
else{
  cswitch = 0;                          // If the pixy camera doesn't detect signature 1, make cswitch equal to 0.
  Serial.println("Precase 0");          // I've been using different serial prints to try to track what loop the algorithm was getting stuck in.
}
  
  switch(cswitch)                       // This line begins the case structure, using cswitch as the controlling variable.
  {
    case 0:                             // This is the algorithm that runs when cswitch = 0.  Basic idea is that if signature 1 is not deteched, it 
                                        // will sweep the servo first one way, then the other until a signal is picked up.
Serial.println("Case 0");               // Prints to the serial monitor to let us know what case the algorithm is running.
      for (posx = posx; posx <= 180; posx += 1) { // goes from 0 degrees to 180 degrees
                                                  // in steps of 1 degree
    myservox.write(posx);               // tell servo to go to position in variable 'posx'

     if ( pixy.getBlocks() > 0)         // Using this if statement to break the loop if the camera picks up signature 1.
     {
      cswitch = 1;
      
     break;
     }
    delay(DELAY);                         // waits 150 ms for the servo to reach the position
  }                                         
  for (posx = posx; posx >= 0; posx -= 1) { // goes from 180 degrees to 0 degrees
    myservox.write(posx);               // tell servo to go to position in variable 'pos'
//        Serial.println(cswitch);
     if ( pixy.getBlocks() > 0)         // Using this if statement to break the loop if the camera picks up signature 1.
     {
      cswitch = 1;
      
     break;
     }
    delay(DELAY);                         // waits 150ms for the servo to reach the position
  
}

  
  case 1:                               // This is the algorithm that runs when cswitch = 1.  If a signature is detected, it takes the x and y position of the camera and moves the servos towards it.
                                        // Note that if we move the servos around, any of the signs below are subject to change.  The signs here are using the mount as of 9/20/2016.

Serial.println("Case 1");               // Prints to the serial monitor to let us know what case the algorithm is running.

                  // Here begins the series of loops that center the camera on the signature's center.  The x-center of the pixy camera is an x-value of 160.
  while(xloc>=175)                      // This states that if the signature's center is to the right of the pixy camera, the x-servo will move right until it is within the specified range.
  {
    pixy.getBlocks();                   // Obtains the current data from the pixy camera.
    xloc = (int) pixy.blocks[0].x;      // Obtains the x-location of signature 1 from the pixy camera
    posx -= 1;                          // Moves the servo 1 degree from its previous position
    myservox.write(posx);               // tell servo to go to position in variable 'posx'
    delay(DELAY);

    }
                                        // The next 3 loops apply the same logic as for the commented loop above but for the other three directions.
   
    
    while(xloc<=145)                    // This moves the camera left while the xcenter of the signature is to the left.
  {
    
    pixy.getBlocks();
    xloc = (int) pixy.blocks[0].x;
    posx +=1;
    myservox.write(posx);               // tell servo to go to position in variable 'pos'
    delay(DELAY);                         // waits 15ms for the servo to reach the position
    
    }

  

    
                                        //This portion tracks the y location of the green block, the y center of the pixy camera is at y=100.
  while(yloc>=115)                      // This moves the camera up while the ycenter of the signature is above.
  {

    pixy.getBlocks();
    yloc = (int) pixy.blocks[0].y;   
    posy += 1;
    myservoy.write(posy);              
    delay(DELAY);

    }
    
    while(yloc<=85)                     // This moves the camera down while the ycenter of the signature is below.
  {
    pixy.getBlocks();
    yloc = (int) pixy.blocks[0].y;
    posy -=1;
    myservoy.write(posy);              
    delay(DELAY);                       
    
    }
  }


}
    
bool blocksSeen() {
  for (int i = 0; i < 7; i++) {
    blockHist[i] = blockHist[i+1];
  }
  if (pixy.getBlocks() > 0) {
    blockHist[7] = 1;
    return true;
  } else {
    blockHist[7] = 0;
    bool seen = false;
    for (int i = 0; i < 8; i++) {
      if (blockHist[i] == 1) {
        seen = true;
      }
    }
  return seen;
  }
}

