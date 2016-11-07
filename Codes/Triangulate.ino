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
   sweep_if_lost v1.1

   Ian Bean/Turner Bowling
   bturn95@vt.edu

*/
#include <SPI.h>
#include <Pixy.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Pixy pixy;

Servo myservox;                         // create servo object to control the xtracking servo

Servo myservoy;                         // create servo object to control the ytracking servo

const int DELAY = 5;                   // Universal delay, could disappear later for increased operation speed
const int SERVODELAY = 20;               // Delay specifically for servo movement, may be depreciated
const int SWEEPDELAY = 30;
const int XUPPERBOUND = 168;            // Set of four axis bounds for keeping camera centered on target board
const int XLOWERBOUND = 152;
const int YUPPERBOUND = 108;
const int YLOWERBOUND = 91;
const int BLOCKCOUNT = 2;
int xtarg = 180;                        // Used as a goal for our sweep state to get to. Switches between 180 and 0
int xadj = 1;                           // Used to guide our sweep when in SWEEPING state
int posx = 121;                         // variable to store the servo position in the horizontal direction
int posy = 155;                         // variable to store the servo position in the vertical direction
int xloc = 0;                               // Variable to store the pixy camera's x-location of signature 1
int yloc = 0;                               // Variable to store the pixy camera's y-location of signature 1
int sigsearch = 1;                           // Signature to search for
int blocktrack1;                        // Tracks which block is signature 1
double stdev = 20;                        // Maximum Deviation of a Distance unit from average value before it's kicked out of the vector
bool blockHist[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Array of bools used for debouncing vision of marker
int distset = 10;                       // Number of distance values to take before calculating a mean
double dist1[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Array of values to be averaged for distance set 1
double dist2[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Array of values to be averaged for distance set 2
double distmean = 0;                     // Value of mean calculated distance
double dist1store = 0;
double dist2store = 0;
double angle1 = 0;
double angle2 = 0;
double temp = 0;
double Origin = 0.0;

//Declaration of all of our statemachine states and the variable to track it all 'state'
enum MachineState {
  SWEEPING, TRACKING, LOCKED
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
  delay(5000);
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

Serial.println("Initialized");

  bno.setExtCrystalUse(true);
}

void loop() {
  pixy.getBlocks();
  for (int i = 0; i < BLOCKCOUNT; i++) {
    if (pixy.blocks[i].signature == sigsearch) {
      blocktrack1 = i;
    }
  }
  xloc = (int) pixy.blocks[blocktrack1].x;          //Typecast our pixy results into integers.
  yloc = (int) pixy.blocks[blocktrack1].y;
  delay(DELAY);
    if (blocksSeen() && xloc > XLOWERBOUND && xloc < XUPPERBOUND && yloc > YLOWERBOUND && yloc < YUPPERBOUND ){
      state = LOCKED;
    }
  else if (blocksSeen()) {
    state = TRACKING;
//    Serial.println("Precase 1");
  }
  else {
    state = SWEEPING;
//    Serial.println("Precase 0");
  }
  switch (state) {
    case SWEEPING:
      Serial.println("SWEEPING");
      if (posx != xtarg) {
        posx += xadj;
        posy = 120 + 25 * xadj * sin((posx * 2 * 6.283) / 180); // posx 8 2pi / 180 to get one period per sweep. xadj inverts func on return
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
        delay(SWEEPDELAY);
      } else {
        xadj = -xadj;
        xtarg = 180 - xtarg;
      }
      break;
    case TRACKING:
      Serial.println("TRACKING");
      pixy.getBlocks();                   // Obtains the current data from the pixy camera.
      xloc = (int) pixy.blocks[blocktrack1].x;      // Obtains the x-location of signature 1 from the pixy camera
      yloc = (int) pixy.blocks[blocktrack1].y;      // Obtains the y-location of signature 1 from the pixy camera
      if (xloc >= XUPPERBOUND ) {
        posx -= 1;
      } else if (xloc <= XLOWERBOUND ) {
        posx += 1;
      }

      if (yloc >= YUPPERBOUND ) {
        posy += 1;
      } else if (yloc <= YLOWERBOUND ) {
        posy -= 1;
      }
      myservox.write(posx);
      myservoy.write(posy);
      delay(SERVODELAY);
      //      Serial.println(getDistance());
      //      for (int i = 0; i < distset; i++){
      //        dist1mean = getDistance() + dist1mean;
      //      }
      //      dist1mean = dist1mean/distset;
      //      Serial.print("Mean Distance of Signature 1 = ");
      //      Serial.println(dist1mean);
      //if (xloc > XLOWERBOUND && xloc < XUPPERBOUND && yloc > YLOWERBOUND && yloc < YUPPERBOUND ){
      //if (xloc > XLOWERBOUND && xloc < XUPPERBOUND){
      //  Serial.println("halfway there");
      //  delay(10);
      //  if (yloc > YLOWERBOUND && yloc < YUPPERBOUND){
      //    Serial.println("There");
      //    state = LOCKED;
      //    Serial.println(state);
      //    delay(10);
      //    break;
      //  }
      //}
      break;
    case LOCKED:
      Serial.println("LOCKED");
      for (int i = 0; i < distset; i++) {
        temp = getDistance();
        if (temp < 0) {
          i = i - 1;
        }
        else {
          distmean = temp + distmean;
        }
        delay(50);
      }
      distmean = distmean / distset;
      Serial.print("Mean Distance of Signature ");
      Serial.print(sigsearch);
      Serial.print(" = ");
      Serial.print(distmean);
      Serial.print(",    At Angle of ");
//      Serial.println(getAngle()); 
      sensors_event_t event;
      bno.getEvent(&event);
      Serial.println(event.orientation.x);
      delay(BNO055_SAMPLERATE_DELAY_MS);
      
      // Here, since we switch over to look for the other signature
      if (sigsearch == 1){
        dist1store = distmean;
        angle1 = event.orientation.x;
        sigsearch = 2;
      }
      else{
        dist2store = distmean;
        angle2 = event.orientation.x;
        sigsearch = 1;
      }
      distmean = 0;
      if ((dist1store != 0) &&(angle1 != 0)&&(dist1store != 0) && (angle2 != 0)){
      triangulate();
      }
      break;
  }
}


/*
   Returns boolean true if blocks are in view, 0 if they haven't been in view for 8 cycles
*/
//bool blocksSeen() {
//  //Shuffle all values to the left
//  for (int i = 0; i < 7; i++) {
//    blockHist[i] = blockHist[i+1];
//  }
//  //Get most recent value
//  if (pixy.getBlocks() > 0) {
////    Serial.println(pixy.getBlocks());
//    blockHist[7] = 1;
//    return true;  //we can cut out here because it's seen
//  } else {        //If not seen now, check if we've seen it recently
//    blockHist[7] = 0;
//    for (int i = 0; i < 8; i++) {
//      if (blockHist[i] == 1) {
//        return true;  //If we've seen it recently, cut out and return true
//      }
//    }
//    return false; //If we've not seen it for a while, return false
//  }
//}
bool blocksSeen() {
  //Shuffle all values to the left
  for (int i = 0; i < 7; i++) {
    blockHist[i] = blockHist[i + 1];
  }

  //Get most recent value
  if (seesig()) {
    //    Serial.println(pixy.getBlocks());
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
bool seesig() {
  //  return false;
  if (pixy.getBlocks() == 0) {
    return false;
  }
  pixy.getBlocks();

  for (int i = 0; i < BLOCKCOUNT; i++) {
    if (pixy.blocks[i].signature == sigsearch) {
      blocktrack1 = i;
      return true;
    }
  }
  return false;
}
/*
   Returns the distance read by the lidar
*/

int getDistance() {
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

  if (2 <= Wire.available()) // if two bytes were received
  {
    reading = Wire.read(); // receive high byte (overwrites previous reading)
    reading = reading << 8; // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    return (reading);
  }
}

double triangulate() {
  double dist3,theta1,theta2,x,y;
//  double theta,atheta,A,B,dist,x,y;
//  double Theta, ATheta, A, B, dist, x, y;
//
//    Theta = (PI/180)*abs(P1Theta-P2Theta);
//    
//    if (dist1store > dist2store) {
//      A = dist2store;
//      B = dist1store;
//      ATheta = angle2;
//    }
//    else{
//      A = dist1store;
//      B = dist2store;
//      ATheta = angle1;
//    }
    
//    Origin = acos((B-(A*cos(Theta)))/sqrt((square(B)+square(A)-(2*B*A*cos(Theta)))));
//    Origin = 90-(180/PI*(Origin+Theta));
//    Origin = Origin + ATheta;
//    dist = 
      theta1 = (PI/180)*abs(angle1-angle2);
      dist3 = sqrt(dist1store*dist1store + dist2store*dist2store - 2*dist1store*dist2store*cos(theta1));
      theta2 = acos(-(dist1store*dist1store-dist2store*dist2store - dist3*dist3)/(2*dist2store*dist3));
      x = dist1store*cos(theta2);
      y = dist1store*sin(theta2);
      Serial.print("x = ");
      Serial.print(x);
      Serial.print("  and y = ");
      Serial.print(y);
      delay(4000);


}
//double getAngle() {
//  sensors_event_t event;
//  bno.getEvent(&event);
//  return (event.orientation.x);
//  delay(BNO055_SAMPLERATE_DELAY_MS);
//  
//}






















//int getDistance(){
//  int reading = 0;
//  for (int i=0; i < distset; i++)
//  {
//  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
//  Wire.write((int)RegisterMeasure); // sets register pointer to  (0x00)
//  Wire.write((int)MeasureValue); // sets register pointer to  (0x00)
//  Wire.endTransmission(); // stop transmitting
//
//  delay(20); // Wait 20ms for transmit
//
//  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
//  Wire.write((int)RegisterHighLowB); // sets register pointer to (0x8f)
//  Wire.endTransmission(); // stop transmitting
//
//  delay(20); // Wait 20ms for transmit
//
//  Wire.requestFrom((int)LIDARLite_ADDRESS, 2); // request 2 bytes from LIDAR-Lite
//
//  if(2 <= Wire.available()) // if two bytes were received
//  {
//    reading = Wire.read(); // receive high byte (overwrites previous reading)
//    reading = reading << 8; // shift high byte to be high 8 bits
//    reading |= Wire.read(); // receive low byte as lower 8 bits
//    return(reading);
//    dist1[i] = reading;
//  }
//  }
//  for (int i=0; i<distset; i++)
//  {
//    dist1mean = dist1mean+dist1[i];
//  }
//  dist1mean = dist1mean/distset;
//  }


