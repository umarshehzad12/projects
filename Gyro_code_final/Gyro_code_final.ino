
/*The open source pothole detection kit has been developed at Make-i-stan in collaboration with ICFJ Knight Fellowships*/
/* We at Makeistan have made use of the example code provided by Jeff Rowberg's library
 i.e. MPU6050, hence the copyright notice below
 */
 
/*I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include <LiquidCrystal.h> // LCD Library
#include <I2Cdev.h> // I2C Library
#include <SoftwareSerial.h> // Software Serial Library for using pin 10 and 11 as serial pins
#include <TinyGPS.h>// GPS library for ublox
using namespace std; 
#include "MPU6050_6Axis_MotionApps20.h" // MPU6050 is the gyro sensor that we're using
#include "Wire.h" // Wire library is used for communicating the two arduino boards
#include <SPI.h> //Used by SD card communication
#include <SD.h> //SD card library

MPU6050 mpu;
File myFile; // file initialized to be stored in SD card for storing the data
int getgps=2; //This variable is used for controlling when to read the GPS coordinates 
int getgyro=2; //This variable is used for controlling when to get the data from Gyro
int iter=0; // to count the number of iterations before reset is required, for debugging purposes
unsigned long telapsed=0; // to keep track of the time being elapsed after every loop
float pitch=0, roll=0; // pitch and roll is stored into these variables
int bumpval=0; // pitch and roll are later combined into this variable
LiquidCrystal lcd(12, 11, 6, 5, 4, 3); // initializing the 16x2 LCD

#define OUTPUT_READABLE_YAWPITCHROLL //Yaw pitch and roll are the three variables that are calculated by gyro using the i2c library

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards 
//#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
//bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
SoftwareSerial mySerial(10, 11); // RX, TX
TinyGPS gps; // an instance of the GPS is created here

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//int smallBups = 0, LargeBups = 0;
class valuesPackage // a special class is created to keep two values of pitch as well as roll each time the code executes and the difference is later calculated to detect a road bump
{
  public:
    float p = 0, r =0; //pitch and roll's intermediate values before they are stored into an integer created above
};

int index = 0; // index is used for controlling the stabilizing time of the gyro
valuesPackage valuesArr[2]; //two values are kept in each cycle (of pitch and roll) which are later subtracted from each other to detect a bump

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() { 
    mpuInterrupt = true;
}
  


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

Wire.begin();
Serial.begin(9600);
mySerial.begin(9600);
lcd.begin(16, 2);


//Initialize SD card code

 while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin(53)) { //pin 53 is used for SD card, if it is not ready, an error is generated and is returned
    Serial.println("SD card initialization failed!");
    lcd.print("SD card initialization failed!");
    delay(2000);
  lcd.clear();
   return;
  }
  Serial.println("SD card initialization done."); //if the port is available
//Initialize SD card code ends here

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
       // Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT); // interrupt is not being used, so this might be removed in subsequent versions

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed")); // MPU6050 connections verified

    // wait for ready



    
    Serial.println(F("\nSend any character to begin DMP programming and demo: ")); // This doesn't require a character anymore, maybe removed in subsequent release
    while (Serial.available() && Serial.read()); // empty buffer
    lcd.print("waiting in 40");
    //while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again




    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        
        
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        //mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt...")); //may get removed in the next release
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

 myFile = SD.open("gyro_gps.txt", FILE_WRITE); //File is created and/or opened by Arduino
 //myFile.println();
 //myFile.println("took approximately 8 seconds to reset");
 Wire.requestFrom(8,15);    // request 15 bytes from slave device #8
 delay(50); // Giving time to GPS to copy all the coordinates into buffer
 if(myFile) //Checks if the file in SD card is opened properly
 {
 if(getgps==1) //only if the GPS is allowed to write on SD card, following code exectutes
 {
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    myFile.print(c);
    Serial.print(c); // print the character
    lcd.print(c);
  }
  myFile.print(", ");//When !Wire.available(), do this
  myFile.close();
 getgyro=1; //Give permission to subsequent gyro code to run
 Serial.println();
 telapsed=millis();
 Serial.print("Iteration number: ");
 iter=iter+1;
 Serial.println(iter);
 Serial.print("Time Elapsed");
 Serial.println(telapsed);
}
else;
 }
 else
 {
 lcd.print("Check SD card");
 Serial.print("Stuck in line 241");
 delay(300);
 lcd.clear();
 pinMode(7,OUTPUT); //Resets Arduino mega, pin 7 is connected to RESET pin


 }
 getgps=0;

 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 3072) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
     //    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {

       
        // wait for correct available data length, should be a VERY short wait
        
        
        
        
        //while (fifoCount < packetSize) //was turned off by awais
          
          
          
          
          fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

valuesPackage obj;

obj.p = ypr[1] * 180/M_PI; //formula to calculate pitch
obj.r = ypr[2] * 180/M_PI; //formula to calculate roll
            valuesArr[index%2] = obj;
index++;
             
  if(index > 50) //First 50 values are disregarded since gyro needs time to stabilize
  {
    int lastIndex =  (index+1)%2; //0 if index==1 otherwise 1
    pitch = valuesArr[index%2].p - valuesArr[lastIndex].p; //difference in values intensity of the bump
    roll = valuesArr[index%2].r - valuesArr[lastIndex].r;
    if(pitch<0)
    pitch= (-1*pitch); 
    else;
    if(roll<0)
    roll= (-1*roll);
    else;
    pitch=pitch*4; //sensitivity quadrupled
    roll=roll*4;
    bumpval= round(pitch+roll); //pitch and roll combined to make an integer value
    //pitch = abs(pitch);
    //roll = abs(roll);
  
  
     Serial.print("-----------------------");
        Serial.print(pitch);Serial.print(" ");
      Serial.println(roll);Serial.println();
      //Writing Data to SD card now
      // if the file opened okay, write to it
      if(getgyro==1){ //if gyro is allowed to write on SD card, execute following
      if (myFile) {
    Serial.print("Writing gyro_gps.txt...");
    //lcd.print("Writing gyro.txt...");
    delay(30);
    lcd.setCursor(0,1);
    //myFile.print(pitch);
    //lcd.print(pitch);
    //myFile.print(", ");
    //lcd.print(", ");
    //myFile.print(roll);
    //lcd.print(roll);
    //myFile.print(", ");
    //lcd.print(", ");
    lcd.print(bumpval);
    myFile.print(bumpval);
    myFile.print(", ");
    lcd.print(", ");
    myFile.println();
    // close the file:
    myFile.close();
    Serial.println("done.");   
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening gyro_gps.txt");
    lcd.setCursor(0,1);
    lcd.println("error opening gyro.txt");
    delay(500);
 
  }
      }
      getgps=1;
  }
  else
  lcd.print("Gyro stabilizing.."); //if index<50, allow gyro to stabilize
delay(100);
lcd.clear();
        #endif

    }

}

