// A sketch to sense the orientation of a rocket during ascent and control two SSRs placed
// between an altimeter or timer and an motor ignitor and a separation charge.  If the rocket's
// ascent deviates a set number of degrees or more from vertical, the SSRs are latched open,
// disallowing ignition of the motor and the separation charge.
//
// This sketch uses the I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// using DMP (MotionApps v2.0) written 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
// This sketch has had much of the original Rowberg code removed.
// Adapted for rocketry by Ed Chess, 3/6/2016
//
// Updated 2/6/2017 for inclusion of a 3-position DIP switch, to allow manual time-of-use selection
// of the critical angle of ascent that triggers the SSRs to open.  The switches set the value of
// variable tiltDegrees according to this schedule:
//
//_______________________________ 
// |  tiltDegrees    d5  d4  d3  |
// |       5         OFF OFF OFF |
// |       10        OFF OFF ON  |
// |       15        OFF ON  OFF |
// |       20        OFF ON  ON  |
// |       25        ON  OFF OFF }
// |       30        ON  OFF ON  |
// |       35        ON  ON  OFF |
// |       40        ON  ON  ON  |
// _______________________________
// 
// Updated 2/20/2017 for inclusion of error checking on acceleration detection to initiate flight mode.
//
// Updated 3/11/2017 for inclusion of a piezoelectric beeper on pin 9 and audible output of tiltDegrees.
//
// Updated 3/15/2017 for removal of piezoelectric beeper on pin 9 and visual output of tiltDegree on LED/
//
// Updated 3/19/2017 for auto axis-orientation.
/* ============================================
I2Cdev device library code is placed under the MIT license
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

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.  Note that some MPU-6050 braekout boards run on 5V input.
   SDA connects to Arduino A4; SCL connects to Arduino A5.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */

// You will get the yaw/pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)

#define LED_PIN 13 // Shows microprocessor activity
#define LED_PYRO_PIN 6 // Shows PYRO_PIN1 and PYRO_PIN2 status
#define PYRO_PIN1 7  // Pyro Channel 1 to SSR 1; HIGH if OK to stage
#define PYRO_PIN2 8  // Pyro Channel 2 to SSR 2; HIGH if OK to stage
#define SW3 3        // DIP switch position 3, lowest bit
#define SW2 4        // DIP switch position 2, middle bit
#define SW1 5        // DIP switch position 1, highest bit
//#define beepPin 9    // Piezo beeper

// main program control variables
bool blinkState = LOW;
bool noStage = HIGH;  // latch to stop staging if tilt gets too large
byte tiltDegrees = 0;  // enter number of degrees tilt that will be allowed--20 or less is safest
bool latch = LOW;     // variable to keep track of Z-acceleration;  gets set HIGH upon launch detect
byte count = 0;       // variable to keep track of number of times in a row pitch or roll > tiltDegrees
bool orientation = LOW; // variable to determine board orientation; LOW if Z-axis vertical, HIGH if Y-axis vertical
byte accelCount = 0;    // variable to keep track of number of times in a row accel > 2G
bool orientFlag = LOW;  // flag to set orientation

// MPU control/status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars for ypr output from MPU
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    delay(100);

    // wait for ready;  comment next four lines out if you don't want to wait for serial response
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setFullScaleAccelRange(1);  // Set full range of accelermeters to 1, or 4 g (4096 counts/g)

    // supply your own gyro offsets here, scaled for min sensitivity
    /*mpu.setXGyroOffset(105); // Offsets set using Calibration sketch
    mpu.setYGyroOffset(-22); // on 3/6/2016 for companion board unit v3
    mpu.setZGyroOffset(105);
    mpu.setXAccelOffset(-651);
    mpu.setYAccelOffset(-4385);
    mpu.setZAccelOffset(1339);
*/
    /*mpu.setXGyroOffset(88); // Offsets set using Calibration sketch
    mpu.setYGyroOffset(-104); //on 3/6/2016 for breadboard testing unit
    mpu.setZGyroOffset(-11);
    mpu.setXAccelOffset(-2002);
    mpu.setYAccelOffset(-446);
    mpu.setZAccelOffset(1180);
*/
    mpu.setXGyroOffset(39); // Offsets set using Calibration sketch
    mpu.setYGyroOffset(-12); //on 3/27/2017 for Companion Board v7 S/N 001
    mpu.setZGyroOffset(2357);
    mpu.setXAccelOffset(344);
    mpu.setYAccelOffset(-1434);
    mpu.setZAccelOffset(968);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        //(if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_PYRO_PIN, OUTPUT);
    pinMode(PYRO_PIN1, OUTPUT);
    pinMode(PYRO_PIN2, OUTPUT);
    pinMode(SW1, INPUT);
    pinMode(SW2, INPUT);
    pinMode(SW3, INPUT);
    digitalWrite(SW1, HIGH); //Set switch pins HIGH with internal pull-up resistor.
    digitalWrite(SW2, HIGH); //When switch is put in ON position, the value drops to GND.
    digitalWrite(SW3, HIGH);
    digitalWrite(PYRO_PIN1, HIGH);   //set SSR1 in ON position
    digitalWrite(PYRO_PIN2, HIGH); //set SSR2  in ON position
    digitalWrite(LED_PYRO_PIN, HIGH);      // Turn on LED to show relays set closed to allow
                                           //continuity circuit of altimeter to work

    // Read value of DIP switches and set tiltDegrees
    boolean reg = digitalRead(SW3);
    bitWrite(tiltDegrees, 0, !reg);
    reg = digitalRead(SW2);
    bitWrite(tiltDegrees, 1, !reg);
    reg = digitalRead(SW1);
    bitWrite(tiltDegrees, 2, !reg);
    tiltDegrees = (5*tiltDegrees) + 5;
    Serial.print("tiltDegrees = "); Serial.println(tiltDegrees);
    //countout(tiltDegrees);  // comment out if not using PIEZO buzzer
    flashcount(tiltDegrees);  // comment out if using a PIEZO buzzer
    //Serial.print("DLPFMode = "); Serial.println(mpu.getDLPFMode()); // used to check DLPFMode; default is 3 (44Hz)
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02)
    {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
        
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);

      // determine orientation of board and lock in using the orientFlag variable
      if ((aa.y > 3500) && !orientFlag)
      {
        orientation = HIGH;
        orientFlag = HIGH;
      }
      if ((aa.z > 3500) && !orientFlag)
      {
        orientation = LOW;
        orientFlag = HIGH; 
      }
        
      // get the appropriate ypr values from MPU
      if (orientation == HIGH)
      {
        mpu.dmpGetYawPitchRollOnEnd(ypr, &q, &gravity);  // Use if MPU board is vertical, Y-axis up
      }
      else
      {
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  // use if MPU board is horizontal, Z-axis up 
      }
      
      // Calculate pitch and roll values 
      float pitch = ypr[1] * 180/M_PI;    // define pitch variable to check angle of ascent
      float roll = ypr[2] * 180/M_PI;     // define roll variable to check angle of ascent
      Serial.print("pitch = "); Serial.print(pitch, 3); // print for troubleshooting
      Serial.print("\troll = "); Serial.print(roll, 3); // print for troubleshooting
      Serial.print("\tY-accel = "); Serial.println(aa.y, DEC);  // Use if MPU board is vertical; print for troubleshooting
      Serial.print("\tZ-accel = "); Serial.println(aa.z, DEC);  // Use if MPU board is horizontal; print for troubleshooting

      // Check to see if launch has occurred by examining the upwards acceleration.
      // This requires knowing the orientation of the board in the "orientation" variable.
      // To accomodate spurious spikes in acceleration and FIFO overflow events,
      // ensure the acceleration magnitude is reproducible for three consecutive measurements.
      
      if (orientation == HIGH)// detect launch at ~2g, use if MPU board is vertical
      {
        if (aa.y < 8192)
        {
          accelCount = 0;
        }
        else
        { 
          aCount();
        }
      }
      if (orientation == LOW)  // detect launch at ~2g, use if MPU board is horizontal
      {
        if (aa.z < 8192)
        {
          accelCount = 0;
        }
        else
        { 
          aCount();
        }
      }
      // if the launch has occurred, check if angles of ascent are within "tiltDegrees" of verticle.
      // Once the limits have been reached, the relay is latched open and cannot be closed again,
      // even if the angles of ascent goes smaller than "tiltDegrees".  This ensures the
      // ignition will not occur if the rocket is looping.  However, to accomodate spurious spikes in
      // pitch and roll values, ensure the tilt is reproducible for three consecutive measurements.
            
      if (latch == HIGH)
      {
        if (((pitch >= -tiltDegrees) && (pitch <= tiltDegrees)) && ((roll >= -tiltDegrees)
           && (roll <= tiltDegrees)) && (noStage != false))
        {                
          Serial.println("Okay to Stage");
          count = 0;  // reset count to zero as excursion of pitch or roll is not sustained
        }
        else
        {
          Count();
        }
      }
            
     //blink LED to indicate activity
     blinkState = !blinkState;
     digitalWrite(LED_PIN, blinkState);
  }
}

void Count() // tracks number of consecutive events above a tilt threshold to detect off-axis flight
{
  switch (count){
     case 0:                              // first excursion
       count = count + 1;
       Serial.println("Count is now 1");
       break;
     case 1:                              // second consecutive excursion
       count = count + 1;
       Serial.println("Count is now 2");
       break;
     case 2:                               // third and final consecutive excursion
        Serial.println("DO NOT STAGE");
        digitalWrite(PYRO_PIN1, LOW);   // set SSRs to not allow continuity (open)
        digitalWrite(PYRO_PIN2, LOW);   // from timer to ignitors
        noStage = false;                // set variable to latch the relays open
        digitalWrite(LED_PYRO_PIN, LOW);     // Turn off LED to show relays open
  }
}
void aCount() // tracks number of consecutive acceleration events above a threshold to detect launch
{
  switch (accelCount) {
     case 0:                              // first excursion
       accelCount = accelCount + 1;
       Serial.println("accelCount is now 1");
       break;
     case 1:                              // second consecutive excursion
       accelCount = accelCount + 1;
       Serial.println("accelCount is now 2");
       break;
     case 2:                               // third and final consecutive excursion
       Serial.println("Flight has started");
       latch = HIGH;                       // set latch HIGH to indicate launch has occurred
  }
}

/*
 * 
 // beeps j times
void beeptimes(int j)
{
  int i;
  for (i=1; i<=j; i++)
  {
    tone(beepPin, 4000, 200);
    delay(500);
  }
}
    
// sends out beep-coded-decimal for c
void countout(byte c)
{
  byte x;
  boolean First = true;
  byte digit=10;
  while (digit > 0)
  {
    x = c / digit;
  
    if ((!First) && (x ==0)) { beeptimes(10); delay(1000);}
    else {  if (x > 0) {beeptimes(x); delay(1000);} }
    if ((First) && (x > 0)){ First = false; }
    c = c % digit;
    digit = digit / 10;
  }
  return;
}
*/
// flashes LED j times
void flashtimes(int j)
{
  int i;
  for (i=1; i<=j; i++)
  {
    //tone(beepPin, 4000, 200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}

// sends out LED-flash-coded-decimal for c
void flashcount(byte c)
{
  byte x;
  boolean First = true;
  byte digit=10;
  while (digit > 0)
  {
    x = c / digit;
  
    if ((!First) && (x ==0)) { flashtimes(10); delay(1000);}
    else {  if (x > 0) {flashtimes(x); delay(1000);} }
    if ((First) && (x > 0)){ First = false; }
    c = c % digit;
    digit = digit / 10;
  }
  return;
}

