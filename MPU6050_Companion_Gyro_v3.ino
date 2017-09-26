// MPU6050_Companion_Gyro_v3
//
// A sketch to sense the orientation of a rocket during ascent and control two solid state relays (SSRs) placed
// between an altimeter or timer and an motor ignitor and a separation charge.  If the rocket's
// ascent deviates a set number of degrees or more from vertical, the SSRs are latched open,
// disallowing ignition of the motor and the separation charge.  This sketch supports the MPU6050
// GY-521 breakout board oriented with Y-axis skyward.
//
// Includes a 3-position DIP switch, to allow manual time-of-use selection
// of the critical angle of ascent that triggers the SSRs to open.
// The switches set the value of variable tiltDegrees according to this schedule:
//
//_______________________________
// |  tiltDegrees     d5  d4  d3 |
// |        5        OFF OFF OFF |
// |       10        OFF OFF ON  |
// |       15        OFF ON  OFF |
// |       20        OFF ON  ON  |
// |       25        ON  OFF OFF }
// |       30        ON  OFF ON  |
// |       35        ON  ON  OFF |
// |       40        ON  ON  ON  |
// | switch position  1   2   3  |
// ______________________________
//
// The printed circuit board can be obtained at OSH Park https://oshpark.com/shared_projects/Xa8MRGnW
//
/* ============================================
  ///////////////////////////////////////////////////////////////////////////////////////
  /*Terms of use
  ///////////////////////////////////////////////////////////////////////////////////////
  //THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  //IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  //FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  //AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  //LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  //OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  //THE SOFTWARE.


  ///////////////////////////////////////////////////////////////////////////////////////
  //Support
  ///////////////////////////////////////////////////////////////////////////////////////
  Website: http://www.brokking.net/imu.html
  Youtube: https://youtu.be/4BoIE8YQwM8
  Version: 1.0 (May 5, 2016)

  Software originally designed by Joop Brokking for Z-axis up for quadcopters, see above reference.
  Modified by Ed Chess for Y-axis up and for use with rocketry applications.
  To simplify calculations, the Y and Z axes data are interchanged, and the program assumes
  Z axis is up.  Thus we check acc_z for liftoff.
  ///////////////////////////////////////////////////////////////////////////////////////
  //Connections
  ///////////////////////////////////////////////////////////////////////////////////////
  MPU6050 - Arduino pro mini
  VCC  -  5V
  GND  -  GND
  SDA  -  A4
  SCL  -  A5

  LCD  - Arduino pro mini
  VCC  -  5V
  GND  -  GND
  SDA  -  A4
  SCL  -  A5

  Other components - Adruino pro mini
  SSR1 -  7
  SSR2 -  8
  LED  -  6
  SW3  -  3
  SW2  -  4
  SW1  -  5
  ===============================================
*/
//Wire library needed to communicate with MPU6050
#include <Wire.h>

// class default I2C address is 0x68 for MPU6050
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68; GY-561 breakout board has a 4.7k pull-down resistor on AD0, and 4.7k pull-up resistors on SCL and SDA
// AD0 high = 0x69

//Input and output definitions
#define LED_PIN 13        // Shows microprocessor activity
#define LED_PYRO_PIN 6    // Shows PYRO_PIN1 and PYRO_PIN2 status
#define PYRO_PIN1 7       // Pyro Channel 1 to SSR 1 (Event 1); HIGH if OK to stage
#define PYRO_PIN2 8       // Pyro Channel 2 to SSR 2 (Event 2); HIGH if OK to stage
#define SW3 3             // DIP switch position 3, lowest bit
#define SW2 4             // DIP switch position 2, middle bit
#define SW1 5             // DIP switch position 1, highest bit
//#define beepPin 9        // Piezo beeper; comment out if not using a beeper
//#define DEBUG    // for debug use

// main program control global variables
bool blinkState = false;    // flag to keep track of flashing LED state
bool noStage = false;      // flag to stop staging if tilt gets too large
byte tiltDegrees = 0;     // enter number of degrees tilt that will be allowed--25 or less is safest
bool latch = false;         // flag to keep track of Z-acceleration;  gets set HIGH upon launch detect
byte count = 0;           // variable to keep track of number of times in a row pitch or roll > tiltDegrees
byte accelCount = 0;      // variable to keep track of number of times in a row accel > 2G
bool angleFlag = false;     // flag to indicate that pitch and roll adjustments have been set

//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc, angle_roll_adjust, angle_pitch_adjust;
float angle_pitch_output, angle_roll_output;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  // initialize serial communication if feedback is desired

  #ifdef DEBUG {
    Serial.begin(115200);
    Serial.println(F("Initializing DMP..."));
  }
  #endif //(DEBUG)

  // configure the DMP
  setup_mpu_6050_registers(); //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PYRO_PIN, OUTPUT);
  pinMode(PYRO_PIN1, OUTPUT);
  pinMode(PYRO_PIN2, OUTPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  digitalWrite(SW1, HIGH);                // Set switch pins HIGH with internal pull-up resistor.
  digitalWrite(SW2, HIGH);                // When switch is put in ON position, the value drops to GND.
  digitalWrite(SW3, HIGH);
  digitalWrite(PYRO_PIN1, HIGH);          // Set SSR1 in ON position
  digitalWrite(PYRO_PIN2, HIGH);          // Set SSR2 in ON position
  digitalWrite(LED_PYRO_PIN, HIGH);       // Turn on LED to show relays set closed to allow continuity circuit of altimeter/timer to work
                                          
  // Read value of DIP switches and set tiltDegrees
  boolean reg = digitalRead(SW3);
  bitWrite(tiltDegrees, 0, !reg);
  reg = digitalRead(SW2);
  bitWrite(tiltDegrees, 1, !reg);
  reg = digitalRead(SW1);
  bitWrite(tiltDegrees, 2, !reg);
  tiltDegrees = (5 * tiltDegrees) + 5;
  #ifdef DEBUG
  Serial.print("tiltDegrees = "); Serial.println(tiltDegrees);
  #endif //(DEBUG)
  //countout(tiltDegrees);  // comment out if not using PIEZO buzzer
  flashcount(tiltDegrees);  // show tilt degrees by flashing LED

  delay(2000);
  digitalWrite(LED_PIN, HIGH);                                           // Indicate calibration on going
  
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                   // Run this code 2000 times
    read_mpu_6050_data();                                                // Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                                // Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                                // Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                                // Add the gyro z-axis offset to the gyro_z_cal variable
    delay(2);                                                            // Delay 2ms to simulate the 400Hz program loop
  }
  gyro_x_cal /= 2000;                                                    // Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                    // Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                    // Divide the gyro_z_cal variable by 2000 to get the avarage offset

  #ifdef DEBUG
  Serial.println("Gyro calibration complete");
  #endif //(DEBUG)
  
  digitalWrite(LED_PIN, LOW);                                            // All done with calibration, turn the LED off

  delay(2000);                                                           // Delay to show LED off before it gets turned on again in main loop

  loop_timer = micros();                                                 //Reset the loop timer
} //end void setup

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  read_mpu_6050_data();                                                  // Read the raw acc and gyro data from the MPU-6050

  //Accelerometer angle calculations
  if (!latch) {                                                          // Use accelerometer for pitch and roll calculations before launch
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); // Calculate the total accelerometer vector
    
    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    //angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;     // Calculate the pitch angle for Z-axis up
    //angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;     // Calculate the roll angle for Z-axis up
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * -57.296;   // Calculate the pitch angle for Y-axis up
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;     // Calculate the roll angle for Y-axis up

    //Correct for offsets before launch is detected
    if (!angleFlag) {
      angle_pitch_adjust = angle_pitch_acc;
      angle_roll_adjust = angle_roll_acc;
      angleFlag = true;
    }

    // Correct the measured angles for offsets
    angle_pitch_acc -= angle_pitch_adjust;                                              // Assumes rocket is pointed vertically at this time
    angle_roll_acc -= angle_roll_adjust;                                                // and sets initial pitch and roll angles to zero

    #ifdef DEBUG
    Serial.print("pitch: "); Serial.print(angle_pitch_acc, 1); Serial.print(" roll: "); Serial.println(angle_roll_acc);
    #endif //(DEBUG)

    // Check to see if launch has occurred by examining the upwards acceleration.
    // To accomodate spurious spikes in acceleration, ensure the acceleration magnitude
    // is reproducible for three consecutive measurements.  This will not correct for large chuffs, which will look like launch events.

    if (acc_z < 10000) //Recall that Y and Z axes are switched, so we query Z, which is really Y.
    {
      accelCount = 0;  // Reset accelCount to zero if acceleration of boost is not sustained
    }
    else
    {
      aCount();        // Used to error correct accel measurements to detect liftoff;  too fast for large motor chuffs.
    }
  }

  if (latch) {                                                           // Launch detected;  start using gyros for roll and pitch measurement
    gyro_x -= gyro_x_cal;                                                // Subtract the offset calibration value from the raw gyro_x value
    gyro_y -= gyro_y_cal;                                                // Subtract the offset calibration value from the raw gyro_y value
    gyro_z -= gyro_z_cal;                                                // Subtract the offset calibration value from the raw gyro_z value

    if (!set_gyro_angles) {                                              // First time through this loop after launch detection
      angle_pitch = angle_pitch_acc;                                     // Set the gyro pitch angle equal to the accelerometer pitch angle
      angle_roll = angle_roll_acc;                                       // Set the gyro roll angle equal to the accelerometer roll angle
      set_gyro_angles = true;                                            // Set the IMU started flag
    }

    if (set_gyro_angles) {                                               // Second time through this loop,
      //Gyro angle calculations
      /*0.0000611 = 1 / (250Hz * 65.5)                                   // Values calculated for various read gyro rates
      0.0000382 = 1 / (400Hz * 65.5)                                      // 500Hz is highest rate that maintains good gyro calculations
      0.0000305 = 1 / (500Hz * 65.5)                                      // with DEBUG not defined.
      0.0000254 = 1 / (600Hz * 65.5)
      0.0000191 = 1 / (800Hz * 65.5)
      */                                                                     //Comment out all lines below except lines with selected rate
      //angle_pitch += gyro_x * 0.0000611;                                   //250Hz; Calculate the traveled pitch angle and add this to the angle_pitch variable
      //angle_roll += gyro_y * 0.0000611;                                    //250Hz; Calculate the traveled roll angle and add this to the angle_roll variable
      //angle_pitch += gyro_x * 0.0000382;                                   //400Hz; Calculate the traveled pitch angle and add this to the angle_pitch variable
      //angle_roll += gyro_y * 0.0000382;                                    //400Hz; Calculate the traveled roll angle and add this to the angle_roll variable
      angle_pitch += gyro_x * 0.0000305;                                   //500Hz; Calculate the traveled pitch angle and add this to the angle_pitch variable
      angle_roll += gyro_y * 0.0000305;                                    //500Hz; Calculate the traveled roll angle and add this to the angle_roll variable
      //angle_pitch += gyro_x * 0.0000254;                                   //600Hz; Calculate the traveled pitch angle and add this to the angle_pitch variable
      //angle_roll += gyro_y * 0.0000254;                                    //600Hz; Calculate the traveled roll angle and add this to the angle_roll variable
      //angle_pitch += gyro_x * 0.0000191;                                   //800Hz; Calculate the traveled pitch angle and add this to the angle_pitch variable
      //angle_roll += gyro_y * 0.0000191;                                    //800Hz; Calculate the traveled roll angle and add this to the angle_roll variable

      // Calculate correction factor for different rates
      /*0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians for 250Hz
      0.000000667 = 0.0000382 * (3.142(PI) / 180degr) The Arduino sin function is in radians for 400Hz
      0.000000533 = 0.0000305 * (3.142(PI) / 180degr) The Arduino sin function is in radians for 500Hz
      0.000000444 = 0.0000254 * (3.142(PI) / 180degr) The Arduino sin function is in radians for 600Hz
      0.000000333 = 0.0000191 * (3.142(PI) / 180degr) The Arduino sin function is in radians for 800Hz
      */                                                                     //Comment out all lines below except lines with selected rate
      //angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //250Hz; If the IMU has yawed transfer the roll angle to the pitch angle
      //angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //250Hz; If the IMU has yawed transfer the pitch angle to the roll angle
      //angle_pitch += angle_roll * sin(gyro_z * 0.000000667);               //400Hz; If the IMU has yawed transfer the roll angle to the pitch angle
      //angle_roll -= angle_pitch * sin(gyro_z * 0.000000667);               //400Hz; If the IMU has yawed transfer the pitch angle to the roll angle
      angle_pitch += angle_roll * sin(gyro_z * 0.000000533);               //500Hz; If the IMU has yawed transfer the roll angle to the pitch angle
      angle_roll -= angle_pitch * sin(gyro_z * 0.000000533);               //500Hz; If the IMU has yawed transfer the pitch angle to the roll angle
      //angle_pitch += angle_roll * sin(gyro_z * 0.000000444);               //600Hz; If the IMU has yawed transfer the roll angle to the pitch angle
      //angle_roll -= angle_pitch * sin(gyro_z * 0.000000444);               //600Hz; If the IMU has yawed transfer the pitch angle to the roll angle
      //angle_pitch += angle_roll * sin(gyro_z * 0.000000333);               //800Hz; If the IMU has yawed transfer the roll angle to the pitch angle
      //angle_roll -= angle_pitch * sin(gyro_z * 0.000000333);               //800Hz; If the IMU has yawed transfer the pitch angle to the roll angle

      #ifdef DEBUG
      Serial.print("pitch: "); Serial.print(angle_pitch, 1); Serial.print(" roll: "); Serial.println(angle_roll);
      #endif //(DEBUG)
    }

    // Once the launch has occurred, check if angles of ascent are within "tiltDegrees" of vertical.
    // Once the limits have been reached, the relay is latched open and cannot be closed again,
    // even if the angles of ascent goes smaller than "tiltDegrees".  This ensures the
    // ignition will not occur if the rocket is looping.  However, to accomodate spurious spikes in
    // pitch and roll values, ensure the tilt is reproducible for three consecutive measurements.


    if (((angle_pitch >= -tiltDegrees) && (angle_pitch <= tiltDegrees)) && ((angle_roll >= -tiltDegrees)
        && (angle_roll <= tiltDegrees)) && (noStage == false))
    {
      //Serial.println("Okay to Stage");
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

  //while (micros() - loop_timer < 4000);       // Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  //while (micros() - loop_timer < 2500);       // Wait until the loop_timer reaches 2500us (400Hz) before starting the next loop
  while (micros() - loop_timer < 2000);       // Wait until the loop_timer reaches 2000us (500Hz) before starting the next loop
  //while (micros() - loop_timer < 1667);       // Wait until the loop_timer reaches 1667us (600Hz) before starting the next loop
  //while (micros() - loop_timer < 1250);       // Wait until the loop_timer reaches 1250us (800Hz) before starting the next loop
  loop_timer = micros();                        // Reset the loop timer
  
}  //end void main loop

void Count() // tracks number of consecutive events above a tilt threshold to detect off-axis flight
{
  switch (count) {
    case 0:                              // first excursion
      count = count + 1;
      //Serial.println("Count is now 1");
      break;
    case 1:                              // second consecutive excursion
      count = count + 1;
      //Serial.println("Count is now 2");
      break;
    case 2:                               // third and final consecutive excursion
      //Serial.println("DO NOT STAGE");   // the next two lines switch off the two relays, disconnecting e-matches from timer board
      digitalWrite(PYRO_PIN1, LOW);   //  COMMENT OUT IF YOU WANT EVENT 1 TO OCCUR REGARDLESS OF ROCKET ATTITUDE (Use for separation charge)
      digitalWrite(PYRO_PIN2, LOW);   // Event 2 (Use for second stage motor ignition)
      noStage = true;                // set variable to latch the relays open
      digitalWrite(LED_PYRO_PIN, LOW);     // Turn off LED to show relays open
  }
}  //end void Count

void aCount() // tracks number of consecutive acceleration events above a threshold to detect launch
{
  switch (accelCount) {
    case 0:                              // first excursion
      accelCount = accelCount + 1;
      //Serial.println("accelCount is now 1");
      break;
    case 1:                              // second consecutive excursion
      accelCount = accelCount + 1;
      //Serial.println("accelCount is now 2");
      break;
    case 2:                               // third and final consecutive excursion
      //Serial.println("Flight has started");
      latch = true;                       // set latch HIGH to indicate launch has occurred
  }
}  //end void aCount

/*
  void beeptimes(int j) // comment out if not using beeper  // beeps j times
  {
  int i;
  for (i=1; i<=j; i++)
  {
    tone(beepPin, 4000, 200);
    delay(500);
  }
  }  //end void beeptimes
*/
  
/*
  void countout(byte c)   // sends out beep-coded-decimal for c
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
  }  //end void countout
*/

void flashtimes(int j)  // flashes LED j times
{
  int i;
  for (i = 1; i <= j; i++)
  {
    //tone(beepPin, 4000, 200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}  //end void flashtimes

void flashcount(byte c)   // sends out LED-flash-coded-decimal for byte c
{
  byte x;
  boolean First = true;
  byte digit = 10;
  while (digit > 0)
  {
    x = c / digit;

    if ((!First) && (x == 0)) {
      flashtimes(10);
      delay(1000);
    }
    else {
      if (x > 0) {
        flashtimes(x);
        delay(1000);
      }
    }
    if ((First) && (x > 0)) {
      First = false;
    }
    c = c % digit;
    digit = digit / 10;
  }
  return;
}  //end void flashcount

void read_mpu_6050_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
  //Exchange y and z reading order for Y-axis up orientation
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
  //acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable for Z-axis up
  //acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable for Z-axis up
  acc_z = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable for Y-axis up
  acc_y = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable for Y-axis up
  temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
  //gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable for Z-axis up
  //gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable for Z-axis up
  gyro_z = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable for Y-axis up
  gyro_y = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable for Y-axis up
}  //end void read_mpu_6050_data

void setup_mpu_6050_registers() {                                      // initializes the MPU 6050 registers and starts MPU
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}  //end void setup_mpu_6050_registers

