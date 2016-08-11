//======Code written and debugged by Moaaz Elsayed & Adham Ayman======//
//======Capstone Unibo 2015===========================================//
//======Lateral System Code 2=========================================//

//This code has been written with knowledge and education gained from the following people:
// Nevit Dilmen: http://nevit.blogspot.ca/2014/07/arduino-mpu-6050-pitch-roll-with-2.html
// Ira Hart: http://www.instructables.com/id/Rideable-Segway-Clone-Low-Cost-and-Easy-Build/

#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define ACCEL_GAIN 18.0 //
#define GYRO_GAIN 5.0   //
#define DEBUG_FORCE_DEADMAN_SWITCH 1 //normal
#define DEBUG_ENABLE_PRINTING 1 //normal
#define DEBUG_DISABLE_MOTORS 0 //normal
#define SABER_MOTOR1_FULL_FORWARD 0
#define SABER_MOTOR1_FULL_REVERSE 180
#define SABER_ALL_STOP  0
#define MPU_INT 0 //is on pin 2

#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define PEDAL_THRESH 800
#define STABLE_THROT 150
#define GRACE_PERIOD_MS 5000
#define BRAKE_UP 180
#define BRAKE_DOWN 0
#define UPDATE_CYCLE 5

#define TIMER_START() loopStartTime = millis();
#define TIMER_ATLEAST(MS) if((timer_p = millis() - loopStartTime) < (MS)) delay((MS) - timer_p);

Servo esc1;
Servo esc2;
Servo brake;
Servo SS1;
Servo SS2;

MPU6050 mpu;

int Press1 = A0; //Load Sensor 1
int Press2 = A1; //Load Sensor 2
int throt = 0;

bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;  // count of all bytes currently in FIFO
uint8_t fifoBuffer[64] = {0}; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t gyro[3];        // [x, y, z]            gyro vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float angle_Y, angular_rate_Y, angular_rate_X;
float angle_X, angle_Z, angular_rate_Z;
float cycle_time = 0.005; //seconds per cycle - currently 5 milliseconds per loop of the program.  
int STD_LOOP_TIME = 4; //4= 5mS loop time 
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0, timer_st, timer_p;
uint8_t skip_count = UPDATE_CYCLE - 1;
enum PressState {PS_INIT = 0, PRESS_ACCL, PRESS_STABLE, NO_PRESS_GRACE, NO_PRESS_DECL} press_state = PS_INIT;
long press_start = 0, press_elapsed = 0;
// Aruino Pin Assignments:
int deadmanButtonPin    = 9;  // deadman button is digital input pin 
int balanceForwardPin           = 5;  //if digital pin  is 5V then reduce balancepoint variable. Allows manual fine tune of the ideal target balance point
int balanceBackwardPin          = 8;  //if digital pin  is 5V then increase balancepoint variable. Allows manual fine tune of the ideal target balance point
int steeringLeftPin           = 7;  //digital pin Used to steer
int steeringRightPin          = 4;  //digital pin Used to steer the other way.
int greenLedPin     = 10; // bi-color LED connected to digital pin 
int commonHighLedPin          = 11; // bi-color LED connected to digital pin 
int redLedPin       = 12; // bi-color LED connected to digital pin 
int oscopePin       = 3; //spare pin or to oscope to see cycle time for debug
float cur_speed;
float level=0;
float Steering;
float SteerValue;
float SteerCorrect;
int   Steer = 0;
float x_acc;
float SG_filter_result;
float x_accdeg;
float initial_angular_rate_Y = 0;
float initial_angular_rate_Y_sum = 0;
float initial_angular_rate_X = 0;
float initial_angular_rate_X_sum = 0;
float gangleratedeg;
float gangleratedeg2;
float gangleraterads;
int   SteerLeftPin;
int   SteerRightPin;
int   DeadManPin;
//add for deadman debounce
int DeadManPin_temp = 1; //this variable is from the digitalRead
int DeadManPin_temp_old = 1; //this variable is delayed from the digitalRead
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce delay in mSecs
float overallgain; 
float gyroangle_dt;
float angle;
float anglerads;
float balance_torque;
float softstart;
float Balance_point;
float balancetrim = 0;
int balancelForward;
int balancelBackward;
float gv0, gv1, gv2, gv3, gv4, gv5, gv6;  //Sav Golay variables.  filter for accelerometer called Savitsky Golay filter.
int i;
int j;
int tipstart;
signed char Motor1percent;
int deadman_occured_flag;
int skip = 0;//for debug
float ti_constant = 3;
const float ANGLE_GAIN = 1.20; //20% increase in angle measurement.
float aa_constant = 0.005; //this means 0.5% of the accelerometer reading is fed into angle of tilt calculation with every loop of program (to correct the gyro).
// ================================================================
// ===         INTERRUPT DETECTION ROUTINE        ===
// ================================================================
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high

void dmpDataReady()
{
  mpuInterrupt = true;
}

// void mydmpDataReady()
// {
//     if(mpu.getFIFOCount() < packetSize) return;
//     mpuInterrupt = true;
//     mpu.getFIFOBytes(fifoBuffer, packetSize);

//     mpu.resetFIFO();
// }
// ================================================================
// ===            INITIAL SETUP            ===
// ================================================================
void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  esc1.attach(6); //Attach Flywheel Right
  esc2.attach(5); //Attach Flywheel Left
  brake.attach(11); //Attach Brake Servo
  SS1.attach(10);
  SS2.attach(9);
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial) delay(20); // wait for Leonardo enumeration, others continue immediately
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

  // // wait for ready
  // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  // while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());        // wait for data
  // while (Serial.available() && Serial.read()); // empty buffer again

 
    // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    // attachInterrupt(0, mydmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  //Initialize IO pins
  //Init LED
  pinMode(redLedPin, OUTPUT);      // sets the digital pin as output
  pinMode(commonHighLedPin, OUTPUT);      // sets the digital pin as output
  pinMode(greenLedPin, OUTPUT);      // sets the digital pin as output
  digitalWrite(redLedPin, LOW);   //LED defaults to RED at setup
  digitalWrite(commonHighLedPin, HIGH);   //LED common is high.
  digitalWrite(greenLedPin, HIGH);   //LED 
  //digital inputs
  pinMode(deadmanButtonPin, INPUT);
  digitalWrite(deadmanButtonPin, HIGH);                   // turn on pullup resistors
  pinMode(balanceForwardPin, INPUT);
  digitalWrite(balanceForwardPin, HIGH);                  // turn on pullup resistors
  pinMode(balanceBackwardPin, INPUT);
  digitalWrite(balanceBackwardPin, HIGH);                 // turn on pullup resistors
  pinMode(steeringLeftPin, INPUT);
  digitalWrite(steeringLeftPin, HIGH);                    // turn on pullup resistors
  pinMode(steeringRightPin, INPUT);
  digitalWrite(steeringRightPin, HIGH);                   // turn on pullup resistors
  //init oscope output
  pinMode(oscopePin, OUTPUT);      // sets the digital pin as output
  digitalWrite(oscopePin, LOW);   //

  //Brushless esc controller calibration
  esc1.write(1023); //Calibrate max throttle Flywheel Right
  esc2.write(1023); //Calibrate max throttle Flywheel Left
  TIMER_START();
  // At start of loop, read the accel/gyro multiple times to get an average baseline value.  
  // This will be subtracted from the current value in the balance loop.
  tipstart = 0;
  cur_speed = 0;
  level = 0;
  Steer = 0;
  angle = 0;
  Steering = 512;
  SteerValue = 512;
  overallgain = 0.5;  //softstart value. Gain will now rise to final of 0.5 at rate of 0.005 per program loop. 
  j = 0;
  while (j<7) 
  {
    if(mpuInterrupt)
    {
      mpuIntStatus = mpu.getIntStatus();
      if(mpuIntStatus & 0x10)
      {
          mpu.resetFIFO();
          continue;
      }
      else if((mpuIntStatus & 0x02) && (mpu.getFIFOCount() > packetSize))
      {
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          mpu.resetFIFO();
          ++j;
      }
      mpuInterrupt = false;
    }
    else 
    {
      delay (10); //10ms
      continue;
    }
    read_accel_gyro();
    initial_angular_rate_Y_sum = (float) initial_angular_rate_Y_sum  + angular_rate_Y; //sum of the 7 readings of front/back tilt gyro
    initial_angular_rate_X_sum = (float) initial_angular_rate_X_sum  + angular_rate_X; //sum of the 7 readings left/right steer gyro
  }
  initial_angular_rate_Y = (float) initial_angular_rate_Y_sum/7;  //initial front/back tilt gyro
  initial_angular_rate_X = (float) initial_angular_rate_X_sum/7;  //initial left/right steer gyro
  TIMER_ATLEAST(2000);
  esc1.write(0); //Calibrate min throttle Flywheel Right
  esc2.write(0); //Calibrate min throttle Flywheel Left
}
// ================================================================
// ===          MAIN PROGRAM LOOP          ===
// ================================================================
void loop()
{
  if (!dmpReady) return;
  if(mpuInterrupt)
  {
    mpuIntStatus = mpu.getIntStatus();
    if(mpuIntStatus & 0x10)
    {
        Serial.println(F("FIFO overflow!"));
        mpu.resetFIFO();
    }
    else if((mpuIntStatus & 0x02) && (mpu.getFIFOCount() > packetSize))
    {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();
    }
    mpuInterrupt = false;
  }
  int PressS1= analogRead(Press1);
  int PressS2= analogRead(Press2);
  esc1.write (throt);
  esc2.write (throt);
  switch(press_state)
  {
    case PS_INIT:
    {
      if (PressS1 >= PEDAL_THRESH || PressS2 >= PEDAL_THRESH)
        press_state = PRESS_ACCL;
      break;
    }
    case PRESS_ACCL:
    {
      if (PressS1 >= PEDAL_THRESH || PressS2 >= PEDAL_THRESH)
      {
        skip_count = (skip_count + 1) % UPDATE_CYCLE;
        if(!skip_count) ++throt;
        if(throt == STABLE_THROT)
        {
          press_state = PRESS_STABLE;
          brake.write(BRAKE_UP);
        }
      }
      else
      {
        --throt;
        press_state = NO_PRESS_DECL;
      }
      break;
    }
    case PRESS_STABLE:
    {
      press_start = millis();
      read_accel_gyro(); // read accel/gyro
      do_calculations(); //do math
      set_motor(); //set motors up
      // if (overallgain < 0.5) overallgain = (float)overallgain + 0.005;
      // else if (overallgain > 0.5) overallgain = 0.5;
      if (PressS1 < PEDAL_THRESH && PressS2 < PEDAL_THRESH)
      {
        press_state = NO_PRESS_GRACE;
      }
      break;
    }
    case NO_PRESS_GRACE:
    {
      press_elapsed = millis();
      if (PressS1 >= PEDAL_THRESH || PressS2 >= PEDAL_THRESH)
      {
        press_state = PRESS_STABLE;
      }
      else
      {
        if(press_elapsed - press_start > GRACE_PERIOD_MS)
        {
          --throt;
          press_state = NO_PRESS_DECL;
          brake.write(BRAKE_DOWN);
        }
      }
      break;
    }
    case NO_PRESS_DECL:
    {
      if (PressS1 >= PEDAL_THRESH || PressS2 >= PEDAL_THRESH)
      {
        ++throt;
        press_state = PRESS_ACCL;
      }
      else
      {
        skip_count = (skip_count + 1) % UPDATE_CYCLE;
        if(!skip_count) --throt;
        if(throt == 0)
        {
          press_state = PS_INIT;
        }
      }
      break;
    }
  }

  lastLoopUsefulTime = millis() - loopStartTime;
  if (lastLoopUsefulTime < STD_LOOP_TIME)
    delay(STD_LOOP_TIME - lastLoopUsefulTime);
  loopStartTime = millis();
}



void read_accel_gyro()  
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);  // access the shared data
  mpu.dmpGetGyro(gyro, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // angle and angular rate
  angle_X = ypr[0]* RAD_TO_DEG;             // not used...0 is center of gravity offset
  angle_Y = ypr[1]* RAD_TO_DEG;             // Accel for Tilt, 0 is center of gravity offset
  angle_Z = ypr[2]* RAD_TO_DEG;             // not used...0 is center of gravity offset
  angular_rate_X = ((double)gyro[0]/131.0); // Gyro for steering, in degs/sec.
  angular_rate_Y = ((double)gyro[1]/131.0); // Gyro for tilt, in degs/sec.
  angular_rate_Z = ((double)gyro[2]/131.0); // Gyro for X, in degs/sec.
 
  angular_rate_X = angular_rate_X * RAD_TO_DEG; // Gyro for steering, in degs/sec.
  angular_rate_Y = angular_rate_Y * RAD_TO_DEG; // Gyro for tilt, 
  angular_rate_Z = angular_rate_Z * RAD_TO_DEG; // Gyro for X
}

////////////////////////////////////////////////////////////////////////////////
void do_calculations()  
{                    
  SteerLeftPin = digitalRead(steeringLeftPin);
  SteerRightPin = digitalRead(steeringRightPin);
  
  //Start Debounce Deadman button
  DeadManPin_temp = digitalRead(deadmanButtonPin);
  
  // If the switch changed, due to noise or pressing:
  if (DeadManPin_temp != DeadManPin_temp_old) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  DeadManPin_temp_old = DeadManPin_temp;
  
  // filter out any deadman switch bounce noise by setting a time buffer
  // dont allow a change from the deadman switch unless 50 mSec has elapsed since the last solid value.
  if ( (millis() - lastDebounceTime) > debounceDelay){
    if (DeadManPin_temp == HIGH) { //DeadManPin = deadman debounced.  Stop motors if DeadManPin=1
      DeadManPin = 1;
    } 
    else {   //if (DeadManPin_temp == LOW) 
      DeadManPin = 0;
    }//close if/else
  }//close if(time buffer)
  //End of deadman debounce
  
  balancelForward = digitalRead(balanceForwardPin);
  balancelBackward = digitalRead(balanceBackwardPin);
    
  if (balancelForward == 0) balancetrim = balancetrim - 0.04; //if pressing balance point adjust switch then slowly alter the balancetrim variable by 0.04 per loop of the program 
  //while you are pressing the switch
  if (balancelBackward == 0) balancetrim = balancetrim + 0.04; //same again in other direction
  if (balancetrim < -30) balancetrim = -30; //stops you going too far with this
  if (balancetrim > 30) balancetrim = 30; //stops you going too far the other way
   
  // Savitsky Golay filter for accelerometer readings. It is better than a simple rolling average which is always out of date.
  // SG filter looks at trend of last few readings, projects a curve into the future, then takes mean of whole lot, giving you a more "current" value - Neat!
  // Lots of theory on this on net.
  gv0 = gv1;
  gv1 = gv2;
  gv2 = gv3;
  gv3 = gv4;
  gv4 = gv5;
  gv5 = gv6;
  gv6 = (float) angle_Y; //from digital gyro accelerometer  IDH
  
  //SG_filter_result is the accelerometer value from the rolling SG filter on the 0-1023 scale
  SG_filter_result = (float) ((-2*gv0) + (3*gv1) + (6*gv2) + (7*gv3) + (6*gv4) + (3*gv5) + (-2*gv6))/21; 
  
  //*****START OF STEERING SECTION
  //Used to adjust steering from drift
  gangleratedeg2 = angular_rate_X - initial_angular_rate_X;  //IDH subtract curent value from inital value to get delta.
  
  if (SteerLeftPin == 1 && SteerRightPin == 1){ // NO steering wanted. Use second gyro to maintain a (roughly) straight line heading (it will drift a bit).
                        
    SteerCorrect = 0; //blocks the direction stabiliser unless rate of turn exceeds -10 or +10 degrees per sec
    if (gangleratedeg2 > 10 || gangleratedeg2 < -10) {   //resists turning if turn rate exceeds 10deg per sec
      SteerCorrect = (float) 0.4 * gangleratedeg2; //vary the 0.4 according to how much "resistance" to being nudged off course you want.
      //a value called SteerCorrect is added to the steering value proportional to the rate of unwanted turning. It keeps getting
      //larger if this condition is still being satisfied i.e. still turning >10deg per sec until the change has been resisted.
      //can experiment with the value of 10. Try 5 deg per sec if you want - play around - this can probably be improved
      //but if you try to turn it fast with your hand while balancing you will feel it resisting you so it does work
      //also, when coming to a stop, one motor has a bit more friction than the other so as this motor stops first then as board
      //comes to standstill it spins round and you can fall off. This is original reason I built in this feature.
      //if motors have same friction you will not notice it so much.
    }
    SteerValue = 512;            
  }
  else { //(SteerLeftPin == 0 || SteerRightPin == 0) We DO want to steer                        
       
    //note: SteerValue of 512 is straight ahead

    if (SteerLeftPin == 0) {
      SteerValue = 612; //add some some right turn power. Experimentally determined.
    }                                                                
                        
    //steer the other way             
    if (SteerRightPin == 0) {
      SteerValue = 412; //add some some left turn power. Experimentally determined.
    }                                                  
                 
    SteerCorrect = 0;
  }        
  //*****END OF STEERING SECTION
  
        
  //Angle Gain
  SG_filter_result = (float) SG_filter_result * ANGLE_GAIN;
 
  // Balancetrim is front/back balance tip adjustment from switch
  // Sensor tilt number below is Determined experimentally. Bigger is more tilted forward.  It needs to change if you adjust ANGLE_GAIN.
  x_accdeg = (float)((SG_filter_result - (80 + balancetrim)) * (1.0));  
  
  if (x_accdeg < -72) x_accdeg = -72; //put in range.
  if (x_accdeg > 72) x_accdeg = 72;
  
  //For digital gyro here 
  gangleratedeg = (float)(angular_rate_Y - initial_angular_rate_Y); // IDH        
  if (gangleratedeg < -110) gangleratedeg = -110;
  if (gangleratedeg > 110) gangleratedeg = 110;
   
  //Key calculations. Gyro measures rate of tilt gangleratedeg in degrees. We know time since last measurement is cycle_time (10ms) so can work out much we have tipped over since last measurement
  //What is ti variable? Strictly it should be 1. However if you tilt board, then it moves along at an angle, then SLOWLY comes back to level point as it is moving along
  //this suggests the gyro is slightly underestimating the rate of tilt and the accelerometer is correcting it (slowly as it is meant to).
  //This is why, by trial and error, I have increased ti to 3 at start of program where I define my variables.
  //experiment with this variable and see how it behaves. Temporarily reconfigure the overallgain potentiometer as an input to change ti and experiment with this variable
  //potentiometer is useful for this sort of experiment. You can alter any variable on the fly by temporarily using the potentiometer to adjust it and see what effect it has

  gyroangle_dt = (float) ti_constant * cycle_time * gangleratedeg; //e.g  = 3*0.01*gyro_reading
 
  gangleraterads = (float) gangleratedeg * 0.017453; //convert to radians - just a scaling issue from history

  //Complementary Filter.
  angle = (float) ((1-aa_constant) * (angle + gyroangle_dt)) + (aa_constant * x_accdeg);//aa=(0.005) allows us to feed a bit (0.5%) of the accelerometer data into the angle calculation
  //so it slowly corrects the gyro (which drifts slowly with time). Accel sensitive to vibration though so aa does not want to be too large.
  //this is why these boards do not work if an accel only is used. We use gyro to do short term tilt measurements because it is insensitive to vibration

  //Complementary Filter. the approximate formula to combine the accelerometer and gyroscope data is:
  //Filtered Angle = Î± Ã— (Gyroscope Angle) + (1 âˆ’ Î±) Ã— (Accelerometer Angle)     where
  //Î± = Ï„/(Ï„ + Î”t)   and   (Gyroscope Angle) = (Last Measured Filtered Angle) + Ï‰Ã—Î”t
  //Î”t = sampling rate, Ï„ = time constant greater than timescale of typical accelerometer noise
 
  anglerads = (float) angle * 0.017453; //converting to radians again a historic scaling issue from past software
  
  balance_torque = (float) (ACCEL_GAIN * anglerads) +  //from accelerometer
    (GYRO_GAIN * gangleraterads); //from Gyro 

  //balance torque is motor control variable we would use even if we just ahd one motor. It is what is required to make the thing balance only.
  //the values of 4.5 and 0.5 came from Trevor Blackwell's segway clone experiments and were derived by good old trial and error
  //I have also found them to be about right
  //We set the torque proportionally to the actual angle of tilt (anglerads), and also proportional to the RATE of tipping over (ganglerate rads)
  //the 4.5 and the 0.5 set the amount of each we use - play around with them if you want.
  //Much more on all this, PID control etc on my website

  cur_speed = (float) (cur_speed + (anglerads * 6 * cycle_time)) * 0.999;
  //this is not current speed. We do not know actual speed as we have no wheel rotation encoders. This is a type of accelerator pedal effect:
  //this variable increases with each loop of the program IF board is deliberately held at an angle (by rider for example)
  //So it means "if we are STILL tilted, speed up a bit" and it keeps accelerating as long as you hold it tilted.
  //You do NOT need this to just balance, but to go up a slight incline for example you would need it: if board hits incline and then stops - if you hold it
  //tilted for long eneough, it will eventually go up the slope (so long as motors powerfull enough and motor controller powerful enough)
  //Why the 0.999 value? I got this from the SeWii project code - thanks!
  //If you have built up a large cur_speed value and you tilt it back to come to a standstill, you will have to keep it tilted back even when you have come to rest
  //i.e. board will stop moving OK but will now not be level as you are tiliting it back other way to counteract this large cur_speed value
  //The 0.999 means that if you bring board level after a long period tilted forwards, the cur_speed value magically decays away to nothing and your board
  //is now not only stationary but also level!

  level = (float)(balance_torque + cur_speed) * overallgain;  //final overall gain = 0.5
  
} //end do_calculations


////////////////////////////////////////////////////////////////////////////////
void set_motor()   
{
  int cSpeedVal_Motor1 = 0;
  static int lastVal = 0;
  
  level = level * 20; //changes it to a scale of about -100 to +100 works ..OK 
  if (level < -100) {level = -100;}
  if (level > 100) {level = 100;}
  
  Steer = (float) SteerValue - SteerCorrect;  //at this point is on the 0-1023 scale 
  //SteerValue is either 512 for dead ahead or bigger/smaller if you are pressing steering switch left or right
  //SteerCorrect is the "adjustment" made by the second gyro that resists sudden turns if one wheel hits a small object for example.  
  Steer = (Steer - 512) * 0.09;   //gets it down from 0-1023 (with 512 as the middle no-steer point) to -100 to +100 with 0 as the middle no-steer point on scale
  
  //set motors using the simplified serial Sabertooth protocol (same for smaller 2 x 5 Watt Sabertooth by the way)                 
  Motor1percent = (signed char) level + Steer;
 
  if (Motor1percent > 100) Motor1percent = 100;
  if (Motor1percent < -100) Motor1percent = -100;

  //debug:
  if (DEBUG_FORCE_DEADMAN_SWITCH == 1) 
      DeadManPin = 0;
 
  //if not pressing deadman button on hand controller - cut everything
  if (DeadManPin > 0)
  { 
    level = 0;
    Steer = 0;
    Motor1percent = 0;
   // Motor2percent = 0;
    digitalWrite(redLedPin, LOW);      //LED is red when stopped.
    digitalWrite(greenLedPin, HIGH);   //LED is red when stopped.
    deadman_occured_flag = 1; //set flag to force jump to start when deadman is released.
  }//End of deadman switch release 
  else if (deadman_occured_flag == 1 ) 
  { //deadman is pressed
    deadman_occured_flag = 0; 
    digitalWrite(redLedPin, HIGH);   //LED is green when running.
    digitalWrite(greenLedPin, LOW);   //LED is green when running.
    
    loop(); //start loop again to start from the beginning.
  }//End of deadman 

  cSpeedVal_Motor1 = map (Motor1percent, -100, 100, SABER_MOTOR1_FULL_REVERSE, SABER_MOTOR1_FULL_FORWARD);

  SS1.write ( cSpeedVal_Motor1 );
  SS2.write ( cSpeedVal_Motor1 );             
  // if(cSpeedVal_Motor1 != lastVal)
  // {
  //  SS1.write ( cSpeedVal_Motor1 );
  //  SS2.write ( cSpeedVal_Motor1 );
  //  lastVal = cSpeedVal_Motor1;
  // }
#ifdef OUTPUT_READABLE_YAWPITCHROLL
  Serial.print(" Yaw: ");
  Serial.print(angle_X);
  Serial.print(" Pitch: ");
  Serial.print(angle_Y);
  Serial.print(" Roll: ");
  Serial.print(angle_Z);
  Serial.print(" ang_rate_X: ");
  Serial.print(angular_rate_X);
  Serial.print(" ang_rate_Y: ");
  Serial.print(angular_rate_Y);
  Serial.print(" ang_rate_Z: ");
  Serial.print(angular_rate_Z);
  Serial.print("   Mot1%: ");
  Serial.println(Motor1percent);
  // Serial.print(" x_accdeg: ");
  // Serial.print(x_accdeg);
  // Serial.print(" gangratedeg: ");
  // Serial.print(gangleratedeg);
  // Serial.print(" gyroangle_dt: ");
  // Serial.print(gyroangle_dt);
  // Serial.print(" cur_speed: ");
  // Serial.print(cur_speed);
  // Serial.print(" level: ");
  // Serial.println(level);
#endif
}

