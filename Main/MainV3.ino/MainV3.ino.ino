
//****************************************************************************************************************************************************
// *** Arduino robot program V3
// *** Version: 2016.09.21
// *** Developer: Wolfgang Glück
// ***
// *** Supported hardware:
// ***   - 2 motors 12V with gear and encoder                       (EMG30, Manu systems AG)
// ***   - Motor control board (MCB)
// ***   - Steering gear with servo                                 (HPI Buggy flux 1/10 axle stubs, Servo SC 1251 MG)
// ***   - Arduino mega                                             (Spider)
// ***   - Compass                                                  (CMPS11 via 10cm I2C bus, using internal pullups by default)
// ***                                                              (For calibration use separated program CPMS11Calibration)
// ***   - 4 x Ultra-sonic sensor                                   (MB7076 XL-MaxSonar-WRLA1, MaxBotix)
// ***   - 1 x Infra red Sensor                                     (Sharp 2Y0A21)
// ***   - 11,1 V Battery for Drives, Arduino and MCB               (LiPO Akku 3S 11.1V 10'000 mAh; Connector Balancer XH, Connector Power EC-5)
// ***   - 5 V / Gnd     for MCB is connected to Arduino 5 V / Gnd
// ***   - 5 V   Battery for Raspberry pi                           (VOLTCRAFT Li-Io 20800 mAh PB-19 POWERBANK, Connector Charging cabele USB Micro, Connector Power USB 2A)
// ***
// ***   - Raspberry pi 2
// ***   - SD card 32GB
// ***   - camera
// ***   - W-LAN adapter                                            (EW-7612UAN V2)
// ***   - Audio Stick
// ***   - USB cable from Raspberry pi to Arduino
// ***
// *** Functions:
// ***   - (ok) One time compass calibration. Use separate program
// ***   - (ok) Read and supervise analog values like battery voltage and motor current
// ***   - (ok) Read compass values via I2C
// ***   - (ok) Smoothe compass value for a calm display
// ***   - (ok) Supervise roll and pitch
// ***   - (ok) Read encoder
// ***   - (ok) Read distances
// ***   - (ok) Send analog and digital values to USB interface
// ***   - (ok) Get and execute driving commands via USB interface (transition beween driving commands directly is allowed)
// ***          (Stop, ForwardSlow, ForwardHalf, ForwardFull)
// ***          (SteeringAhead, SteeringLeft, SteeringRight in combination with Forward Commands)
// ***          (TurnSlow90Left, TurnSlow90Right, TurnSlow180Left, turnSlowTo value)
// ***   - (ok) Get switch on/off command for audio amplifier
// ***   - (ok) Get reset command for encoder values
// ***   - (ok) Get status about W-LAN from USB interface
// ***   - (ok) Get switch command on/off for audio amplifier
// ***   - (  ) Get align command for forward direction control
// ***   - (ok) Supervise communication status for USB interface
// ***   - (ok) Emergency stop if any battery voltage level is low or an obstruction is close or an abyss is detected
// ***          or any motor stall or pitch or roll exceeds a limit or communication to USB is down or W-Lan is down
// *************************************************************************************************************************

// Definitions
// *************************************************************************************************************************
// *************************************************************************************************************************
#include <math.h>
#include <Wire.h>
#include <Servo.h>

// Servo
// *************************************************************************************************************************
Servo steeringservo;
#define ServoPWM 7
#define ServoMiddle 90
#define ServoExtreme 35
#define ServoLeft ServoMiddle-ServoExtreme
#define ServoRight ServoMiddle+ServoExtreme
#define ServoPmin 470
#define ServoPmax 2400


// Compass
// *************************************************************************************************************************
#define CMPS11_VCC 22        // VCC for dynamic power on via digital output
#define CMPS11_ADDRESS 0x60  // Address of CMPS11 shifted right one bit for arduino wire library
#define CMPS11_Byte1 0xF0    // command for calibration
#define CMPS11_Byte2 0xF5    // command for calibration
#define CMPS11_Byte3 0xF7    // command for calibration
#define CMPS11_Byte4 0xF8    // command for calibration mode off
#define ANGLE_8  1           // Register to read from
unsigned char high_byte, low_byte, angle8;
char pitch, roll;
int angle16;
int intermediateAngle;
float angleVectorList [10][2];  // Shift register for smoothing compass direction value
float vectora1 = 0.0;
float vectora2 = 0.0;
float vectorLength = 0.0;
float cosinus = 0.0;
float angle = 0.0;

#define UpitchLimit 30        // + - limit in degrees
#define LpitchLimit -30
#define UrollLimit 20         // + - limit in degrees
#define LrollLimit -20
boolean UpitchLimitExceeded = false;
boolean LpitchLimitExceeded = false;
boolean UrollLimitExceeded = false;
boolean LrollLimitExceeded = false;

// Amplifier
// *************************************************************************************************************************
#define amplifier_VCC 23       // VCC switch for amplifier

// Communication
// *************************************************************************************************************************
boolean wlanDisturbance = false;  // if WLAN is not ready for more than 3 cycles
int wlanReadyCount = 0;
boolean wlanReady = false;        // will be set by command from USB interface and reset by Arduino
boolean usbDisturbance = false;   // if USB interface is not ready

// Batteries
// **************************************************************************************************************************
#define battery12VProbe A0    // wire to battery via 1:3 voltage divider
int battery12VRawValue = 0;
float battery12VFinalValue = 0.0;
float battery12VLowerLimit = 11.1;
boolean battery12VLow = false;

#define battery5VProbe A2     // supervision of power bank, wire from Raspberry pi 5V via 4.7 kOhm resistor
int battery5VRawValue = 0;
float battery5VFinalValue = 0.0;
float battery5VLowerLimit = 4.5;
boolean battery5VLow = false;

#define Arduino5VProbe A3     // supervision of Arduino 5 V, wire from Arduino 5V via 4.7 kOhm resistor
int Arduino5VRawValue = 0;
float Arduino5VFinalValue = 0.0;
float Arduino5VLowerLimit = 4.5;
boolean Arduino5VLow = false;

// Motor
// *******************************************************************************************************************************
float motorStallLimit = 2.4;  // Stall limit
float motorStallLimitSlow = 1.6;
float motorStallLimitHalf = 2.0;
float motorStallLimitFull = 2.4;

// Motor right back
#define motor1Direction 32
#define motor1PWM 6
#define motor1CurrentProbe A15
int motor1RawValue = 0;
float motor1FinalValue = 0.0;
boolean motor1Stall = false;

// Motor left back
#define motor3Direction 30
#define motor3PWM 2
#define motor3CurrentProbe A13
int motor3RawValue = 0;
float motor3FinalValue = 0.0;
boolean motor3Stall = false;

// Motor control
// *******************************************************************************************************************************
boolean emergencyStop = false;
boolean forward             = 0;
boolean backward            = 1;
int originAngle = 0;
int startAngle = 0;
int turnedAngle = 0;
int turnAngle = 0;                      // Absolute angle to be turned to
int turnAngleRelative = 0;              // relative angle to be turned
int sequenceCounter90 = 0;              // Sequence counter for  90 degrees
int sequenceCounter180 = 0;             // Sequence counter for 180 degrees
int sequenceCounterTo = 0;              // Sequence counter for turn to
int alpha = 0;                          // small angle step to turn
int betas = 0;                          // wide angle step to turn
int i = 0;
int targetAngle = 0;                    // turn to target angle
float turningRadius = 44.0;             // Turning radius
float beta, bs = 0.0;                   // beta, b'
boolean forwardStopCommand  = true;     // Stop command
boolean forwardSlowCommand  = false;    // Forward slow command
boolean backwardSlowCommand  = false;   // Backward slow command
boolean forwardHalfCommand  = false;    // Forward half command
boolean forwardFullCommand  = false;    // Forward full command
boolean steeringLeftCommand = false;    // Steering left hand
boolean steeringRightCommand = false;   // Steering right hand
boolean turnSlow180LeftCommand = false; // Turn slow 180 degrees left
boolean turnSlow90LeftBackwardFirstCommand  = false; // Turn slow 90 degrees left backward first
boolean turnSlow90LeftForwardFirstCommand  = false;  // Turn slow 90 degrees left vorward first
boolean turnSlow90RightCommand = false; // Turn slow 90 degrees right
boolean turnSlowToCommand = false;      // Turn to an absolute angle
boolean turnSlowLeftToCommand = false;  // Turn left to an absolute angle
boolean turnSlowRightToCommand = false; // Turn left to an absolute angle
boolean turnFinished = true;


// align command
boolean alignCommand = false;
boolean alignTrue = false;
int stopDutyCycle           =   0;   //   0% of 256
int slowDutyCycle12         =  25;   //  10% of 256
int slowDutyCycle34         =  25;   //
int halfDutyCycle12         = 100;   //  40% of 256
int halfDutyCycle34         = 100;   //  
int fullDutyCycle12         = 150;   //  60% of 256
int fullDutyCycle34         = 150;   //  
int steeringRate            = 70;    //    % of DutyCycle
int steeringDutyCycle12     = 0;     // Calculated dutyCycle
int steeringDutyCycle34     = 0;     // Calculated dutyCycle


// Arrays and variables for alignment algorithm
float SrL [10] [6] = {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},   // Left: 0  1   2    3      4      5
  {2.0, 0.0, 0.0, 0.0, 0.0, 0.0},   //       x, y, dxi, dyi, dxi*dyi, dxi2
  {3.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {4.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {5.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {6.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {7.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {8.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {9.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {10.0, 0.0, 0.0, 0.0, 0.0, 0.0}
};

float SrR [10] [6] = {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},   // Right:
  {2.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {3.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {4.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {5.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {6.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {7.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {8.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {9.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  {10.0, 0.0, 0.0, 0.0, 0.0, 0.0}
};



float bL, bR, aL, aR  = 0.0;                          // function left and right f(x)= ax + b
int countL, countR;
boolean alignLTrue, alignRTrue = false;


boolean linearRegression(float Sr[10][6], float value, float a, float b, int count)
// not yet tested
{ // Linear regression calculation
  int i;                                                // *****************************
  float xm = 5.5, ym = 0.0;                             // mean values
  float Sdxidyi, Sdxi2 = 0.0;                           // sums over i

  for (i = 9; i > 0; i --) Sr[i][1] = Sr[i - 1][1];     // shift register + 1

  Sr[0][1] = value;                                     // put new value in to register

  if (count < 10) return false;
  else {
    count += 1;
    for (i = 0; i < 10; i ++)                             // calculate mean values
    { xm = xm + Sr[i][0];
      ym = ym + Sr[i][1];
    }
    xm = xm / 10.0;
    ym = ym / 10.0;

    for (i = 0; i < 10; i ++)                              // calculate table and sums
    { Sr[i][2] = Sr[i][0] - xm;                           // dxi
      Sr[i][3] = Sr[i][1] - ym;                           // dyi
      Sr[i][4] = Sr[i][2] * Sr[i][3];                     // dxi * dyi
      Sdxidyi  = Sdxidyi + Sr[i][4];                      // sum (dxi * dyi)
      Sr[i][5] = Sr[i][2] * Sr[i][2];                     // dxi2
      Sdxi2    = Sdxi2 + Sr[i][5];                        // sum (dxi2)
    }

    if (Sdxi2 > 0)                                       // calculate function paramters of f(x) = ax + b
    {
      a = Sdxidyi / Sdxi2;
      b = ym - a * xm;
      return true;
    }
    else return false;
  }
}

void MotorControl()
{
  if (emergencyStop || forwardStopCommand) {        // Emergency stop or manually stop
    analogWrite(motor1PWM, stopDutyCycle);          // *******************************
    analogWrite(motor3PWM, stopDutyCycle);
    forwardSlowCommand = false;
    forwardHalfCommand = false;
    forwardFullCommand = false;
    steeringLeftCommand = false;
    steeringRightCommand = false;
    turnSlow180LeftCommand = false;
    turnSlow90LeftBackwardFirstCommand = false;
    turnSlow90LeftForwardFirstCommand = false;
    turnSlow90RightCommand = false;
    turnSlowLeftToCommand = false;
    turnSlowRightToCommand = false;
    forwardStopCommand = true;
    alignCommand = false;
    turnFinished = true;
    startAngle = 0;
    Serial.print("S@Turn finished: ");          // remote status for turn finished to true
    Serial.println(turnFinished);
  }

  if (forwardSlowCommand) {                     // Forward slow manually
    digitalWrite(motor1Direction, forward);     // ************
    digitalWrite(motor3Direction, forward);
    steeringDutyCycle12 = slowDutyCycle12 + (slowDutyCycle12 * steeringRate / 100);
    steeringDutyCycle34 = slowDutyCycle34 + (slowDutyCycle34 * steeringRate / 100);
    if (steeringLeftCommand) {
      analogWrite(motor3PWM, slowDutyCycle34);  // Steering Left
      analogWrite(motor1PWM, steeringDutyCycle12);
      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);      
      }
      steeringservo.write(ServoLeft);
    }
    else {
      if (steeringRightCommand) {
        analogWrite(motor3PWM, steeringDutyCycle34); // Steering Right
        analogWrite(motor1PWM, slowDutyCycle12);
        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
        }
        steeringservo.write(ServoRight);
      }
      else {
        analogWrite(motor1PWM, slowDutyCycle12); // Steering ahead
        analogWrite(motor3PWM, slowDutyCycle34);
        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
        }
        steeringservo.write(ServoMiddle);
      }
    }
  }

  if (backwardSlowCommand) {                     // Backward slow manually
    digitalWrite(motor1Direction, backward);     // ************
    digitalWrite(motor3Direction, backward);
    steeringDutyCycle12 = slowDutyCycle12 + (slowDutyCycle12 * steeringRate / 100);
    steeringDutyCycle34 = slowDutyCycle34 + (slowDutyCycle34 * steeringRate / 100);
    if (steeringLeftCommand) {
      analogWrite(motor3PWM, slowDutyCycle34);   // Steering Left
      analogWrite(motor1PWM, steeringDutyCycle12);
      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
      }
      steeringservo.write(ServoLeft);
    }
    else {
      if (steeringRightCommand) {
        analogWrite(motor3PWM, steeringDutyCycle34);  // Steering Right
        analogWrite(motor1PWM, slowDutyCycle12);
        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
        }
        steeringservo.write(ServoRight);
      }
      else {
        analogWrite(motor1PWM, slowDutyCycle12);      // Steering ahead
        analogWrite(motor3PWM, slowDutyCycle34);
        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
        }
        steeringservo.write(ServoMiddle);
      }
    }
  }

  if (forwardHalfCommand) {                     // Forward half manually
    digitalWrite(motor1Direction, forward);     // ************
    digitalWrite(motor3Direction, forward);
    steeringDutyCycle12 = halfDutyCycle12 + (halfDutyCycle12 * steeringRate / 100);
    steeringDutyCycle34 = halfDutyCycle34 + (halfDutyCycle34 * steeringRate / 100);
    if (steeringLeftCommand) {
      analogWrite(motor3PWM, halfDutyCycle34);// Steering Left
      analogWrite(motor1PWM, steeringDutyCycle12);
      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
      }
      steeringservo.write(ServoLeft);
    }
    else {
      if (steeringRightCommand) {
        analogWrite(motor3PWM, steeringDutyCycle34);  // Steering Right
        analogWrite(motor1PWM, halfDutyCycle12);
        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
        }
        steeringservo.write(ServoRight);
      }
      else {
        analogWrite(motor3PWM, halfDutyCycle34);  // Steering ahead
        analogWrite(motor1PWM, halfDutyCycle12);
        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
        }
        steeringservo.write(ServoMiddle);
      }
    }
  }

  if (forwardFullCommand) {                     // Forward full manually
    digitalWrite(motor1Direction, forward);     // ************
    digitalWrite(motor3Direction, forward);
    steeringDutyCycle12 = fullDutyCycle12 + (fullDutyCycle12 * steeringRate / 100);
    steeringDutyCycle34 = fullDutyCycle34 + (fullDutyCycle34 * steeringRate / 100);
    if (steeringLeftCommand) {
      analogWrite(motor3PWM, fullDutyCycle34);// Steering Left
      analogWrite(motor1PWM, steeringDutyCycle12);
      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
      }
      steeringservo.write(ServoLeft);
    }
    else {
      if (steeringRightCommand) {
        analogWrite(motor3PWM, steeringDutyCycle34);  // Steering Right
        analogWrite(motor1PWM, fullDutyCycle12);
        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
        }
        steeringservo.write(ServoRight);
      }
      else {
        analogWrite(motor1PWM, fullDutyCycle12);      // Steering ahead
        analogWrite(motor3PWM, fullDutyCycle34);
        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
        }
        steeringservo.write(ServoMiddle);
      }
    }
  }

  if (turnSlowLeftToCommand) {                    // Supervision of turn left to (see project documentation)
                                                  // ****************************************
    // Input:  angle16          actual angle according to compass
    //         startAngle       angle        where the step has been startet 
    //         turnAngle        target angle
            
    // Output: turnedAngle      angle relative that has been turned
    //         motor stopDutyCycle, turn finished, turnslowLeftTo command = false

    if (((startAngle - angle16) < 0) && ((startAngle - angle16) > -20)) turnedAngle = 0; // Filter measuring faults (-19..-1)
    else turnedAngle = (3600 + startAngle - angle16) % 3600;                             // calculate turned angle

    if (((((turnAngle >= 0) && (turnAngle < 1800))                                                    // brown T1, T2
          || ((turnAngle >= 1800) && (turnAngle < 3600) && (startAngle >= 1800) && (startAngle < 3600)) // brown T3, T4
         )
         && (angle16 <= turnAngle)
         && (angle16 < startAngle)
         && (turnedAngle > 0)
        )
        || (((turnAngle >= 0) && (turnAngle < 900))                                                  // brown/red T1
            && (angle16 > startAngle)
            && (turnedAngle > 0)
           )
        || (((turnAngle >= 1800) && (turnAngle < 3600) && (startAngle >= 0) && (startAngle < 1800))  // blue T3, T4
            && (angle16 <= turnAngle)
            && (angle16 > startAngle)
            && (turnedAngle > 0)
           )
       ) {                                                                                          // Target reached
      analogWrite(motor1PWM, stopDutyCycle);
      analogWrite(motor3PWM, stopDutyCycle);
      turnSlowLeftToCommand = false;
      turnSlow90LeftBackwardFirstCommand = false;  // fast stop
      turnSlow90LeftForwardFirstCommand = false;   // fast stop
      //forwardStopCommand = true;
      turnFinished = true;                         // local status
    }
  }

  if (turnSlowRightToCommand) {                   // Supervision turn right to (see project documentation)
                                                  // *****************************************************
    // Input:  angle16          actual angle according to compass
    //         startAngle       angle        where the step has been startet 
    //         turnAngle        target angle
    
    // Output: turnedAngle      angle relative that has been turned
    //         motor stopDutyCycle, turn finished, turnslowRightTo command = false

    if (((angle16 - startAngle) < 0) && ((angle16 - startAngle) > -20)) turnedAngle = 0; // Filter measuring faults (-19..-1)
    else turnedAngle = (3600 + angle16 - startAngle) % 3600;                             // calculate turned angle

    if (((((turnAngle >= 1800) && (turnAngle < 3600))                                             // violet T3, T4
          || ((turnAngle >= 0) && (turnAngle < 1800) && (startAngle >= 0) && (startAngle < 1800)) // violet T1, T2
         )
         && ((angle16 >= turnAngle)
             && (angle16 > startAngle))
         && (turnedAngle > 0)
        )
        || (((turnAngle >= 2700) && (turnAngle < 3600))                                           // violet/yellow T4
            && (angle16 < startAngle)
            && (turnedAngle > 0)
           )

        || (((turnAngle >= 0) && (turnAngle < 1800) && (startAngle >= 1800) && (startAngle < 3600)) // green T1, T2
            && (angle16 >= turnAngle)
            && (angle16 < startAngle)
            && (turnedAngle > 0)
           )
       ) {                                                                                         // Target reached
      analogWrite(motor1PWM, stopDutyCycle);      // fast stop
      analogWrite(motor3PWM, stopDutyCycle);      // fast stop
      turnSlowRightToCommand = false;
      turnFinished = true;                        // local status

    }
  }
  if (turnSlow90RightCommand) {                   // Turn right 90 degrees sequence backward first (see project documentation)
                                                  // **********************************************************

    if (sequenceCounter90 == 1) {                 // 1------------------------------------------------------
      steeringDutyCycle12 = slowDutyCycle12 + (slowDutyCycle12 * steeringRate / 100);
      steeringDutyCycle34 = slowDutyCycle34 + (slowDutyCycle34 * steeringRate / 100);
      beta = 90.0;                                                                                       // Robot deviation from target (degrees)
      bs = sqrt(2 * pow(turningRadius, 2) - 2 * pow(turningRadius, 2) * cos(beta * PI / 180)) / 2;       // b' (cm)
      betas = int(10 * acos(- ((pow(bs, 2) - 2 * pow(turningRadius, 2)) / (2 * pow(turningRadius, 2)))) * 180 / PI); // beta' (degrees *10)
      alpha = int((int(10 * beta) - betas) / 2);                                                         // alpha (degrees *10)

      originAngle = angle16;                      // origin angle

      startAngle = angle16;                       // store start angle
      turnAngleRelative = alpha;                  // relative turn angle
      turnAngle = (angle16 + alpha) % 3600;       // final angle

      digitalWrite(motor3Direction, backward);    // back left alpha
      digitalWrite(motor1Direction, backward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
      }             
      steeringservo.write(ServoLeft);             // Servo left

      analogWrite(motor1PWM, steeringDutyCycle12);// run
      analogWrite(motor3PWM, slowDutyCycle34);

      turnFinished = false;
      turnSlowRightToCommand = true;              // supervise turn until turn finished

      Serial.print("S@Turn finished: ");          // remote status for turn finished to false
      Serial.println(turnFinished);
      sequenceCounter90 = 2;                      // next step
    }

    if (sequenceCounter90 == 2) {                  // 2------------------------------------------------------
      if (turnFinished) sequenceCounter90 = 3;     // wait for turn
    }

    if (sequenceCounter90 == 3) {                  // 3------------------------------------------------------
      startAngle = angle16;                        // store start angle
      turnAngleRelative = betas;                   // relative turn angle
      turnAngle = (angle16 + betas) % 3600;        // final angle

      digitalWrite(motor3Direction, forward);      // forward right beta'
      digitalWrite(motor1Direction, forward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
      }                   
      steeringservo.write(ServoRight);             // Servo right

      analogWrite(motor1PWM, slowDutyCycle12);     // run
      analogWrite(motor3PWM, steeringDutyCycle34);

      turnFinished = false;
      turnSlowRightToCommand = true;               // supervise turn until turn finished

      sequenceCounter90 = 4;                       // next step
    }

    if (sequenceCounter90 == 4) {                  // 4------------------------------------------------------
      if (turnFinished) sequenceCounter90 = 5;     // wait for turn
    }

    if (sequenceCounter90 == 5) {                  // 5------------------------------------------------------
      startAngle = angle16;                        // store start angle
      alpha = (originAngle + 900 - angle16) % 3600;// calculated rest angle
      turnAngleRelative = alpha;                   // relative turn angle
      turnAngle = (angle16 + alpha) % 3600;        // final angle

      digitalWrite(motor3Direction, backward);     // back left alpha
      digitalWrite(motor1Direction, backward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
      }               
      steeringservo.write(ServoLeft);              // Servo left

      analogWrite(motor1PWM, steeringDutyCycle12); // run
      analogWrite(motor3PWM, slowDutyCycle34);

      turnFinished = false;
      turnSlowRightToCommand = true;               // supervise turn until turn finished

      sequenceCounter90 = 6;                       // next step
    }

    if (sequenceCounter90 == 6) {                  // 6------------------------------------------------------
      if (turnFinished) {                          // wait for turn

        turnSlow90RightCommand = false;            // target reached
        sequenceCounter90 = 0;

        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);         
        }             
        steeringservo.write(ServoMiddle);          // Servo middle

        Serial.print("S@Turn finished: ");         // remote status after full 90 degrees only
        Serial.println(turnFinished);

      }
    }
  }

  if (turnSlow90LeftBackwardFirstCommand) {       // Turn left 90 degrees sequence backward first (see project documentation)
    // ************************************************************************
    if (sequenceCounter90 = 1) {                  // 1

      steeringDutyCycle12 = slowDutyCycle12 + (slowDutyCycle12 * steeringRate / 100);
      steeringDutyCycle34 = slowDutyCycle34 + (slowDutyCycle34 * steeringRate / 100);
      beta = 90.0;                                                                                       // Robot deviation from target (degrees)
      bs = sqrt(2 * pow(turningRadius, 2) - 2 * pow(turningRadius, 2) * cos(beta * PI / 180)) / 2;       // b' (cm)
      betas = int(10 * acos(- ((pow(bs, 2) - 2 * pow(turningRadius, 2)) / (2 * pow(turningRadius, 2)))) * 180 / PI); // beta' (degrees *10)
      alpha = int((int(10 * beta) - betas) / 2);                                                         // alpha (degrees *10)

      originAngle = angle16;                      // origin angle
      startAngle = angle16;                       // store start angle
      turnAngleRelative = alpha;                  // relative turn angle
      turnAngle = (3600 + angle16 - alpha) % 3600;// final angle

      digitalWrite(motor3Direction, backward);    // back right alpha
      digitalWrite(motor1Direction, backward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
      }              
      steeringservo.write(ServoRight);            // Servo right

      analogWrite(motor1PWM, slowDutyCycle12);    // run
      analogWrite(motor3PWM, steeringDutyCycle34);

      turnFinished = false;
      turnSlowLeftToCommand = true;               // supervise turn until turn finished

      Serial.print("S@Turn finished: ");          // remote status for turn finished to false
      Serial.println(turnFinished);
      sequenceCounter90 = 2;                      // next step
    }

    if (sequenceCounter90 == 2) {                  // 2------------------------------------------------------
      if (turnFinished) sequenceCounter90 = 3;     // wait for turn
    }

    if (sequenceCounter90 == 3) {                  // 3------------------------------------------------------
      startAngle = angle16;                        // store start angle
      turnAngleRelative = betas;                   // relative turn angle
      turnAngle = (3600 + angle16 - betas) % 3600; // final angle

      digitalWrite(motor3Direction, forward);      // forward left beta'
      digitalWrite(motor1Direction, forward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
      }               
      steeringservo.write(ServoLeft);              // Servo left

      analogWrite(motor1PWM, steeringDutyCycle12); // run
      analogWrite(motor3PWM, slowDutyCycle34);

      turnFinished = false;
      turnSlowLeftToCommand = true;                // supervise turn until turn finished

      sequenceCounter90 = 4;                       // next step
    }

    if (sequenceCounter90 == 4) {                  // 4------------------------------------------------------
      if (turnFinished) sequenceCounter90 = 5;     // wait for turn
    }

    if (sequenceCounter90 == 5) {                  // 5------------------------------------------------------
      startAngle = angle16;                        // store start angle
      alpha = (originAngle - 900 - angle16) % 3600;// calculated rest angle
      turnAngleRelative = alpha;                   // relative turn angle
      turnAngle = (3600 + angle16 - alpha) % 3600;        // final angle

      digitalWrite(motor3Direction, backward);     // back right alpha
      digitalWrite(motor1Direction, backward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
      }               
      steeringservo.write(ServoRight);             // Servo right

      analogWrite(motor1PWM, slowDutyCycle12);     // run
      analogWrite(motor3PWM, steeringDutyCycle34);

      turnFinished = false;
      turnSlowLeftToCommand = true;                // supervise turn until turn finished

      sequenceCounter90 = 6;                       // next step
    }

    if (sequenceCounter90 == 6) {                  // 6------------------------------------------------------
      if (turnFinished) {                          // wait for turn

        turnSlow90LeftBackwardFirstCommand = false;// target reached
        sequenceCounter90 = 0;

        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
        }             
        steeringservo.write(ServoMiddle);         // Servo middle
      }
    }
  }
  if (turnSlow90LeftForwardFirstCommand) {        // Turn left 90 degrees sequence forward first (see project documentation)
    // ***********************************************************************
    if (sequenceCounter90 = 1) {                  // 1

      steeringDutyCycle12 = slowDutyCycle12 + (slowDutyCycle12 * steeringRate / 100);
      steeringDutyCycle34 = slowDutyCycle34 + (slowDutyCycle34 * steeringRate / 100);
      beta = 90.0;                                                                                       // Robot deviation from target (degrees)
      bs = sqrt(2 * pow(turningRadius, 2) - 2 * pow(turningRadius, 2) * cos(beta * PI / 180)) / 2;       // b' (cm)
      betas = int(10 * acos(- ((pow(bs, 2) - 2 * pow(turningRadius, 2)) / (2 * pow(turningRadius, 2)))) * 180 / PI); // beta' (degrees *10)
      alpha = int((int(10 * beta) - betas) / 2);                                                         // alpha (degrees *10)

      originAngle = angle16;                      // origin angle
      startAngle = angle16;                       // store start angle
      turnAngleRelative = alpha;                  // relative turn angle
      turnAngle = (3600 + angle16 - alpha) % 3600;// final angle

      digitalWrite(motor3Direction, forward);     // forward left alpha
      digitalWrite(motor1Direction, forward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
      }               
      steeringservo.write(ServoLeft);             // Servo left

      analogWrite(motor1PWM, steeringDutyCycle12);// run
      analogWrite(motor3PWM, slowDutyCycle34);

      turnFinished = false;
      turnSlowLeftToCommand = true;               // supervise turn until turn finished

      Serial.print("S@Turn finished: ");          // remote status for turn finished to false
      Serial.println(turnFinished);
      sequenceCounter90 = 2;                      // next step
    }

    if (sequenceCounter90 == 2) {                  // 2------------------------------------------------------
      if (turnFinished) sequenceCounter90 = 3;     // wait for turn
    }

    if (sequenceCounter90 == 3) {                  // 3------------------------------------------------------
      startAngle = angle16;                        // store start angle
      turnAngleRelative = betas;                   // relative turn angle
      turnAngle = (3600 + angle16 - betas) % 3600; // final angle

      digitalWrite(motor3Direction, backward);     // backward right beta'
      digitalWrite(motor1Direction, backward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
      }              
      steeringservo.write(ServoRight);             // Servo right

      analogWrite(motor1PWM, slowDutyCycle12);     // run
      analogWrite(motor3PWM, steeringDutyCycle34);

      turnFinished = false;
      turnSlowLeftToCommand = true;                // supervise turn until turn finished

      sequenceCounter90 = 4;                       // next step
    }

    if (sequenceCounter90 == 4) {                  // 4------------------------------------------------------
      if (turnFinished) sequenceCounter90 = 5;     // wait for turn
    }

    if (sequenceCounter90 == 5) {                  // 5------------------------------------------------------
      startAngle = angle16;                        // store start angle
      alpha = (originAngle - 900 - angle16) % 3600;// calculated rest angle
      turnAngleRelative = alpha;                   // relative turn angle
      turnAngle = (3600 + angle16 - alpha) % 3600; // final angle

      digitalWrite(motor3Direction, forward);      // forward left alpha
      digitalWrite(motor1Direction, forward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
      }              
      steeringservo.write(ServoLeft);              // Servo left

      analogWrite(motor1PWM, steeringDutyCycle12); // run
      analogWrite(motor3PWM, slowDutyCycle34);

      turnFinished = false;
      turnSlowLeftToCommand = true;                // supervise turn until turn finished

      sequenceCounter90 = 6;                       // next step
    }

    if (sequenceCounter90 == 6) {                  // 6------------------------------------------------------
      if (turnFinished) {                          // wait for turn

        turnSlow90LeftForwardFirstCommand = false; // target reached
        sequenceCounter90 = 0;

        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
        }             
        steeringservo.write(ServoMiddle);          // Servo middle
      }

    }
  }
  if (turnSlow180LeftCommand) {                    // Turn left 180 degrees sequence (see project documentation)
                                                   // ***********************************************************
    if (sequenceCounter180 = 1) {                    // 1----------------------------------------------------------
      sequenceCounter90 = 1;
      turnSlow90LeftForwardFirstCommand = true;      // Turn 90 degrees left forward first

      sequenceCounter180 = 3;                        // next step
    }

    if (sequenceCounter180 == 3) {                   // 3----------------------------------------------------------
      if (turnSlow90LeftForwardFirstCommand == false) {
        sequenceCounter90 = 1;
        turnSlow90LeftBackwardFirstCommand = true;    // Turn 90 degrees left backward first

        sequenceCounter180 = 4;                       // next step

      }
    }
    if (sequenceCounter180 == 4) {                   // 4----------------------------------------------------------
      if (turnSlow90LeftBackwardFirstCommand == false) {
        turnSlow180LeftCommand = false;

        sequenceCounter180 = 0;
      }
    }
  }

  if (turnSlowToCommand) {                         // Turn to absolute angle sequence (see project documentation)
                                                   // ***********************************************************
                                                   // construction area!
    if (sequenceCounterTo == 1) {                   // 1----------------------------------------------------------
      turnFinished = false;
      Serial.print("S@Turn finished: ");              // remote status for turn finished to false
      Serial.println(turnFinished);
      steeringDutyCycle12 = slowDutyCycle12 + (slowDutyCycle12 * steeringRate / 100);
      steeringDutyCycle34 = slowDutyCycle34 + (slowDutyCycle34 * steeringRate / 100); 
              
      startAngle = angle16;                                                         // store start angle
      targetAngle = turnAngle;                                                      // store target angle
      i = (targetAngle - angle16);                                                  // relative angle -3600 .. 0 .. + 3600 degrees
                                                                                    // turn angle relative -1 .. -1749 turn left; +1 .. +1850 turn right
                                                                                    // ***************************************************************
      turnAngleRelative = i;                                                        // - 1   .. - 1749 --> turn left; +1 .. +1850 --> turn right                                                       
      if (i <= -1750) turnAngleRelative = i + 3600;                                 // -1750 .. - 3600 --> turn angle relative right = + 1850 .. 0
      if (i > 1850) turnAngleRelative = i - 3600;                                   // +1851 .. + 3600 --> turn angle relative left  = - 1749 .. 0      
      i = turnAngleRelative;                                                        // save turn angle relative
      
      if (i < -1 || i > 1) {                                                        // do nothing in range (-1 .. +1) degrees
        
        if (i > 0) {                                                                // turn right to
                                                                                    // *************
          if (  i > 1000) {                                                         // turn first 90 degrees right
            sequenceCounterTo = 2;        
          }
          else {
            sequenceCounterTo = 3;       
          }        
        }
        else {                                                                      // turn left to
                                                                                    // ************ 
          if (i < 1000) {                                                           // turn first 90 degrees left
            sequenceCounterTo = 20;      
          }
          else {
            sequenceCounterTo = 21;       
          }         
        }
      }
    }
    if (sequenceCounterTo == 2) {                  // 2----------------------------------------------------------
                                                   
      sequenceCounter90 = 1;
      turnSlow90RightCommand = true;                // Turn 90 degrees right
      sequenceCounterTo = 3;
    }

    if (sequenceCounterTo == 3) {                  // 3----------------------------------------------------------
      if (turnSlow90RightCommand == false){

        beta = abs(i);                              // Robot deviation from target (degrees)
        bs = sqrt(2 * pow(turningRadius, 2) - 2 * pow(turningRadius, 2) * cos(beta * PI / 180)) / 2;       // b' (cm)
        betas = int(10 * acos(- ((pow(bs, 2) - 2 * pow(turningRadius, 2)) / (2 * pow(turningRadius, 2)))) * 180 / PI); // beta' (degrees *10)
        alpha = int((int(10 * beta) - betas) / 2);                                                         // alpha (degrees *10)
  
        originAngle = angle16;                      // origin angle
  
        startAngle = angle16;                       // store start angle
        turnAngleRelative = alpha;                  // relative turn angle
        turnAngle = (angle16 + alpha) % 3600;       // final angle
  
        digitalWrite(motor3Direction, backward);    // back left alpha
        digitalWrite(motor1Direction, backward);
  
        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
        }              
        steeringservo.write(ServoLeft);             // Servo left
  
        analogWrite(motor1PWM, steeringDutyCycle12);// run
        analogWrite(motor3PWM, slowDutyCycle34);
  
        turnFinished = false;
        turnSlowRightToCommand = true;              // supervise turn until turn finished
                  
        sequenceCounterTo = 4;                      // next step
      }
    }

    if (sequenceCounterTo == 4) {                  // 4----------------------------------------------------------
      if (turnFinished) sequenceCounterTo = 5;      // wait for turn
    }
    
    if (sequenceCounterTo == 5) {                  // 5----------------------------------------------------------

      startAngle = angle16;                        // store start angle
      turnAngleRelative = betas;                   // relative turn angle
      turnAngle = (angle16 + betas) % 3600;        // final angle

      digitalWrite(motor3Direction, forward);      // forward right beta'
      digitalWrite(motor1Direction, forward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
      }                
      steeringservo.write(ServoRight);             // Servo right

      analogWrite(motor1PWM, slowDutyCycle12);     // run
      analogWrite(motor3PWM, steeringDutyCycle34);

      turnFinished = false;
      turnSlowRightToCommand = true;               // supervise turn until turn finished
      sequenceCounterTo = 6;                       // next step
    }

    if (sequenceCounterTo == 6) {                  // 6----------------------------------------------------------
      if (turnFinished) sequenceCounterTo = 7;      // wait for turn
    }

    if (sequenceCounterTo == 7) {                  // 7----------------------------------------------------------
      startAngle = angle16;                        // store start angle
      alpha = (targetAngle - angle16) % 3600;      // calculated rest angle
      turnAngleRelative = alpha;                   // relative turn angle
      turnAngle = (angle16 + alpha) % 3600;        // final angle

      digitalWrite(motor3Direction, backward);     // back left alpha
      digitalWrite(motor1Direction, backward);

      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
      }               
      steeringservo.write(ServoLeft);              // Servo left

      analogWrite(motor1PWM, steeringDutyCycle12); // run
      analogWrite(motor3PWM, slowDutyCycle34);

      turnFinished = false;
      turnSlowRightToCommand = true;               // supervise turn until turn finished
      sequenceCounterTo = 8;                       // next step 
    }

    if (sequenceCounterTo == 8) {                  // 8----------------------------------------------------------
      if (turnFinished) sequenceCounterTo = 30;     // wait for turn
    }

    

    if (sequenceCounterTo == 20) {                 // 20----------------------------------------------------------

      sequenceCounter90 = 1;
      turnSlow90LeftForwardFirstCommand = true;     // Turn 90 degrees left forward first
        

    }

    if (sequenceCounterTo == 21) {                 // 21----------------------------------------------------------
                                                    // turn to rest angle left
      turnAngleRelative = abs(turnAngleRelative);
      turnAngle = (3600 + targetAngle + 10) % 3600; // cycle tolerance
      turnAngleRelative = (3600 + turnAngleRelative - 10) % 3600;
      turnSlowLeftToCommand = true;
        

    }

    if (sequenceCounterTo == 22) {                 // 22----------------------------------------------------------
                                                    // wait for turn   

    }
    
    if (sequenceCounterTo == 30) {                 // 30----------------------------------------------------------
      
      turnSlowToCommand = false;
      sequenceCounterTo == 0;
      if (not steeringservo.attached()){
        steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
      }                
      steeringservo.write(ServoMiddle);             // Servo middle   
      Serial.print("S@Turn finished: ");            // remote status after automatic turn only
      Serial.println(turnFinished);
    }
  }
}


// Encoders
// *******************************************************************************************************************************

#define RT_PHASE_A (PIND & B00010000)
#define RT_PHASE_B (PIND & B00001000)

static volatile int16_t encDeltaLt, encDeltaRt;
static int16_t lastLt = 0, lastRt = 0;
int encLt = 0, encRt = 0;

ISR( TIMER2_COMPA_vect )
{
  int16_t val, diff;

  val = 0;
  if ( RT_PHASE_A )
    val = 3;
  if ( RT_PHASE_B )
    val ^= 1; // convert gray to binary
  diff = lastRt - val; // difference last - new
  if ( diff & 1 ) { // bit 0 = value (1)
    lastRt = val; // store new as next last
    encDeltaRt += (diff & 2) - 1; // bit 1 = direction (+/-)
  }
  return;
}

void QuadratureEncoderInit(void)
{
  int16_t val;
  cli();
  //set timer2 interrupt at 2kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR2A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 and CS20 bits for 64 prescaler
  TCCR2B |= (1 << CS21) | (1 << CS20);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  val = 0;
  if (RT_PHASE_A)
    val = 3;
  if (RT_PHASE_B)
    val ^= 1;
  lastRt = val;
  encDeltaRt = 0;
  encRt = 0;
  sei();
  return;
}

int16_t QuadratureEncoderReadRt( void ) // read single step encoders
{
  int16_t val;
  cli();
  val = encDeltaRt;
  encDeltaRt = 0;
  sei();
  return val; // counts since last call
}

// US sensors
// ***************************************************************************************************************

#define distancebRightEcho 48
#define distancebRightTrig 49
#define distancebRightLimit 20
int distancebRightPulseTime = 0;
int distancebRightCm = 0;
boolean distancebRightObstruction = false;

#define distancebLeftEcho 50
#define distancebLeftTrig 51
#define distancebLeftLimit 20
int distancebLeftPulseTime = 0;
int distancebLeftCm = 0;
boolean distancebLeftObstruction = false;


#define distanceFrontEcho 38
#define distanceFrontTrig 39
#define distanceFrontLimit 20
int distanceFrontPulseTime = 0;
int distanceFrontCm = 0;
int distanceFrontReg[4] = {0, 0, 0, 0};
boolean distanceFrontObstruction = false;

#define distanceDownLimit 15
int distanceDownCm = 0;
boolean distanceDownObstruction = false;

#define distanceUpEcho 34
#define distanceUpTrig 35
#define distanceUpLimit 210
int distanceUpPulseTime = 0;
int distanceUpCm = 0;
boolean distanceUpDoor = false;

// IR sensors
// ***************************************************************************************************************
#define distanceDownProbe A4
int distanceDownRawValue = 0;

// Others
// *************************************************************************************************************************
int j;
#define ledPin 13                 // LED for heart beat
#define baud 38400                // Transmission speed for serial
int testPoint1 = 0;
int testPoint2 = 0;
int testPoint3 = 0;

// Commands
// ***********************************************************************************************************************

const int bSize = 25;
int ByteCount;
char Buffer[bSize];             // Serial buffer
char Command[23];               // Arbitrary Value for command size
String CommandString = "";
String CommandStringS = "";     // for mirroring commands

int SerialParser(void)
{
  //  One command per line.
  //  Command Format: "up to 23 letter command + \n"
  //  read up to X chars until EOT - in this case "\n"
  ByteCount = -1;
  ByteCount =  Serial.readBytesUntil('\n', Buffer, bSize);
  if (ByteCount  > 0) strcpy(Command, strtok(Buffer, "\n"));
  memset(Buffer, 0, sizeof(Buffer));   // Clear contents of Buffer
  return ByteCount;
}

//  Setup
// ***********************************************************************************************************************
// ***********************************************************************************************************************
void setup()
{
  Serial.begin(baud);
  Serial.setTimeout(10);

  pinMode(ledPin, OUTPUT);
  pinMode(distancebLeftTrig, OUTPUT);
  pinMode(distancebRightTrig, OUTPUT);
  pinMode(distanceFrontTrig, OUTPUT);
  pinMode(distanceUpTrig, OUTPUT);
  pinMode(motor1Direction, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor3Direction, OUTPUT);
  pinMode(motor3PWM, OUTPUT);
  pinMode(ServoPWM, OUTPUT);
  pinMode(CMPS11_VCC, OUTPUT);
  pinMode(amplifier_VCC, OUTPUT);

  // Servo initiation
  // ************************************************************************************************************

  steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
  steeringservo.write(ServoMiddle);            // middle position

  // CMPS11 startup
  // ************************************************************************************************************
  digitalWrite(CMPS11_VCC, 1);
  // Wait for CMPS has been restarted
  delay(2000);
  Wire.begin();

  // Encoder Software initiation
  // ************************************************************************************************************
  QuadratureEncoderInit();

  // send static values
  // ************************************************************************************************************
  while (not Serial) {
    Serial.end();
    Serial.begin(baud);
    Serial.setTimeout(10);
    delay(2000);
  }
  Serial.print ("MV@Version: V ");
  Serial.println(ARDUINO);
  Serial.print("MV@Battery 12V: LL ");
  Serial.println(battery12VLowerLimit);
  Serial.print("MV@Battery 5V: LL ");
  Serial.println(battery5VLowerLimit);
  Serial.print("MV@Arduino 5V: LL ");
  Serial.println(Arduino5VLowerLimit);
  Serial.print("MV@Roll: UL ");
  Serial.println(UrollLimit, DEC);
  Serial.print("MV@Roll: LL ");
  Serial.println(LrollLimit, DEC);
  Serial.print("MV@Pitch: UL ");
  Serial.println(UpitchLimit, DEC);
  Serial.print("MV@Pitch: LL ");
  Serial.println(LpitchLimit, DEC);
  Serial.print("MV@Distance bleft: LL ");
  Serial.println(distancebLeftLimit);
  Serial.print("MV@Distance bright: LL ");
  Serial.println(distancebRightLimit);
  Serial.print("MV@Distance front: LL ");
  Serial.println(distanceFrontLimit);
  Serial.print("MV@Distance up: LL ");
  Serial.println(distanceUpLimit);
  Serial.print("MV@Distance down: UL ");
  Serial.println(distanceDownLimit);
  Serial.print("S@Turn finished: ");          // remote status for turn finished to true
  Serial.println(turnFinished);
}

// Loop
// ***********************************************************************************************************************************
// ***********************************************************************************************************************************
void loop()
{

  // LED heart beat and cycle time control
  // *********************************************************************************************************************************
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin by XOR
  // regulation of cycle time because of intermediateAngle calculation  total cycletime should be <= 500 ms

  if (turnFinished == true) {
    // Read battery probe and check limit if no turn is running (shorten cycle time)
    // *********************************************************************************************************************************
    battery12VRawValue = analogRead(battery12VProbe);
    battery5VRawValue = analogRead(battery5VProbe);
    Arduino5VRawValue = analogRead(Arduino5VProbe);

    battery12VFinalValue = battery12VRawValue * 15.3 / 1023.0;
    battery12VLow = (battery12VFinalValue < battery12VLowerLimit);

    battery5VFinalValue = battery5VRawValue * 5.0 / 1023.0;
    battery5VLow = (battery5VFinalValue < battery5VLowerLimit);

    Arduino5VFinalValue = Arduino5VRawValue * 5.0 / 1023.0;
    Arduino5VLow = (Arduino5VFinalValue < Arduino5VLowerLimit);
  }

  // Read motor current probe and check limit
  // ************************************************************************************************************************************
  motor1RawValue = analogRead(motor1CurrentProbe);
  motor3RawValue = analogRead(motor3CurrentProbe);

  motor1FinalValue = motor1RawValue * 5.0 / 1023.0;
  motor1Stall = (motor1FinalValue > motorStallLimit);

  motor3FinalValue = motor3RawValue * 5.0 / 1023.0;
  motor3Stall = (motor3FinalValue > motorStallLimit);

  // Read CMPS11 values
  // ***********************************************************************************************************************************

  Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
  Wire.write(ANGLE_8);                     //Sends the register we wish to start reading from
  Wire.endTransmission();

  // Request 5 bytes from the CMPS11
  // this will give us the 8 bit bearing, both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS11_ADDRESS, 5);

  while (Wire.available() < 5);       // Wait for all bytes to come back

  angle8 = Wire.read();               // Read back the 5 bytes
  high_byte = Wire.read();
  low_byte = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();
  UpitchLimitExceeded = (pitch > UpitchLimit); // checking pitch limit
  LpitchLimitExceeded = (pitch < LpitchLimit); // checking pitch limit
  UrollLimitExceeded = (roll > UrollLimit);    // checking roll limit
  LrollLimitExceeded = (roll < LrollLimit);    // checking roll limit
  angle16 = high_byte;                       // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;
  if ((angle16 >= 3595) || (angle16 <= 5)) angle16 = 0;               // Hysteresis +- 0.5 degree arount 0 degrees
  //****************************************************************************************************************

  if (turnFinished == true) {
    // calculate sliding intermediate value over the last 10 values if no turn is running (shorten cycle time)
    for (i = 9; i > 0; i --) {
      angleVectorList[i][0] = angleVectorList[i - 1][0];                // shift register right
      angleVectorList[i][1] = angleVectorList[1 - 1][1];
    }

    angleVectorList[0][0] = sin(angle16 * PI / 1800);                   // write new vector in to register
    angleVectorList[0][1] = cos(angle16 * PI / 1800);

    vectora1 = 0.0;
    vectora2 = 0.0;
    for (i = 0; i <= 9; i ++) {
      vectora1 = vectora1 + angleVectorList[i][0];
      vectora2 = vectora2 + angleVectorList[i][1];
    }

    vectorLength = sqrt((vectora1 * vectora1) + (vectora2 * vectora2));
    cosinus = vectora2 / vectorLength;
    angle = acos(cosinus);
    if (vectora1 > 0) intermediateAngle = angle * 1800 / PI;
    else intermediateAngle = 3600 - (angle * 1800 / PI);
    if (intermediateAngle == 3600) intermediateAngle = 0;


    // Read distances to obstructions (US-015 contains a temperature compensation)
    // *************************************************************************************************************************************

    digitalWrite(distancebLeftTrig, LOW);                               // back Left
    delayMicroseconds(5);                                               // *********
    digitalWrite(distancebLeftTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(distancebLeftTrig, LOW);
    distancebLeftPulseTime = pulseIn(distancebLeftEcho, HIGH, 23600);
    if (distancebLeftPulseTime > 60) {                                  // disturbance filter
      distancebLeftCm = distancebLeftPulseTime / 59;
    }
    else {
      distancebLeftCm = 210;                                            // out of range
    }
    distancebLeftObstruction = (distancebLeftCm < distancebLeftLimit);// Obstruction detected back left

    digitalWrite(distancebRightTrig, LOW);                              // back Right
    delayMicroseconds(5);                                               // **********
    digitalWrite(distancebRightTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(distancebRightTrig, LOW);
    distancebRightPulseTime = pulseIn(distancebRightEcho, HIGH, 23600);
    if (distancebRightPulseTime > 60) {                                 // disturbance filter
      distancebRightCm = distancebRightPulseTime / 59;
    }
    else {
      distancebRightCm = 210;                                           // out of range
    }
    distancebRightObstruction = (distancebRightCm < distancebRightLimit);// Obstruction detected back right

    // Alignment
    // *********
    alignLTrue = false;
    alignRTrue = false;
    if (alignCommand && (distancebLeftCm < 10 || distancebLeftCm > 200) && (distancebRightCm < 10 || distancebRightCm > 200)) {
      // no wall in range
      alignCommand = false;
      countL = 1;
      countR = 1;
    }
    else {
      if (alignCommand && distancebLeftCm >= 10 && distancebLeftCm <= 200 && distancebRightCm >= 10 && distancebRightCm <= 200) {
        // two walls in range
        alignLTrue = linearRegression(SrL, distancebLeftCm, aL, bL, countL);
        alignRTrue = linearRegression(SrR, distancebRightCm, aR, bR, countR);

        // select the wall in shorter distance
        if (distancebLeftCm <= distancebRightCm)  alignRTrue = false;
        else                                      alignLTrue = false;
      }
      else {
        if (alignCommand && distancebLeftCm >= 10 && distancebLeftCm <= 200) {
          // left wall only in range
          alignLTrue = linearRegression(SrL, distancebLeftCm, aL, bL, countL);
          countR = 1;
        }
        else {
          if (alignCommand) {
            // right wall only in range
            alignRTrue = linearRegression(SrR, distancebRightCm, aR, bR, countR);
            countL = 1;
          }
        }
      }
    }
    if (alignCommand && alignLTrue) {
      // Align based on left sensor
      if (aL > - 0.2 && aL < 0.2) {
        // steer ahead and end align
        steeringLeftCommand = false;
        steeringRightCommand = false;
        alignCommand = false;
      }
      if (aL > 0) {
        // Steering right
        steeringLeftCommand = false;
        steeringRightCommand = true;
      }
      else {
        // Steering left
        steeringRightCommand = false;
        steeringLeftCommand = true;
      }
    }
    if (alignCommand && alignRTrue) {
      // Align based on right sensor
      if (aR > - 0.2 && aR < 0.2) {
        // steer ahead and end align
        steeringLeftCommand = false;
        steeringRightCommand = false;
        alignCommand = false;
      }
      if (aR > 0) {
        // Steering right
        steeringLeftCommand = false;
        steeringRightCommand = true;
      }
      else {
        // Steering left
        steeringRightCommand = false;
        steeringLeftCommand = true;
      }
    }
    // Front
    // *****
    for (j = 0; j < 2; j++) {                                             // 2 measures per cycle
      digitalWrite(distanceFrontTrig, LOW);
      delayMicroseconds(20);
      digitalWrite(distanceFrontTrig, HIGH);
      delayMicroseconds(10);
      digitalWrite(distanceFrontTrig, LOW);
      distanceFrontPulseTime = pulseIn(distanceFrontEcho, HIGH, 47200);   // Range limited to 2 x 400cm (timeout delivers 0)
      if (distanceFrontPulseTime > 60) {                                  // filter for timeout
        distanceFrontCm = distanceFrontPulseTime / 59;                    // 340m/s = 29.4 us/cm for two ways

        for (i = 3; i > 0; i --) {
          distanceFrontReg[i] = distanceFrontReg[i - 1];                  // shift register one step right
        }
        distanceFrontReg[0] = distanceFrontCm;                            // put new value to register

      }
    }

    distanceFrontCm = 0;
    for (i = 0; i < 3; i ++) {
      distanceFrontCm = distanceFrontCm + distanceFrontReg[i];
    }
    distanceFrontCm = int(distanceFrontCm / 3);                           // mean value from last 3 valid values


    distanceFrontObstruction = (distanceFrontCm < distanceFrontLimit);    // Obstruction detected front side


    digitalWrite(distanceUpTrig, LOW);                                  // Up
    delayMicroseconds(5);                                               // **
    digitalWrite(distanceUpTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(distanceUpTrig, LOW);
    distanceUpPulseTime = pulseIn(distanceUpEcho, HIGH, 35400);         // Range limited to 2 x 300cm
    if (distanceUpPulseTime > 60) {                                     // disturbance filter
      distanceUpCm = distanceUpPulseTime / 59;
    }
    else {
      distanceUpCm = 310;                                               // out of range
    }
    distanceUpDoor = (distanceUpCm < distanceUpLimit);                  // Door passing

    // Down
    // ****

    distanceDownRawValue = analogRead(distanceDownProbe);               // Read IR sensor
    distanceDownCm = (4800 / (distanceDownRawValue - 20));
    distanceDownObstruction = (distanceDownCm > distanceDownLimit);     // Obstruction detected
  }

  // Build and execute emergency stop
  // *************************************************************************************************************************************

  if (emergencyStop == false) {                                       // keep emergency stop stored until manually reset
    emergencyStop =  battery12VLow || battery5VLow  || Arduino5VLow
                     //                     || distanceFrontObstruction
                     || motor1Stall  || motor3Stall
                     || UpitchLimitExceeded  || LpitchLimitExceeded || UrollLimitExceeded  || LrollLimitExceeded

                     //                     || distanceDownObstruction
                     //                     || wlanDisturbance
                     || usbDisturbance;
  }


  // Read encoders
  // *************************************************************************************************************************************
  // calculation of driving distance to be done at Raspberry pi 2
  encRt += QuadratureEncoderReadRt();

  // Motor control
  // *************************************************************************************************************************************

  MotorControl();
  if (steeringservo.attached()&& forwardStopCommand == true) {
    delay(100);
    steeringservo.detach();
  }


  // Supervision of W-LAN Communication
  // *************************************************************************************************************************************

  if (wlanReady == false) {
    wlanReadyCount += 1;                                                            // no wlanReady received since last cycle
    if (wlanReadyCount < 3) wlanDisturbance = false;                                // W-LAN ok
    else {
      wlanDisturbance = true;                                                       // W-LAN Disturbance
      wlanReadyCount = 3;
    }
  }
  else {
    // wlanReady ok
    wlanReady = false;
    wlanReadyCount = 0;
  }

  // check serial interface
  // *************************************************************************************************************************************
  if (Serial) {
    usbDisturbance = false;                                            // USB ok
    if (turnFinished == true) {
      // Send values to USB interface if no turn command is running (shorten cycle time)
      // *************************************************************************************************************************************

      Serial.print("MV@EncLt: V ");
      Serial.println(encLt, DEC);
      Serial.print("MV@EncRt: V ");
      Serial.println(encRt, DEC);

      Serial.print("MV@Battery 12V: V ");
      Serial.println(battery12VFinalValue);
      Serial.print("MV@Battery 12V: LL_Exceeded ");
      Serial.println(battery12VLow);

      Serial.print("MV@Battery 5V: V ");
      Serial.println(battery5VFinalValue);
      Serial.print("MV@Battery 5V: LL_Exceeded ");
      Serial.println(battery5VLow);

      Serial.print("MV@Arduino 5V: V ");
      Serial.println(Arduino5VFinalValue);
      Serial.print("MV@Arduino 5V: LL_Exceeded ");
      Serial.println(Arduino5VLow);

      Serial.print("MV@Distance bleft: V ");
      Serial.println(distancebLeftCm);
      Serial.print("MV@Distance bleft: LL_Exceeded ");
      Serial.println(distancebLeftObstruction);

      Serial.print("MV@Distance bright: V ");
      Serial.println(distancebRightCm);
      Serial.print("MV@Distance bright: LL_Exceeded ");
      Serial.println(distancebRightObstruction);

      Serial.print("MV@Distance front: V ");
      Serial.println(distanceFrontCm);
      Serial.print("MV@Distance front: LL_Exceeded ");
      Serial.println(distanceFrontObstruction);

      Serial.print("MV@Distance up: V ");
      Serial.println(distanceUpCm);

      //      Serial.print("MV@Distance down: V ");
      //      Serial.println(distanceDownCm);
      //      Serial.print("MV@Distance down: UL_Exceeded ");
      //      Serial.println(distanceDownObstruction);

      Serial.print("S@Forward slow: ");
      Serial.println(forwardSlowCommand);
      Serial.print("S@Backward slow: ");
      Serial.println(backwardSlowCommand);
      Serial.print("S@Forward half: ");
      Serial.println(forwardHalfCommand);
      Serial.print("S@Forward full: ");
      Serial.println(forwardFullCommand);
      Serial.print("S@Steering left: ");
      Serial.println(steeringLeftCommand);
      Serial.print("S@Steering right: ");
      Serial.println(steeringRightCommand);
      Serial.print("S@Turn slow 180 left: ");
      Serial.println(turnSlow180LeftCommand);
      Serial.print("S@Turn slow 90 left B: ");
      Serial.println(turnSlow90LeftBackwardFirstCommand);
      Serial.print("S@Turn slow 90 left V: ");
      Serial.println(turnSlow90LeftForwardFirstCommand);
      Serial.print("S@Turn slow 90 right: ");
      Serial.println(turnSlow90RightCommand);
      Serial.print("S@Align: ");
      Serial.println(alignCommand);

      Serial.print("MV@Motor1 current: UL ");
      Serial.println(motorStallLimit);
      Serial.print("MV@Motor3 current: UL ");
      Serial.println(motorStallLimit);

      Serial.print("MV@Motor1 current: V ");
      Serial.println (motor1FinalValue);
      Serial.print("MV@Motor1 current: UL_Exceeded ");
      Serial.println(motor1Stall);

      Serial.print("MV@Motor3 current: V ");
      Serial.println(motor3FinalValue);
      Serial.print("MV@Motor3 current: UL_Exceeded ");
      Serial.println(motor3Stall);

      Serial.print("MV@Roll: V ");
      Serial.println(roll, DEC);
      Serial.print("MV@Roll: UL_Exceeded ");
      Serial.println(UrollLimitExceeded);
      Serial.print("MV@Roll: LL_Exceeded ");
      Serial.println(LrollLimitExceeded);

      Serial.print("MV@Pitch: V ");
      Serial.println(pitch, DEC);
      Serial.print("MV@Pitch: UL_Exceeded ");
      Serial.println(UpitchLimitExceeded);
      Serial.print("MV@Pitch: LL_Exceeded ");
      Serial.println(LpitchLimitExceeded);

      Serial.print("MV@Actual angle: V ");
      Serial.print(angle16 / 10, DEC);
      Serial.print(".");
      Serial.println(angle16 % 10, DEC);

      Serial.print("MV@Smoothed angle: V ");
      Serial.print(intermediateAngle / 10, DEC);
      Serial.print(".");
      Serial.println(intermediateAngle % 10, DEC);

      Serial.print("S@Compass: ");
      Serial.println(digitalRead(CMPS11_VCC));

      Serial.print("S@Amplifier: ");
      Serial.println(digitalRead(amplifier_VCC));

      Serial.print("S@W-LAN disturbance: ");
      Serial.println(wlanDisturbance);
    }
    Serial.print("MV@Turned angle: V ");
    Serial.println(turnedAngle);

    Serial.print("S@Emergency stop: ");
    Serial.println(emergencyStop);

    Serial.print("S@Stop: ");
    Serial.println(forwardStopCommand);

    //    Serial.print("MV@Distance down raw value: "); // For test reasons only
    //    Serial.println(distanceDownRawValue);         // For test reasons only
    //    Serial.print("MV@Distance down pulse time: ");// For test reasons only
    //    Serial.println(distanceDownPulseTime);        // For test reasons only

    //    Serial.print("S@TestPoint1: ");               // for testing set e.g. testPoint1 = 1; and read it in the datastream on your PC
    //    Serial.println(testPoint1);
    //    Serial.print("S@TestPoint2: ");
    //    Serial.println(testPoint2);
    //    Serial.print("S@TestPoint3: ");
    //    Serial.println(testPoint3);


    // Get commands from USB interface
    // *************************************************************************************************************************************
    while (SerialParser() > 0) {
      CommandString = Command;
      CommandStringS = CommandString;
      // It is allowed to change between driving commands directly, no need for a stop command between driving commands

      if (CommandString.startsWith("Stop")) {
        forwardStopCommand = true;
        if (not steeringservo.attached()){
          steeringservo.attach(ServoPWM);         
        }
        steeringservo.write(ServoMiddle);
      }

      if (CommandString.startsWith("Forward slow")) {
        forwardStopCommand = false; forwardSlowCommand = true; forwardHalfCommand = false; forwardFullCommand = false;
        turnSlow180LeftCommand = false;
        turnSlow90LeftBackwardFirstCommand = false; turnSlow90LeftForwardFirstCommand = false; turnSlow90RightCommand = false;
        turnSlowLeftToCommand = false; turnSlowRightToCommand = false;
        motorStallLimit = motorStallLimitSlow;
      }
      if (CommandString.startsWith("Forward half")) {
        forwardStopCommand = false; forwardHalfCommand = true; forwardSlowCommand = false; forwardFullCommand = false;
        turnSlow180LeftCommand = false;
        turnSlow90LeftBackwardFirstCommand = false; turnSlow90LeftForwardFirstCommand = false; turnSlow90RightCommand = false;
        turnSlowLeftToCommand = false; turnSlowRightToCommand = false;
        motorStallLimit = motorStallLimitHalf;
      }
      if (CommandString.startsWith("Forward full")) {
        forwardStopCommand = false; forwardFullCommand = true; forwardSlowCommand = false; forwardHalfCommand = false;
        turnSlow180LeftCommand = false;
        turnSlow90LeftBackwardFirstCommand = false; turnSlow90LeftForwardFirstCommand = false; turnSlow90RightCommand = false;
        turnSlowLeftToCommand = false; turnSlowRightToCommand = false;
        motorStallLimit = motorStallLimitFull;
      }
      if (CommandString.startsWith("Align")) {
        alignCommand = true;
        countL = 1; // reset counter for shiftregister
        countR = 1;
        steeringLeftCommand = false;
        steeringRightCommand = false;
      }

      if (CommandString.startsWith("Steering left"))  {
        steeringLeftCommand = true;
        steeringRightCommand = false;
        alignCommand = false;
      }
      if (CommandString.startsWith("Steering right")) {
        steeringLeftCommand = false;
        steeringRightCommand = true;
        alignCommand = false;
      }
      if (CommandString.startsWith("Steering ahead")) {
        steeringLeftCommand = false;
        steeringRightCommand = false;
        alignCommand = false;
      }
      if (CommandString.startsWith("Turn slow 180 left")) {
        startAngle = angle16;                     // store start angle
        turnAngleRelative = 1790;
        turnAngle = (3600 + angle16 + 1790) % 3600;
        turnSlow180LeftCommand = true;

        turnSlow90LeftBackwardFirstCommand = true; turnSlow90LeftForwardFirstCommand = false; turnSlow90RightCommand = false;
        turnSlowRightToCommand = false; turnSlowLeftToCommand = false;
        forwardStopCommand = false; forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
        steeringLeftCommand = false; steeringRightCommand = false; alignCommand = false;
        motorStallLimit = motorStallLimitSlow;
        sequenceCounter180 = 1;
      }

      if (CommandString.startsWith("Turn slow 90 left")) {
        turnSlow90LeftBackwardFirstCommand = true;

        turnSlow90LeftForwardFirstCommand = false; turnSlow180LeftCommand = false; turnSlow90RightCommand = false;
        turnSlowLeftToCommand = false; turnSlowRightToCommand = false;
        forwardStopCommand = false; forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
        steeringLeftCommand = false; steeringRightCommand = false; alignCommand = false;
        motorStallLimit = motorStallLimitSlow;
        sequenceCounter90 = 1;
      }
      if (CommandString.startsWith("Turn slow 90 right")) {
        turnSlow90RightCommand = true;

        turnSlow90LeftBackwardFirstCommand = false; turnSlow90LeftForwardFirstCommand = false; turnSlow180LeftCommand = false;
        turnSlowLeftToCommand = false; turnSlowRightToCommand = false;
        forwardStopCommand = false; forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
        steeringLeftCommand = false; steeringRightCommand = false; alignCommand = false;
        motorStallLimit = motorStallLimitSlow;
        sequenceCounter90 = 1;
      }
      if (CommandString.startsWith("Turn slow to: ")) {
        CommandString.replace("Turn slow to: ", "");
        turnAngle = CommandString.toInt();                                            // Absolute angle to be turned to
        turnSlowToCommand = true;

        turnSlow90LeftBackwardFirstCommand = false; turnSlow90LeftForwardFirstCommand = false; turnSlow90RightCommand = false; turnSlow180LeftCommand = false;
        turnSlowLeftToCommand = false; turnSlowRightToCommand = false;
        forwardStopCommand = false; forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
        steeringLeftCommand = false; steeringRightCommand = false; alignCommand = false;

        motorStallLimit = motorStallLimitSlow;
        sequenceCounterTo = 1;
      }

      if (CommandString.startsWith("Wlan ready")) wlanReady = true;                   // Live beat of W-LAN communication

      if (CommandString.startsWith("Encoder reset")) {
        encLt = 0;                                                                    // Encoder reset
        encRt = 0;
      }
      if (CommandString.startsWith("Amplifier: ")) {                                  // Amplifier 0/1
        if (CommandString.substring(11) == "0") digitalWrite(amplifier_VCC, 0);
        else digitalWrite(amplifier_VCC, 1);
      }
      if (CommandStringS != "") {
        Serial.print("I@Command: ");
        Serial.println(CommandStringS);
        CommandStringS = "";
      }
    }
    Serial.print("I@Stepcount: ");
    Serial.println(sequenceCounter90);
    Serial.print("I@b': ");
    Serial.println(bs);
    Serial.print("I@beta': ");
    Serial.println(betas);
    Serial.print("I@alpha: ");
    Serial.println(alpha);

  }
  else {
    usbDisturbance = true;                                                        // USB disturbance
    Serial.end();
    Serial.begin(baud);
    Serial.setTimeout(10);
    delay(2000);                                                                  // wait for next trial
  }

}

