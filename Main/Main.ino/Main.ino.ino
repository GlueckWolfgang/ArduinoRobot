
//****************************************************************************************************************************************************
// *** Arduino robot program
// *** Version: 2016.04.21
// *** Developer: Wolfgang Glück
// ***
// *** Supported hardware:
// ***   - Rover5 chassis with 4 motors and encoders                (2 encoders used only)
// ***   - Rover5 chassis with 2 motors and encoders                (optional, just remove the unused parts of code)
// ***   - Motor control board (MCB) for 4 motors and encoders
// ***   - Motor control Board (MCB) for 2 motors and encoders      (optional)
// ***   - Arduino mega                                             (Spider)
// ***   - Compass                                                  (CMPS11 via 10cm I2C bus, using internal pullups by default)
// ***                                                              (For calibration use separated program CPMS11Calibration)
// ***   - 6 x Ultra-sonic sensor                                   (US-015)
// ***   - 1 x Infra red Sensor                                     (Sharp 2Y0A21)
// ***   - 7,2 V Battery for MCB and drives                         (6x 1,2 V NiMH Akku type D 10'000 mAh, same battery pack as for 9,6 V)
// ***   - 9,6 V Battery for Arduino and MCB                        (8x 1,2 V NiMH Akku type D 10'000 mAh)
// ***   - Battery holder for 8 type D cells
// ***   - 5 V / Gnd     for MCB is connected to Arduino 5 V / Gnd
// ***   - 5 V   Battery for Raspberry pi                           (VOLTCRAFT 20800 mAh PB-19 POWERBANK)
// ***   - 5 Resistors as voltage divider                           (4,7 kOhm 0,5 W)
// ***   - Audio amplifier                                          (KEMO M031N 3.5 W 4.5 - 12 V, operated at 9,6 V Battery)
// ***   - Laudspeaker                                              (LSM-28M/NT-B, 8 OHM, 2W)
// ***   - FET Transitor for switching audio amlifier on/off        (IRL3803PbF)
// ***   - Potentiometer for volume regulation                      (10 kOhm)
// ***
// ***   - Raspberry pi 2
// ***   - SD card 32GB
// ***   - camera
// ***   - USB Sound card                                           (Speedlink VIGO USB Soundcard)
// ***   - W-LAN adapter                                            (EW-7612UAN V2)
// ***   - USB cable from Raspberry pi to Arduino
// ***
// *** Functions:
// ***   - (ok) One time compass calibration. For this, code adjustments have to be done manually (see comments under setup)
// ***   - (ok) Read and supervise analog values like battery voltage and motor current
// ***   - (ok) Read compass values via I2C
// ***   - (ok) smoothe compass value for a calm display
// ***   - (ok) Supervise roll and pitch
// ***   - (ok) Read encoders
// ***   - (ok) Read obstruction distances
// ***   - (ok) Send analog and digital values to USB interface
// ***   - (ok) Get and execute driving commands via USB interface (transition beween driving commands directly is allowed)
// ***          (Stop, ForwardSlow, ForwardHalf, ForwardFull, SteeringAhead, SteeringLeft, SteeringRight in combination with Forward Commands)
// ***          (TurnSlow45Left, TurnSlow45Right, TurnSlow90Left, TurnSlow90Right, turnSlowTo value)
// ***   - (ok) get switch on/off command for audio amplifier
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

// Compass
// *************************************************************************************************************************
#include <math.h>
#include <Wire.h>
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
#define battery9VProbe A0    // wire to battery via 1:1 voltage divider with 4.7kOhm resistors
int battery9VRawValue = 0;
float battery9VFinalValue = 0.0;
float battery9VLowerLimit = 8.0;
boolean battery9VLow = false;

#define battery7VProbe A1    // wire to battery via 1:1 voltage divider with 4.7 kOhm resistors
int battery7VRawValue = 0;
float battery7VFinalValue = 0.0;
float battery7VLowerLimit = 6.0;
boolean battery7VLow = false;

#define battery5VProbe A2     // supervision of power bank, wire from Raspberry pi 5V via 4.7 kOhm resistor
int battery5VRawValue = 0;
float battery5VFinalValue = 0.0;
float battery5VLowerLimit = 4.5;
boolean battery5VLow = false;

#define Arduino5VProbe A3     // supervision of Arduino 5 V
int Arduino5VRawValue = 0;
float Arduino5VFinalValue = 0.0;
float Arduino5VLowerLimit = 4.5;
boolean Arduino5VLow = false;

// Motors
// *******************************************************************************************************************************
float motorStallLimit = 1.0;  // Stall limit for all motors
float motorStallLimitSlow = 1.0;
float motorStallLimitHalf = 1.0;
float motorStallLimitFull = 1.2;

// Motor right front
#define motor1Direction 32
#define motor1PWM 6
#define motor1CurrentProbe A15
int motor1RawValue = 0;
float motor1FinalValue = 0.0;
boolean motor1Stall = false;

//Motor right back
#define motor2Direction 33
#define motor2PWM 7
#define motor2CurrentProbe A14
int motor2RawValue = 0;
float motor2FinalValue = 0.0;
boolean motor2Stall = false;

#define motor3Direction 30
#define motor3PWM 2
#define motor3CurrentProbe A13
int motor3RawValue = 0;
float motor3FinalValue = 0.0;
boolean motor3Stall = false;

#define motor4Direction 31
#define motor4PWM 3
#define motor4CurrentProbe A12
int motor4RawValue = 0;
float motor4FinalValue = 0.0;
boolean motor4Stall = false;

// Motor control
// *******************************************************************************************************************************
boolean emergencyStop = false;
boolean forward             = 0;
boolean backward            = 1;
int startAngle = 0;
int turnedAngle = 0;
int turnAngle = 0;                      // Absolute angle to be turned to
int turnAngleRelative = 0;              // relative angle to be turned
boolean forwardStopCommand  = true;     // Stop command
boolean forwardSlowCommand  = false;    // Forward slow command
boolean forwardHalfCommand  = false;    // Forward half command
boolean forwardFullCommand  = false;    // Forward full command
boolean steeringLeftCommand = false;    // Steering left hand
boolean steeringRightCommand = false;   // Steering right hand
boolean turnSlow45LeftCommand  = false; // Turn slow 45 degrees left
boolean turnSlow45RightCommand = false; // Turn slow 45 degrees right
boolean turnSlow90LeftCommand  = false; // Turn slow 90 degrees left
boolean turnSlow90RightCommand = false; // Turn slow 90 degrees right
boolean turnSlowLeftToCommand = false;  // Turn left to an absolute angle
boolean turnSlowRightToCommand = false; // Turn left to an absolute angle
boolean turnFinished = true;

// align command
boolean alignCommand = false;
boolean alignTrue = false;
int stopDutyCycle           =   0;   //   0% of 256
int slowDutyCycle           =  64;   //  25% of 256
int halfDutyCycle           = 128;   //  50% of 256
int fullDutyCycle           = 213;   //  80% of 256, contains reserve for steering
int steeringRate            = 20;    //    % of DutyCycle
int steeringDutyCycle       = 0;     // Calculated dutyCycle

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
                      {10.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                      
float SrR [10] [6] = {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},   // Right: 
                      {2.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {3.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {4.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {5.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {6.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {7.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {8.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {9.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                      {10.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
                      

       
float bL, bR, aL, aR  = 0.0;                          // function left and right f(x)= ax + b
int countL, countR;
boolean alignLTrue, alignRTrue = false;


boolean linearRegression(float Sr[10][6], float value, float a, float b, int count)
                                                      // not yet tested
{                                                     // Linear regression calculation 
int i;                                                // *****************************
float xm = 5.5, ym = 0.0;                             // mean values
float Sdxidyi, Sdxi2 = 0.0;                           // sums over i
         
for (i = 9; i > 0; i --) Sr[i][1] = Sr[i - 1][1];     // shift register + 1

Sr[0][1] = value;                                     // put new value in to register

if (count < 10) return false;
else{
  count += 1;                                                    
  for (i = 0; i < 10; i ++)                             // calculate mean values                             
   {xm = xm + Sr[i][0];
    ym = ym + Sr[i][1];
   }
  xm = xm/10.0;
  ym = ym/10.0;
                                                         
  for (i = 0; i < 10; i ++)                              // calculate table and sums
    {Sr[i][2] = Sr[i][0] - xm;                           // dxi
     Sr[i][3] = Sr[i][1] - ym;                           // dyi
     Sr[i][4] = Sr[i][2] * Sr[i][3];                     // dxi * dyi
     Sdxidyi  = Sdxidyi + Sr[i][4];                      // sum (dxi * dyi)
     Sr[i][5] = Sr[i][2] * Sr[i][2];                     // dxi2
     Sdxi2    = Sdxi2 + Sr[i][5];                        // sum (dxi2)
    }
    
    if (Sdxi2 > 0)                                       // calculate funktion paramters of f(x) = ax + b
      {
        a = Sdxidyi/Sdxi2;
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
    analogWrite(motor2PWM, stopDutyCycle);
    analogWrite(motor3PWM, stopDutyCycle);
    analogWrite(motor4PWM, stopDutyCycle);
    forwardSlowCommand = false;
    forwardHalfCommand = false;
    forwardFullCommand = false;
    steeringLeftCommand = false;
    steeringRightCommand = false;
    turnSlow45LeftCommand = false;
    turnSlow45RightCommand = false;
    turnSlow90LeftCommand = false;
    turnSlow90RightCommand = false;
    turnSlowLeftToCommand = false;
    turnSlowRightToCommand = false;
    forwardStopCommand = true;
    alignCommand = false;
    turnFinished = true;
    startAngle = 0;
  }
  
  if (forwardSlowCommand) {                     // Forward slow
    digitalWrite(motor1Direction, forward);     // ************
    digitalWrite(motor2Direction, forward);
    digitalWrite(motor3Direction, forward);
    digitalWrite(motor4Direction, forward);
    steeringDutyCycle = slowDutyCycle + (slowDutyCycle * steeringRate / 100);
    if (steeringLeftCommand) {
      analogWrite(motor1PWM, steeringDutyCycle);// Steering Left
      analogWrite(motor2PWM, steeringDutyCycle);
      analogWrite(motor3PWM, slowDutyCycle);
      analogWrite(motor4PWM, slowDutyCycle);
    }
    else {
      if (steeringRightCommand) {
        analogWrite(motor1PWM, slowDutyCycle);  // Steering Right
        analogWrite(motor2PWM, slowDutyCycle);
        analogWrite(motor3PWM, steeringDutyCycle);
        analogWrite(motor4PWM, steeringDutyCycle);
      }
      else {
        analogWrite(motor1PWM, slowDutyCycle);  // Steering ahead
        analogWrite(motor2PWM, slowDutyCycle);
        analogWrite(motor3PWM, slowDutyCycle);
        analogWrite(motor4PWM, slowDutyCycle);
      }
    }
  }

  if (forwardHalfCommand) {                     // Forward half
    digitalWrite(motor1Direction, forward);     // ************
    digitalWrite(motor2Direction, forward);
    digitalWrite(motor3Direction, forward);
    digitalWrite(motor4Direction, forward);
    steeringDutyCycle = halfDutyCycle + (halfDutyCycle * steeringRate / 100);
    if (steeringLeftCommand) {
      analogWrite(motor1PWM, steeringDutyCycle);// Steering Left
      analogWrite(motor2PWM, steeringDutyCycle);
      analogWrite(motor3PWM, halfDutyCycle);
      analogWrite(motor4PWM, halfDutyCycle);
    }
    else {
      if (steeringRightCommand) {
        analogWrite(motor1PWM, halfDutyCycle);  // Steering Right
        analogWrite(motor2PWM, halfDutyCycle);
        analogWrite(motor3PWM, steeringDutyCycle);
        analogWrite(motor4PWM, steeringDutyCycle);
      }
      else {
        analogWrite(motor1PWM, halfDutyCycle);  // Steering ahead
        analogWrite(motor2PWM, halfDutyCycle);
        analogWrite(motor3PWM, halfDutyCycle);
        analogWrite(motor4PWM, halfDutyCycle);
      }
    }
  }

  if (forwardFullCommand) {                     // Forward full
    digitalWrite(motor1Direction, forward);     // ************
    digitalWrite(motor2Direction, forward);
    digitalWrite(motor3Direction, forward);
    digitalWrite(motor4Direction, forward);
    steeringDutyCycle = fullDutyCycle + (fullDutyCycle * steeringRate / 100);
    if (steeringLeftCommand) {
      analogWrite(motor1PWM, steeringDutyCycle);// Steering Left
      analogWrite(motor2PWM, steeringDutyCycle);
      analogWrite(motor3PWM, fullDutyCycle);
      analogWrite(motor4PWM, fullDutyCycle);
    }
    else {
      if (steeringRightCommand) {
        analogWrite(motor1PWM, fullDutyCycle);  // Steering Right
        analogWrite(motor2PWM, fullDutyCycle);
        analogWrite(motor3PWM, steeringDutyCycle);
        analogWrite(motor4PWM, steeringDutyCycle);
      }
      else {
        analogWrite(motor1PWM, fullDutyCycle);  // Steering ahead
        analogWrite(motor2PWM, fullDutyCycle);
        analogWrite(motor3PWM, fullDutyCycle);
        analogWrite(motor4PWM, fullDutyCycle);
      }
    }
  }

  if (turnSlowLeftToCommand) {                    // Turn left to (see project documentation)
                                                  // ************  
    digitalWrite(motor1Direction, backward);
    digitalWrite(motor2Direction, backward);
    digitalWrite(motor3Direction, forward);
    digitalWrite(motor4Direction, forward);
    analogWrite(motor1PWM, slowDutyCycle);
    analogWrite(motor2PWM, slowDutyCycle);
    analogWrite(motor3PWM, slowDutyCycle);
    analogWrite(motor4PWM, slowDutyCycle);
    
    if (((startAngle - angle16) < 0) && ((startAngle - angle16) > -10)) turnedAngle = 0; // Filter measuring faults (-9..-1)
    else turnedAngle = (3600 + startAngle - angle16) % 3600;                             // calculate turned angle
                                                                                           
    if (((((turnAngle >= 0) && (turnAngle < 1800))                                                    // brown T1, T2
          ||((turnAngle >= 1800) && (turnAngle < 3600) && (startAngle >= 1800) && (startAngle < 3600))// brown T3, T4
         )                                                                                  
         && (angle16 <= turnAngle)                                                           
         && (angle16 < startAngle)
         && (turnedAngle > 0)
        )
        ||(((turnAngle >= 0) && (turnAngle < 900))                                                   // brown/red T1                                                                                                                                       
         && (angle16 > startAngle)
         && (turnedAngle > 0)
        )     
        ||(((turnAngle >= 1800) && (turnAngle < 3600) && (startAngle >= 0) && (startAngle < 1800))   // blue T3, T4  
           && (angle16 <= turnAngle) 
           && (angle16 > startAngle)
           && (turnedAngle > 0)
          )
        ){                                                                                           // Target reached  
          analogWrite(motor1PWM, stopDutyCycle);
          analogWrite(motor2PWM, stopDutyCycle);
          analogWrite(motor3PWM, stopDutyCycle);
          analogWrite(motor4PWM, stopDutyCycle);
          turnSlowLeftToCommand = false;
          turnSlow45LeftCommand = false;
          turnSlow90LeftCommand = false;
          forwardStopCommand = true;
          turnFinished = true;                        // local status
          Serial.print("S@Turn finished: ");          // remote status
          Serial.println(turnFinished);
         }
  }

  if (turnSlowRightToCommand) {                   // Turn right to (see project documentation)
                                                  // *************   
    digitalWrite(motor1Direction, forward);
    digitalWrite(motor2Direction, forward);
    digitalWrite(motor3Direction, backward);
    digitalWrite(motor4Direction, backward);
    analogWrite(motor1PWM, slowDutyCycle);
    analogWrite(motor2PWM, slowDutyCycle);
    analogWrite(motor3PWM, slowDutyCycle);
    analogWrite(motor4PWM, slowDutyCycle);
    
    if (((angle16 - startAngle) < 0) && ((angle16 - startAngle) > -10)) turnedAngle = 0; // Filter measuring faults (-9..-1)
    else turnedAngle = (3600 + angle16 - startAngle) % 3600;                             // calculate turned angle

    if (((((turnAngle >= 1800) && (turnAngle < 3600))                                             // violet T3, T4
          ||((turnAngle >= 0) && (turnAngle < 1800) && (startAngle >= 0) && (startAngle < 1800))  // violet T1, T2
         )                                                                               
          && ((angle16 >= turnAngle) 
          && (angle16 > startAngle))
          && (turnedAngle > 0)
        )
        ||(((turnAngle >= 2700) && (turnAngle < 3600))                                            // violet/yellow T4                                                                             
          && (angle16 < startAngle)
          && (turnedAngle > 0)
        )
      
        ||(((turnAngle >= 0) && (turnAngle < 1800) && (startAngle >= 1800) && (startAngle < 3600))// green T1, T2
          && (angle16 >= turnAngle) 
          && (angle16 < startAngle)
          && (turnedAngle > 0)
          )
      ){                                                                                          // Target reached    
        analogWrite(motor1PWM, stopDutyCycle);
        analogWrite(motor2PWM, stopDutyCycle);
        analogWrite(motor3PWM, stopDutyCycle);
        analogWrite(motor4PWM, stopDutyCycle);
        turnSlowRightToCommand = false;
        turnSlow45RightCommand = false;
        turnSlow90RightCommand = false;
        forwardStopCommand = true;
        turnFinished = true;                        // local status
        Serial.print("S@Turn finished: ");          // remote status
        Serial.println(turnFinished);
    }
  }
}

// Encoders
// *******************************************************************************************************************************

#define encoderRightA 29
#define encoderRightB 28
#define encoderLeftA 27
#define encoderLeftB 26

#define LT_PHASE_A digitalRead(encoderLeftA)
#define LT_PHASE_B digitalRead(encoderLeftB)
#define RT_PHASE_A digitalRead(encoderRightA)
#define RT_PHASE_B digitalRead(encoderRightB)
static volatile int16_t encDeltaLt, encDeltaRt;
static int16_t lastLt = 0, lastRt = 0;
int encLt = 0, encRt = 0;

ISR( TIMER2_COMPA_vect )
{
  int16_t val, diff;

  val = 0;
  if ( LT_PHASE_A )
    val = 3;
  if ( LT_PHASE_B )
    val ^= 1; // convert gray to binary
  diff = lastLt - val; // difference last - new
  if ( diff & 1 ) { // bit 0 = value (1)
    lastLt = val; // store new as next last
    encDeltaLt += (diff & 2) - 1; // bit 1 = direction (+/-)
  }

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
  TIMSK2 |= (1 << OCIE2A);
  val = 0;
  if (LT_PHASE_A)
    val = 3;
  if (LT_PHASE_B)
    val ^= 1;
  lastLt = val;
  encDeltaLt = 0;

  val = 0;
  if (RT_PHASE_A)
    val = 3;
  if (RT_PHASE_B)
    val ^= 1;
  lastRt = val;
  encDeltaRt = 0;

  encLt = 0;
  encRt = 0;
  sei();
  return;
}

int16_t QuadratureEncoderReadLt( void ) // read single step encoders (works correct)
{
  int16_t val;
  cli();
  val = encDeltaLt;
  encDeltaLt = 0;
  sei();
  return val; // counts since last call
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
#define distancefRightEcho 36
#define distancefRightTrig 37
#define distancefRightLimit 20
int distancefRightPulseTime = 0;
int distancefRightCm = 0;
boolean distancefRightObstruction = false;

#define distancefLeftEcho 42
#define distancefLeftTrig 43
#define distancefLeftLimit 20
int distancefLeftPulseTime = 0;
int distancefLeftCm = 0;
boolean distancefLeftObstruction = false;

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
int i;
#define ledPin 13                 // LED for heart beat
#define baud 38400                // Transmission speed for serial
int testPoint1 = 0;
int testPoint2 = 0;
int testPoint3 = 0;

// Commands
// ***********************************************************************************************************************

const int bSize = 20;
int ByteCount;
char Buffer[bSize];             // Serial buffer
char Command[18];               // Arbitrary Value for command size
String CommandString = "";
String CommandStringS = "";     // for mirroring commands

void SerialParser(void)
{
  //  One command per line.
  //  Command Format: "up to 18 Letter command <\n>"
  //  count will  be below Zero on a timeout.
  //  read up to X chars or until EOT - in this case "\n"
  ByteCount = -1;
  ByteCount =  Serial.readBytesUntil('\n', Buffer, bSize);
  if (ByteCount  > 0) strcpy(Command, strtok(Buffer, "\n"));
  memset(Buffer, 0, sizeof(Buffer));   // Clear contents of Buffer
  //Serial.flush();
}

//  Setup
// ***********************************************************************************************************************
// ***********************************************************************************************************************
void setup()
{
  Serial.begin(baud);
  Serial.setTimeout(10);
  
  pinMode(ledPin, OUTPUT);
  pinMode(distancefRightTrig, OUTPUT);
  pinMode(distancefLeftTrig, OUTPUT);
  pinMode(distancebLeftTrig, OUTPUT);
  pinMode(distancebRightTrig, OUTPUT);
  pinMode(distanceFrontTrig, OUTPUT);
  pinMode(distanceUpTrig, OUTPUT);
  pinMode(motor1Direction, OUTPUT);
  pinMode(motor2Direction, OUTPUT);
  pinMode(motor3Direction, OUTPUT);
  pinMode(motor4Direction, OUTPUT);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor3PWM, OUTPUT);
  pinMode(motor4PWM, OUTPUT);
  pinMode(CMPS11_VCC, OUTPUT);
  pinMode(amplifier_VCC, OUTPUT);

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
  Serial.print("MV@Battery 9V: LL ");
  Serial.println(battery9VLowerLimit);
  Serial.print("MV@Battery 7V: LL ");
  Serial.println(battery7VLowerLimit);  
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
  Serial.print("MV@Distance fleft: LL ");
  Serial.println(distancefLeftLimit);
  Serial.print("MV@Distance fright: LL ");
  Serial.println(distancefRightLimit);
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
    battery9VRawValue = analogRead(battery9VProbe);
    battery7VRawValue = analogRead(battery7VProbe);
    battery5VRawValue = analogRead(battery5VProbe);
    Arduino5VRawValue = analogRead(Arduino5VProbe);
  
    battery9VFinalValue = battery9VRawValue * 10.0 / 1023.0;
    battery9VLow = (battery9VFinalValue < battery9VLowerLimit);
  
    battery7VFinalValue = battery7VRawValue * 10.0 / 1023.0;
    battery7VLow = (battery7VFinalValue < battery7VLowerLimit);
  
  
    battery5VFinalValue = battery5VRawValue * 5.0 / 1023.0;
    battery5VLow = (battery5VFinalValue < battery5VLowerLimit);
  
    Arduino5VFinalValue = Arduino5VRawValue * 5.0 / 1023.0;
    Arduino5VLow = (Arduino5VFinalValue < Arduino5VLowerLimit);
  }

  // Read motor current probe and check limit
  // ************************************************************************************************************************************
  motor1RawValue = analogRead(motor1CurrentProbe);
  motor2RawValue = analogRead(motor2CurrentProbe);
  motor3RawValue = analogRead(motor3CurrentProbe);
  motor4RawValue = analogRead(motor4CurrentProbe);

  motor1FinalValue = motor1RawValue * 5.0 / 1023.0;
  motor1Stall = (motor1FinalValue > motorStallLimit);

  motor2FinalValue = motor2RawValue * 5.0 / 1023.0;
  motor2Stall = (motor2FinalValue > motorStallLimit);

  motor3FinalValue = motor3RawValue * 5.0 / 1023.0;
  motor3Stall = (motor3FinalValue > motorStallLimit);

  motor4FinalValue = motor4RawValue * 5.0 / 1023.0;
  motor4Stall = (motor4FinalValue > motorStallLimit);

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
  angle16 += 1800;                           // correction of 180 degrees mounting difference to heading
  angle16 = angle16 % 3600;
  if ((angle16 >= 3595) || (angle16 <= 5)) angle16 = 0;               // Hysteresis +- 0.5 degree arount 0 degrees

  if (turnFinished == true){
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

  
    // Read distances to obstructions if no turn comman is running (shorten cycle time)
    // *************************************************************************************************************************************
    digitalWrite(distancefLeftTrig, LOW);                               // front Left
    delayMicroseconds(5);                                               // ****
    digitalWrite(distancefLeftTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(distancefLeftTrig, LOW);
    distancefLeftPulseTime = pulseIn(distancefLeftEcho, HIGH, 12000);    // Range limited to 200cm (timeout delivers 0)
    if (distancefLeftPulseTime > 60) {                                  // disturbance filter
      distancefLeftCm = distancefLeftPulseTime / 29 / 2;
    }
    else { 
      distancefLeftCm = 210;                                            // out of range
    }
    distancefLeftObstruction = (distancefLeftCm < distancefLeftLimit);  // Obstruction detected front left
  
    digitalWrite(distancefRightTrig, LOW);                              // front Right
    delayMicroseconds(5);                                               // *****
    digitalWrite(distancefRightTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(distancefRightTrig, LOW);
    distancefRightPulseTime = pulseIn(distancefRightEcho, HIGH, 12000);  // Range limited to 200cm
    if (distancefRightPulseTime > 60) {                                  // disturbance filter
      distancefRightCm = distancefRightPulseTime / 29 / 2;
    }
    else {
      distancefRightCm = 210;                                            // out of range
    }
    distancefRightObstruction = (distancefRightCm < distancefRightLimit);// Obstruction detected front right
    
    
  //  digitalWrite(distancebLeftTrig, LOW);                               // back Left
  //  delayMicroseconds(5);                                               // ****
  //  digitalWrite(distancebLeftTrig, HIGH);
  //  delayMicroseconds(10);
  //  digitalWrite(distancebLeftTrig, LOW);
  //  distancebLeftPulseTime = pulseIn(distancebLeftEcho, HIGH, 12000);
  //  if (distancebLeftPulseTime > 60) {                                  // disturbance filter
  //    distancebLeftCm = distancebLeftPulseTime / 29 / 2;
  //  }
  //  else {
  //    distancebLeftCm = 210;                                            // out of range
  //  }
  //  distancebLeftObstruction = (distancebLeftCm < distancebLeftLimit);// Obstruction detected back left
  //  
  //  digitalWrite(distancebRightTrig, LOW);                              // back Right
  //  delayMicroseconds(5);                                               // *****
  //  digitalWrite(distancebRightTrig, HIGH);
  //  delayMicroseconds(10);
  //  digitalWrite(distancebRightTrig, LOW);
  //  distancebRightPulseTime = pulseIn(distancebRightEcho, HIGH, 12000);
  //  if (distancebRightPulseTime > 60) {                                 // disturbance filter
  //    distancebRightCm = distancebRightPulseTime / 29 / 2;
  //  }
  //  else {
  //    distancebRightCm = 210;                                           // out of range
  //  }
  //  distancebRightObstruction = (distancebRightCm < distancebRightLimit);// Obstruction detected back right
    
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
        if (alignCommand && distancebLeftCm >= 10 && distancebLeftCm <= 200){
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
      if (aL > - 0.2 && aL < 0.2){
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
      if (aR > - 0.2 && aR < 0.2){
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
  
    digitalWrite(distanceFrontTrig, LOW);                               // Front
    delayMicroseconds(5);                                               // *****
    digitalWrite(distanceFrontTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(distanceFrontTrig, LOW);
    distanceFrontPulseTime = pulseIn(distanceFrontEcho, HIGH, 12000);   // Range limited to 200cm
    if (distanceFrontPulseTime > 60) {                                  // disturbance filter
      distanceFrontCm = distanceFrontPulseTime / 29 / 2;
    }
    else {
      distanceFrontCm = 210;                                            // out of range
    }
    distanceFrontObstruction = (distanceFrontCm < distanceFrontLimit);// Obstruction detected front side
  
    digitalWrite(distanceUpTrig, LOW);                                  // Up
    delayMicroseconds(5);                                               // **
    digitalWrite(distanceUpTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(distanceUpTrig, LOW);
    distanceUpPulseTime = pulseIn(distanceUpEcho, HIGH, 17000);         // Range limited to 300cm
    if (distanceUpPulseTime > 60) {                                     // disturbance filter
      distanceUpCm = distanceUpPulseTime / 29 / 2;
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
    emergencyStop =  battery9VLow || battery7VLow || battery5VLow  || Arduino5VLow
                     || distancefRightObstruction  || distancefLeftObstruction  || distanceFrontObstruction  
                     || motor1Stall  || motor2Stall  || motor3Stall  || motor4Stall  
                     || UpitchLimitExceeded  || LpitchLimitExceeded || UrollLimitExceeded  || LrollLimitExceeded
                     || usbDisturbance;
//                     || distanceDownObstruction || wlanDisturbance;
  }


  // Read encoders
  // *************************************************************************************************************************************
  encLt += QuadratureEncoderReadLt(); // calculation of intermediate value (Lt, Rt) and driving distance to be done at Raspberry pi 2
  encRt += QuadratureEncoderReadRt();

  // Motor control
  // *************************************************************************************************************************************

  MotorControl();


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
  if (Serial) { usbDisturbance = false;                                            // USB ok
    if (turnFinished == true) {
      // Send values to USB interface if no turn command is running (shorten cycle time)
      // *************************************************************************************************************************************
    
      Serial.print("MV@EncLt: V ");
      Serial.println(encLt, DEC);
      Serial.print("MV@EncRt: V ");
      Serial.println(encRt, DEC);
    
      Serial.print("MV@Battery 9V: V ");
      Serial.println(battery9VFinalValue);
      Serial.print("MV@Battery 9V: LL_Exceeded ");
      Serial.println(battery9VLow);
    
      Serial.print("MV@Battery 7V: V ");
      Serial.println(battery7VFinalValue);
      Serial.print("MV@Battery 7V: LL_Exceeded ");
      Serial.println(battery7VLow);
    
      Serial.print("MV@Battery 5V: V ");
      Serial.println(battery5VFinalValue);
      Serial.print("MV@Battery 5V: LL_Exceeded ");
      Serial.println(battery5VLow);
    
      Serial.print("MV@Arduino 5V: V ");
      Serial.println(Arduino5VFinalValue);
      Serial.print("MV@Arduino 5V: LL_Exceeded ");
      Serial.println(Arduino5VLow);

      Serial.print("MV@Distance fleft: V ");
      Serial.println(distancefLeftCm);
      Serial.print("MV@Distance fleft: LL_Exceeded ");
      Serial.println(distancefLeftObstruction);
    
      Serial.print("MV@Distance fright: V ");
      Serial.println(distancefRightCm);
      Serial.print("MV@Distance fright: LL_Exceeded ");
      Serial.println(distancefRightObstruction);
  
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
      
      Serial.print("MV@Distance down: V ");
      Serial.println(distanceDownCm);
      Serial.print("MV@Distance down: UL_Exceeded ");
      Serial.println(distanceDownObstruction);

      Serial.print("S@Forward slow: ");
      Serial.println(forwardSlowCommand);
      Serial.print("S@Forward half: ");
      Serial.println(forwardHalfCommand);
      Serial.print("S@Forward full: ");
      Serial.println(forwardFullCommand);
      Serial.print("S@Steering left: ");
      Serial.println(steeringLeftCommand);
      Serial.print("S@Steering right: ");
      Serial.println(steeringRightCommand);
      Serial.print("S@Turn slow 45 left: ");
      Serial.println(turnSlow45LeftCommand);
      Serial.print("S@Turn slow 45 right: ");
      Serial.println(turnSlow45RightCommand);
      Serial.print("S@Turn slow 90 left: ");
      Serial.println(turnSlow90LeftCommand);
      Serial.print("S@Turn slow 90 right: ");
      Serial.println(turnSlow90RightCommand);
      Serial.print("S@Align: ");
      Serial.println(alignCommand);

      Serial.print("MV@Motor1 current: UL ");
      Serial.println(motorStallLimit);
      Serial.print("MV@Motor2 current: UL ");
      Serial.println(motorStallLimit);
      Serial.print("MV@Motor3 current: UL ");
      Serial.println(motorStallLimit);
      Serial.print("MV@Motor4 current: UL ");
      Serial.println(motorStallLimit);
      
      Serial.print("MV@Motor1 current: V ");
      Serial.println (motor1FinalValue);
      Serial.print("MV@Motor1 current: UL_Exceeded ");
      Serial.println(motor1Stall);
    
      Serial.print("MV@Motor2 current: V ");
      Serial.println(motor2FinalValue);
      Serial.print("MV@Motor2 current: UL_Exceeded ");
      Serial.println(motor2Stall);
    
      Serial.print("MV@Motor3 current: V ");
      Serial.println(motor3FinalValue);
      Serial.print("MV@Motor3 current: UL_Exceeded ");
      Serial.println(motor3Stall);
    
      Serial.print("MV@Motor4 current: V ");
      Serial.println(motor4FinalValue);
      Serial.print("MV@Motor4 current: UL_Exceeded ");
      Serial.println(motor4Stall);
  
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
    
      Serial.print("MV@Turned angle: V ");
      Serial.println(turnedAngle);
  
      Serial.print("S@Compass: ");
      Serial.println(digitalRead(CMPS11_VCC));
  
      Serial.print("S@Amplifier: ");
      Serial.println(digitalRead(amplifier_VCC));
      
      Serial.print("S@W-LAN disturbance: ");
      Serial.println(wlanDisturbance);
    }
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
//    Serial.print("I@Command: ");
//    Serial.println(CommandStringS);
  
    // Get command from USB interface
    // *************************************************************************************************************************************
  
    SerialParser();
    if (ByteCount  > 0) {
      CommandString = Command;
      CommandStringS = CommandString;
      // It is allowed to change between driving commands directly, no need for a stop command between driving commands
  
      if (CommandString.startsWith("Stop")) {forwardStopCommand = true;
      }
      
      if (CommandString.startsWith("Forward slow")) {
        forwardStopCommand = false; forwardSlowCommand = true; forwardHalfCommand = false; forwardFullCommand = false;
        turnSlow45LeftCommand = false; turnSlow45RightCommand = false;
        turnSlow90LeftCommand = false; turnSlow90RightCommand = false;
        turnSlowLeftToCommand = false; turnSlowRightToCommand = false;
        motorStallLimit = motorStallLimitSlow;
      }
      if (CommandString.startsWith("Forward half")) {
        forwardStopCommand = false; forwardHalfCommand = true; forwardSlowCommand = false; forwardFullCommand = false;
        turnSlow45LeftCommand = false; turnSlow45RightCommand = false;
        turnSlow90LeftCommand = false; turnSlow90RightCommand = false;
        turnSlowLeftToCommand = false; turnSlowRightToCommand = false;
        motorStallLimit = motorStallLimitHalf;
      }
      if (CommandString.startsWith("Forward full")) {
        forwardStopCommand = false; forwardFullCommand = true; forwardSlowCommand = false; forwardHalfCommand = false;
        turnSlow45LeftCommand = false; turnSlow45RightCommand = false;
        turnSlow90LeftCommand = false; turnSlow90RightCommand = false;
        turnSlowLeftToCommand = false; turnSlowRightToCommand = false;
        motorStallLimit = motorStallLimitFull;
      }
      if (CommandString.startsWith("Align")){
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
      if (CommandString.startsWith("Turn slow 45 left")) {
        startAngle = angle16;                     // store start angle
        turnAngleRelative = 430;
        turnAngle = (3600 + angle16 - 430) % 3600;
        
        turnSlow45LeftCommand = true;
        turnSlowLeftToCommand = true;
        forwardStopCommand = false; forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
        steeringLeftCommand = false; steeringRightCommand = false; alignCommand = false;
        turnSlowRightToCommand = false; turnFinished = false;
        motorStallLimit = motorStallLimitSlow;
      }
  
      if (CommandString.startsWith("Turn slow 45 right")) {
        startAngle = angle16;                     // store start angle
        turnAngleRelative = 430;
        turnAngle = (angle16 + 430) % 3600;
        
        turnSlow45RightCommand = true;
        turnSlowRightToCommand = true;  
        forwardStopCommand = false; forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
        steeringLeftCommand = false; steeringRightCommand = false; alignCommand = false;
        turnSlowLeftToCommand = false; turnFinished = false;
        motorStallLimit = motorStallLimitSlow;
      }
      if (CommandString.startsWith("Turn slow 90 left")) {
        startAngle = angle16;                     // store start angle
        turnAngleRelative = 880;
        turnAngle = (3600 + angle16 - 880) % 3600;
        
        turnSlow90LeftCommand = true;
        turnSlowLeftToCommand = true;
        forwardStopCommand = false; forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
        steeringLeftCommand = false; steeringRightCommand = false; alignCommand = false;
        turnSlowRightToCommand = false; turnFinished = false;
        motorStallLimit = motorStallLimitSlow;
      }
      if (CommandString.startsWith("Turn slow 90 right")) {
        startAngle = angle16;                     // store start angle
        turnAngleRelative = 880;
        turnAngle = (angle16 + 880) % 3600;
               
        turnSlow90RightCommand = true;
        turnSlowRightToCommand = true;  
        forwardStopCommand = false; forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
        steeringLeftCommand = false; steeringRightCommand = false; alignCommand = false;
        turnSlowLeftToCommand = false; turnFinished = false;
        motorStallLimit = motorStallLimitSlow;
      }
      if (CommandString.startsWith("Turn slow to: ")) {
        forwardStopCommand = false; forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
        steeringLeftCommand = false; steeringRightCommand = false; alignCommand = false;        
        turnSlowLeftToCommand = false; turnSlowRightToCommand = false; turnFinished = false;
        motorStallLimit = motorStallLimitSlow;

        startAngle = angle16;                                                         // store start angle

        // calculate relative angle and decide direction left right
        CommandString.replace("Turn slow to: ", "");
        turnAngle = CommandString.toInt();                                            // Absolute angle to be turned to
        i = (turnAngle - angle16);                                                     
        turnAngleRelative = i;
        if (i < -1800) turnAngleRelative = i + 3600;                                  // decision turn left or right
        if (i > 1800) turnAngleRelative = i - 3600;
        
        if (turnAngleRelative >= 0) {                                                 // turn right to
          turnAngle = (3600 + turnAngle - 20) % 3600;                                 // cycle tolerance
          turnAngleRelative = (3600 + turnAngleRelative - 20) % 3600;
          turnSlowRightToCommand = true;                                              
        }
        else {                                                                        // turn left to
          turnAngleRelative = abs(turnAngleRelative);                                 
          turnAngle = (3600 + turnAngle + 20) % 3600;                                 // cycle tolerance
          turnAngleRelative = (3600 + turnAngleRelative - 20) % 3600;
          turnSlowLeftToCommand = true;                                               
        }       
      }
     
      if (CommandString.startsWith("Wlan ready")) wlanReady = true;                   // Live beat of W-LAN communication
      if (CommandString.startsWith("Encoder reset")) {
        encLt = 0;                                                                    // Encoder reset
        encRt = 0;
      }
      if (CommandString.startsWith("Amplifier: ")){                                   // Amplifier 0/1
        if (CommandString.substring(11) == "0") digitalWrite(amplifier_VCC, 0);
        else digitalWrite(amplifier_VCC, 1);
      }
    }
  }
  else {usbDisturbance = true;                                                        // USB disturbance
  Serial.end();
  Serial.begin(baud);
  Serial.setTimeout(10);
  delay(2000);                                                                        // wait for next trial
  }

}

