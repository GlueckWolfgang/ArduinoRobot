// ****************************************************************************************************************************************************
// *** Arduino robot program
// *** Version: 2015.12_22
// *** Developer: Wolfgang Glück
// ***
// *** Supported hardware:
// ***   - Rover5 chassis with 4 motors and encoders                (2 encoders used only)
// ***   - Rover5 chassis with 2 motors and encoders                (optional, just remove the unused parts of code)
// ***   - Motor control board (MCB) for 4 motors and encoders
// ***   - Motor control Board (MCB) for 2 motors and encoders      (optional)
// ***   - Arduino mega (Spider)
// ***   - CMPS11                                                   (via 10cm I2C bus, using internal pullups by default)
// ***   - 5 x US-015
// ***   - 1 x IR Sensor (Sharp)
// ***   - 7,2 V Battery for MCB and drives                         (6x 1,2 V NiMH Akku type D 10'000 mAh, same battery pack as for 9,6 V)
// ***   - 9,6 V Battery for Arduino and MCB                        (8x 1,2 V NiMH Akku type D 10'000 mAh)
// ***   - 5 V / Gnd     for MCB is connected to Arduino 5 V / Gnd
// ***   - 5 V   Battery for Raspberry pi                           (Power bank 20'000 mAh)
// ***
// ***   - Raspberry pi 2
// ***   - SD card 32GB
// ***   - camera
// ***   - W-LAN adapter EW-7612UAN V2
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
// ***          (TurnSlow45Left, TurnSlow45Right, TurnSlow90Left, TurnSlow90Right)
// ***   - (ok) Get reset command for encoder values
// ***   - (ok) Get status about W-LAN from USB interface
// ***   - (ok) Supervise communication status to USB interface
// *** Emergency stop commented out for test reasons
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

#define pitchLimit 30        // + - limit in degrees
#define rollLimit 20         // + - limit in degrees
boolean pitchLimitExceeded = false;
boolean rollLimitExceeded = false;
int check;
int i;

// LED, communication status
// *************************************************************************************************************************
#define ledPin 13                 // LED for heart beat

boolean wlanDisturbance = false;  // if WLAN is not ready for more than 3 cycles
int wlanReadyCount = 0;
boolean wlanReady = false;        // will be set by command from USB interface and reset by Arduino
boolean usbDisturbance = false;   // if USB interface is not ready

// Batteries
// **************************************************************************************************************************
#define battery9VProbe A0    // wire to battery via 1:1 voltage divider with 4.7kOhm resistors
int battery9VRawValue = 0;
float battery9VFinalValue = 0.0;
float battery9VLowerLimit = 6.9;
boolean battery9VLow = false;

#define battery7VProbe A1    // wire to battery via 1:1 voltage divider with 4.7 kOhm resistors
int battery7VRawValue = 0;
float battery7VFinalValue = 0.0;
float battery7VLowerLimit = 4.9;
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
boolean forwardStopCommand  = false; // Stop command
boolean forwardSlowCommand  = false; // Forward slow command
boolean forwardHalfCommand  = false; // Forward half command
boolean forwardFullCommand  = false; // Forward full command
boolean steeringLeftCommand = false; // Steering left hand
boolean steeringRightCommand = false; // Steering right hand
boolean turnSlow45LeftCommand  = false; // Turn slow 45 degrees left
boolean turnSlow45RightCommand = false; // Turn slow 45 degrees right
boolean turnSlow90LeftCommand  = false; // Turn slow 90 degrees left
boolean turnSlow90RightCommand = false; // Turn slow 90 degrees right
boolean turnFinished        = true;
int stopDutyCycle           =   0;   //   0% of 256
int slowDutyCycle           =  64;   //  25% of 256
int halfDutyCycle           = 128;   //  50% of 256
int fullDutyCycle           = 213;   //  80% of 256, contains reserve for steering
int steeringRate            = 20;    //    % of DutyCycle
int steeringDutyCycle       = 0;     // Calculated dutyCycle

void MotorControl()
{
  if (emergencyStop || forwardStopCommand) {        // Emergency stop or manually stop
    analogWrite(motor1PWM, stopDutyCycle);          // *******************************
    analogWrite(motor2PWM, stopDutyCycle);
    analogWrite(motor3PWM, stopDutyCycle);
    analogWrite(motor4PWM, stopDutyCycle);
    forwardStopCommand = false;
    forwardSlowCommand = false;
    forwardHalfCommand = false;
    forwardFullCommand = false;
    steeringLeftCommand = false;
    steeringRightCommand = false;
    turnSlow45LeftCommand = false;
    turnSlow45RightCommand = false;
    turnSlow90LeftCommand = false;
    turnSlow90RightCommand = false;
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

  if (turnSlow45LeftCommand) {                  // Turn left 45 degrees
    if (startAngle == 0) {                      // ********************
      startAngle = angle16;                     // store start angle
      turnFinished = false;
    }
    if ((startAngle - angle16) < 0 && (startAngle - angle16) > -10) turnedAngle = 0; // Filter measuring faults
    else turnedAngle = (3600 + startAngle - angle16) % 3600;
    digitalWrite(motor1Direction, backward);
    digitalWrite(motor2Direction, backward);
    digitalWrite(motor3Direction, forward);
    digitalWrite(motor4Direction, forward);
    analogWrite(motor1PWM, slowDutyCycle);
    analogWrite(motor2PWM, slowDutyCycle);
    analogWrite(motor3PWM, slowDutyCycle);
    analogWrite(motor4PWM, slowDutyCycle);

    if (turnedAngle >= 450) {                    // Target reached
      analogWrite(motor1PWM, stopDutyCycle);
      analogWrite(motor2PWM, stopDutyCycle);
      analogWrite(motor3PWM, stopDutyCycle);
      analogWrite(motor4PWM, stopDutyCycle);
      turnSlow45LeftCommand = false;
      startAngle = 0;
      turnFinished = true;
    }
  }

  if (turnSlow45RightCommand) {                   // Turn right 45 degrees
    if (startAngle == 0) {                        // *********************
      startAngle = angle16;                       // store start angle
      turnFinished = false;
    }
    if ((angle16 - startAngle) < 0 && (angle16 - startAngle) > -10) turnedAngle = 0; // Filter measuring faults
    else turnedAngle = (3600 + angle16 - startAngle) % 3600;
    digitalWrite(motor1Direction, forward);
    digitalWrite(motor2Direction, forward);
    digitalWrite(motor3Direction, backward);
    digitalWrite(motor4Direction, backward);
    analogWrite(motor1PWM, slowDutyCycle);
    analogWrite(motor2PWM, slowDutyCycle);
    analogWrite(motor3PWM, slowDutyCycle);
    analogWrite(motor4PWM, slowDutyCycle);

    if (turnedAngle >= 450) {                      // Target reached
      analogWrite(motor1PWM, stopDutyCycle);
      analogWrite(motor2PWM, stopDutyCycle);
      analogWrite(motor3PWM, stopDutyCycle);
      analogWrite(motor4PWM, stopDutyCycle);
      turnSlow45RightCommand = false;
      startAngle = 0;
      turnFinished = true;
    }
  }

  if (turnSlow90LeftCommand) {                    // Turn left 90 degrees
    if (startAngle == 0) {                        // ********************
      startAngle = angle16;                       // store start angle
      turnFinished = false;
    }
    if ((startAngle - angle16) < 0 && (startAngle - angle16) > -10) turnedAngle = 0; // Filter measuring faults
    else turnedAngle = (3600 + startAngle - angle16) % 3600;
    digitalWrite(motor1Direction, backward);
    digitalWrite(motor2Direction, backward);
    digitalWrite(motor3Direction, forward);
    digitalWrite(motor4Direction, forward);
    analogWrite(motor1PWM, slowDutyCycle);
    analogWrite(motor2PWM, slowDutyCycle);
    analogWrite(motor3PWM, slowDutyCycle);
    analogWrite(motor4PWM, slowDutyCycle);

    if (turnedAngle >= 900) {                     // Target reached
      analogWrite(motor1PWM, stopDutyCycle);
      analogWrite(motor2PWM, stopDutyCycle);
      analogWrite(motor3PWM, stopDutyCycle);
      analogWrite(motor4PWM, stopDutyCycle);
      turnSlow90LeftCommand = false;
      startAngle = 0;
      turnFinished = true;
    }
  }

  if (turnSlow90RightCommand) {                   // Turn right 90 degrees
    if (startAngle == 0) {                        // *********************
      startAngle = angle16;                       // store start angle
      turnFinished = false;
    }
    if ((angle16 - startAngle) < 0 && (angle16 - startAngle) > -10) turnedAngle = 0; // Filter measuring faults
    else turnedAngle = (3600 + angle16 - startAngle) % 3600;
    digitalWrite(motor1Direction, forward);
    digitalWrite(motor2Direction, forward);
    digitalWrite(motor3Direction, backward);
    digitalWrite(motor4Direction, backward);
    analogWrite(motor1PWM, slowDutyCycle);
    analogWrite(motor2PWM, slowDutyCycle);
    analogWrite(motor3PWM, slowDutyCycle);
    analogWrite(motor4PWM, slowDutyCycle);

    if (turnedAngle >= 900) {                      // Target reached
      analogWrite(motor1PWM, stopDutyCycle);
      analogWrite(motor2PWM, stopDutyCycle);
      analogWrite(motor3PWM, stopDutyCycle);
      analogWrite(motor4PWM, stopDutyCycle);
      turnSlow90RightCommand = false;
      startAngle = 0;
      turnFinished = true;
    }
  }
  return;
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
#define distanceRightEcho 36
#define distanceRightTrig 37
#define distanceRightLimit 40
int distanceRightPulseTime = 0;
int distanceRightCm = 0;
boolean distanceRightObstruction = false;

#define distanceLeftEcho 42
#define distanceLeftTrig 43
#define distanceLeftLimit 40
int distanceLeftPulseTime = 0;
int distanceLeftCm = 0;
boolean distanceLeftObstruction = false;

#define distanceFrontEcho 38
#define distanceFrontTrig 39
#define distanceFrontLimit 40
int distanceFrontPulseTime = 0;
int distanceFrontCm = 0;
boolean distanceFrontObstruction = false;

#define distanceDownEcho 40
#define distanceDownTrig 41
#define distanceDownLimit 10
int distanceDownPulseTime = 0;
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


// Commands
// ***********************************************************************************************************************

const int bSize = 20;
int ByteCount;
char Buffer[bSize];  // Serial buffer
char Command[18];    // Arbitrary Value for command size
String CommandString = "";


void SerialParser(void)
{
  //  One command per line.
  //  Command Format: "up to 18 Letter command <\0>"
  //  count will  be below Zero on a timeout.
  //  read up to X chars or until EOT - in this case "\0"
  ByteCount = -1;
  ByteCount =  Serial.readBytesUntil('\0', Buffer, bSize);
  if (ByteCount  > 0) strcpy(Command, strtok(Buffer, "\0"));
  memset(Buffer, 0, sizeof(Buffer));   // Clear contents of Buffer
  Serial.flush();
}

//  Setup
// ***********************************************************************************************************************
// ***********************************************************************************************************************
void setup()
{
  Serial.begin(250000);
  Serial.setTimeout(10);
  pinMode(ledPin, OUTPUT);

  pinMode(distanceRightTrig, OUTPUT);
  pinMode(distanceLeftTrig, OUTPUT);
  pinMode(distanceFrontTrig, OUTPUT);
  pinMode(distanceDownTrig, OUTPUT);
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

  // CMPS11 startup
  // ************************************************************************************************************
  digitalWrite(CMPS11_VCC, 1);
  delay(2000);
  Wire.begin();

  // Encoder Software initiation
  // ************************************************************************************************************
  QuadratureEncoderInit();

  // Calibration
  // ************************************************************************************************************
  // Initiate calibration
  // ********************
  // for calibration deactivate "Reading CPMS values" code and activate "Initiate calibration" code below
  // after loading Software to Arduiono LED on CMPS11 should be off
  // Turn the robot (CMPS11) around 360 degrees slowly
  // The LED on CMPS11 is blinking for any detected maximum value
  // deactivate "Initiate calibration" code and activate "Stop calibration" code

  //Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
  //Wire.write(byte(0x00));                  //set register pointer to command register
  //Wire.write(CMPS11_Byte1);                //Send command
  //Wire.endTransmission();
  //delay(20);

  //Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
  //Wire.write(byte(0x00));                  //set register pointer to command register
  //Wire.write(CMPS11_Byte2);                //Send command
  //Wire.endTransmission();
  //delay(20);

  //Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
  //Wire.write(byte(0x00));                  //set register pointer to command register
  //Wire.write(CMPS11_Byte3);                //Send command
  //Wire.endTransmission();
  //delay(20);

  // Stop calibration
  // ****************
  // For stopping calibration deactivate "Initiate calibration" code and activate "Stop calibration" code
  // after stopping calibration deactivate "Stop calibration" code and  activate "Read CMPS11 values" code again

  //Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
  //Wire.write(byte(0x00));                  //set register pointer to command register
  //Wire.write(CMPS11_Byte4);                //Send command
  //Wire.endTransmission();
  //delay(20);
}

// Loop
// ***********************************************************************************************************************************
// ***********************************************************************************************************************************
void loop()
{

  // LED heart beat
  // *********************************************************************************************************************************
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin by XOR
  delay(250);   // regulation of cycle time because of intermediateAngle calculation

  // Read battery probe and check limit
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
  check = pitch;
  if (check < 0) check = check * -1;         // make a positive value for checking + - limit
  pitchLimitExceeded = (check > pitchLimit); // checking pitch limit
  check = roll;
  if (check < 0) check = check * -1;         // make a positive value for checking + - limit
  rollLimitExceeded = (check > rollLimit);   // checking roll limit

  angle16 = high_byte;                       // Calculate 16 bit angle
  angle16 <<= 8;
  angle16 += low_byte;
  angle16 += 1800;                           // correction of 180 degrees mounting difference to heading
  angle16 = angle16 % 3600;
  if ((angle16 >= 3595) || (angle16 <= 5)) angle16 = 0;               // Hysteresis +- 0.5 degree arount 0 degrees

  // calculate sliding intermediate value over the last 10 values
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

  // Read distances to obstructions
  // *************************************************************************************************************************************
  digitalWrite(distanceLeftTrig, LOW);                                // Left
  delayMicroseconds(5);                                               // ****
  digitalWrite(distanceLeftTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(distanceLeftTrig, LOW);
  distanceLeftPulseTime = pulseIn(distanceLeftEcho, HIGH);
  if (distanceLeftPulseTime > 60) {                                   // disturbance filter
    distanceLeftCm = distanceLeftPulseTime / 29 / 2;
  }
  distanceLeftObstruction = (distanceLeftCm < distanceLeftLimit);     // Obstruction detected left side

  digitalWrite(distanceRightTrig, LOW);                               // Right
  delayMicroseconds(5);                                               // *****
  digitalWrite(distanceRightTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(distanceRightTrig, LOW);
  distanceRightPulseTime = pulseIn(distanceRightEcho, HIGH);
  if (distanceRightPulseTime > 60) {                                  // disturbance filter
    distanceRightCm = distanceRightPulseTime / 29 / 2;
  }
  distanceRightObstruction = (distanceRightCm < distanceRightLimit);  // Obstruction detected right side

  digitalWrite(distanceFrontTrig, LOW);                               // Front
  delayMicroseconds(5);                                               // *****
  digitalWrite(distanceFrontTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(distanceFrontTrig, LOW);
  distanceFrontPulseTime = pulseIn(distanceFrontEcho, HIGH);
  if (distanceFrontPulseTime > 60) {                                  // disturbance filter
    distanceFrontCm = distanceFrontPulseTime / 29 / 2;
  }
  distanceFrontObstruction = (distanceFrontCm < distanceFrontLimit);  // Obstruction detected front side

  digitalWrite(distanceUpTrig, LOW);                                  // Up
  delayMicroseconds(5);                                               // **
  digitalWrite(distanceUpTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(distanceUpTrig, LOW);
  distanceUpPulseTime = pulseIn(distanceUpEcho, HIGH);
  if (distanceUpPulseTime > 60) {                                     // disturbance filter
    distanceUpCm = distanceUpPulseTime / 29 / 2;
  }
  distanceUpDoor = (distanceUpCm < distanceUpLimit);                  // Door passing

  digitalWrite(distanceDownTrig, LOW);                                // Down
  delayMicroseconds(5);                                               // ****
  digitalWrite(distanceDownTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(distanceDownTrig, LOW);
  distanceDownPulseTime = pulseIn(distanceDownEcho, HIGH);
  distanceDownRawValue = analogRead(distanceDownProbe);               // Reading IR sensor
                                                                      // Get a correct new distance  from US- and IR- sensor
                                                                      // otherwise keep the old distance to kompensate short disturbances
  if (distanceDownPulseTime > 60) {                                   // disturbance filter
    distanceDownCm = distanceDownPulseTime / 29 / 2;
  
    if (distanceDownCm <= distanceDownLimit){ 
      distanceDownCm = (5000 / (distanceDownRawValue - 20)) - 15;
    }
  }
  distanceDownObstruction = (distanceDownCm > distanceDownLimit);     // Obstruction like down stair detected

  // Build and execute emergency stop
  // *************************************************************************************************************************************

  if (emergencyStop == false) {                                       // keep emergency stop stored until manually reset
    emergencyStop =  battery9VLow || battery7VLow || battery5VLow  || Arduino5VLow
                     || distanceRightObstruction  || distanceLeftObstruction  || distanceFrontObstruction  || distanceDownObstruction
                     || motor1Stall  || motor2Stall  || motor3Stall  || motor4Stall  || pitchLimitExceeded  || rollLimitExceeded
                     || usbDisturbance;
//                     || wlanDisturbance;
  }


  // Read encoders
  // *************************************************************************************************************************************
  encLt += QuadratureEncoderReadLt(); // calculation of intermediate value (Lt, Rt) and driving distance to be done at Raspberry pi 2
  encRt += QuadratureEncoderReadRt();

  // Motor control
  // *************************************************************************************************************************************

  MotorControl();


  // Send values to USB interface
  // *************************************************************************************************************************************

  Serial.println("");
  Serial.print ("Version: ");
  Serial.println(ARDUINO);

  Serial.print("EncLt: ");
  Serial.println(encLt, DEC);
  Serial.print("EncRt: ");
  Serial.println(encRt, DEC);

  Serial.print("Battery 9V: ");
  Serial.println(battery9VFinalValue);
  Serial.print("Battery 9V: LL ");
  Serial.println(battery9VLowerLimit);
  Serial.print("Battery 9V: LL below: ");
  Serial.println(battery9VLow);

  Serial.print("Battery 7V: ");
  Serial.println(battery7VFinalValue);
  Serial.print("Battery 7V: LL ");
  Serial.println(battery7VLowerLimit);
  Serial.print("Battery 7V: LL below ");
  Serial.println(battery7VLow);

  Serial.print("Battery 5V: ");
  Serial.println(battery5VFinalValue);
  Serial.print("Battery 5V: LL ");
  Serial.println(battery5VLowerLimit);
  Serial.print("Battery 5V: LL below ");
  Serial.println(battery5VLow);

  Serial.print("Arduino 5V: ");
  Serial.println(Arduino5VFinalValue);
  Serial.print("Arduino 5V: LL ");
  Serial.println(Arduino5VLowerLimit);
  Serial.print("Arduino 5V: LL below ");
  Serial.println(Arduino5VLow);

  Serial.print("Motor1 current: UL ");
  Serial.println(motorStallLimit);
  Serial.print("Motor1 current: ");
  Serial.println (motor1FinalValue);
  Serial.print("Motor1 current: Stall");
  Serial.println(motor1Stall);

  Serial.print("Motor2 current: UL ");
  Serial.println(motorStallLimit);
  Serial.print("Motor2 current: ");
  Serial.println(motor2FinalValue);
  Serial.print("Motor2 current: Stall ");
  Serial.println(motor2Stall);

  Serial.print("Motor3 current: UL ");
  Serial.println(motorStallLimit);
  Serial.print("Motor3 current: ");
  Serial.println(motor3FinalValue);
  Serial.print("Motor3 current: Stall ");
  Serial.println(motor3Stall);

  Serial.print("Motor4 current: UL ");
  Serial.println(motorStallLimit);
  Serial.print("Motor4 current: ");
  Serial.println(motor4FinalValue);
  Serial.print("Motor4 current: Stall: ");
  Serial.println(motor4Stall);

  Serial.print("Roll: ");
  Serial.println(roll, DEC);
  Serial.print("Roll: UL ");
  Serial.println(rollLimit, DEC);
  Serial.print("Roll: UL Exceeded: ");
  Serial.println(rollLimitExceeded);

  Serial.print("Pitch: ");
  Serial.println(pitch, DEC);
  Serial.print("Pitch: UL ");
  Serial.println(pitchLimit, DEC);
  Serial.print("Pitch: UL Exceeded ");
  Serial.println(pitchLimitExceeded);

  Serial.print("Actual angle: ");
  Serial.print(angle16 / 10, DEC);
  Serial.print(".");
  Serial.println(angle16 % 10, DEC);

  Serial.print("Smoothed angle: ");
  Serial.print(intermediateAngle / 10, DEC);
  Serial.print(".");
  Serial.println(intermediateAngle % 10, DEC);

  Serial.print("Distance fleft: ");
  Serial.println(distanceLeftCm);
  Serial.print("Distance fleft: LL ");
  Serial.println(distanceLeftLimit);
  Serial.print("Distance fleft: Obstruction ");
  Serial.println(distanceLeftObstruction);

  Serial.print("Distance fright: ");
  Serial.println(distanceRightCm);
  Serial.print("Distance fright: LL ");
  Serial.println(distanceRightLimit);
  Serial.print("Distance fright: Obstruction ");
  Serial.println(distanceRightObstruction);

  Serial.print("Distance front: ");
  Serial.println(distanceFrontCm);
  Serial.print("Distance front: LL ");
  Serial.println(distanceFrontLimit);
  Serial.print("Distance front: Obstruction ");
  Serial.println(distanceFrontObstruction);

  Serial.print("Distance up: ");
  Serial.println(distanceUpCm);
  Serial.print("Distance up: LL ");
  Serial.println(distanceUpLimit);

  // Serial.print("Distance down raw value: "); // For test reasons only
  // Serial.println(distanceDownRawValue);      // For test reasons only
  // Serial.print("Distance down pulse time: ");// For test reasons only
  // Serial.println(distanceDownPulseTime);     // For test reasons only
  Serial.print("Distance down: ");
  Serial.println(distanceDownCm);
  Serial.print("Distance down: UL ");
  Serial.println(distanceDownLimit);
  Serial.print("Distance down: Obstruction ");
  Serial.println(distanceDownObstruction);

  Serial.print("Turned angle: ");
  Serial.println(turnedAngle);

  Serial.print("Turn finished: ");
  Serial.println(turnFinished);
  // if (wlanDisturbance)           Serial.println("W-LAN disturbance!"); // For test reasons only Raspbery supervises the interface too
  // if (usbDisturbance)            Serial.println("USB disturbance!");   // For test reasons only Raspbery supervises the interface too
  if (emergencyStop)             Serial.println("Emergency stop!");
  if (forwardStopCommand)        Serial.println("Stop!");
  if (forwardSlowCommand)        Serial.println("Forward slow!");
  if (forwardHalfCommand)        Serial.println("Forward half!");
  if (forwardFullCommand)        Serial.println("Forward full!");
  if (steeringLeftCommand)       Serial.println("Steering left!");
  if (steeringRightCommand)      Serial.println("Steering right!");
  if (turnSlow45LeftCommand)     Serial.println("Turn slow 45 left!");
  if (turnSlow45RightCommand)    Serial.println("Turn slow 45 right!");
  if (turnSlow90LeftCommand)     Serial.println("Turn slow 90 left!");
  if (turnSlow90RightCommand)    Serial.println("Turn slow 90 right!");

  // Get command from USB interface
  // *************************************************************************************************************************************

  SerialParser();
  if (ByteCount  > 0) {
    CommandString = Command;
    // It is allowed to change between driving commands directly, no need for a stop command between other driving commands

    if (CommandString.startsWith("Stop")) forwardStopCommand = true;
    if (CommandString.startsWith("ForwardSlow")) {
      forwardSlowCommand = true; forwardHalfCommand = false; forwardFullCommand = false;
      turnSlow45LeftCommand = false; turnSlow45RightCommand = false;
      turnSlow90LeftCommand = false; turnSlow90RightCommand = false; turnFinished = true;
    }
    if (CommandString.startsWith("ForwardHalf")) {
      forwardHalfCommand = true; forwardSlowCommand = false; forwardFullCommand = false;
      turnSlow45LeftCommand = false; turnSlow45RightCommand = false;
      turnSlow90LeftCommand = false; turnSlow90RightCommand = false; turnFinished = true;
    }
    if (CommandString.startsWith("ForwardFull")) {
      forwardFullCommand = true; forwardSlowCommand = false; forwardHalfCommand = false;
      turnSlow45LeftCommand = false; turnSlow45RightCommand = false;
      turnSlow90LeftCommand = false; turnSlow90RightCommand = false; turnFinished = true;
    }
    if (CommandString.startsWith("SteeringLeft"))  {
      steeringLeftCommand = true;
      steeringRightCommand = false;
    }
    if (CommandString.startsWith("SteeringRight")) {
      steeringLeftCommand = false;
      steeringRightCommand = true;
    }
    if (CommandString.startsWith("SteeringAhead")) {
      steeringLeftCommand = false;
      steeringRightCommand = false;
    }
    if (CommandString.startsWith("TurnSlow45Left")) {
      turnSlow45LeftCommand = true;
      forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
    }

    if (CommandString.startsWith("TurnSlow45Right")) {
      turnSlow45RightCommand = true;
      forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
      steeringLeftCommand = false; steeringRightCommand = false;
    }
    if (CommandString.startsWith("TurnSlow90Left")) {
      turnSlow90LeftCommand = true;
      forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
      steeringLeftCommand = false; steeringRightCommand = false;
    }
    if (CommandString.startsWith("TurnSlow90Right")) {
      turnSlow90RightCommand = true;
      forwardSlowCommand = false; forwardHalfCommand = false; forwardFullCommand = false;
      steeringLeftCommand = false; steeringRightCommand = false;
    }
    if (CommandString.startsWith("WlanReady")) wlanReady = true;                    // Live beat of W-LAN communication
    if (CommandString.startsWith("EncoderReset")) {
      encLt = 0;  // Encoder reset
      encRt = 0;
    }
  }

  // Supervision of Communication
  // *************************************************************************************************************************************

  if (Serial) usbDisturbance = false;                                             // USB ok
  else usbDisturbance = true;                                                     // USB Disturbance

  if (wlanReady == false) wlanReadyCount += 1;                                    // no wlanReady received since last cykle
  else {
    wlanReady = false;  // wlanReady ok
    wlanReadyCount = 0;
  }
  if (wlanReadyCount < 3) wlanDisturbance = false;                                // W-LAN ok
  else {
    wlanDisturbance = true;  // W-LAN Disturbance
    wlanReadyCount = 3;
  }
}
