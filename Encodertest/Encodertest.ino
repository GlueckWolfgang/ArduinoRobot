// LED
// *************************************************************************************************************************
#define ledPin 13                 // LED for heart beat

// Motors
// *************************************************************************************************************************
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
boolean turnSlow45LeftCommand  = false; // Turn slow 45 degrees left
boolean turnSlow45RightCommand = false; // Turn slow 45 degrees right
boolean turnSlow90LeftCommand  = false; // Turn slow 90 degrees left
boolean turnSlow90RightCommand = false; // Turn slow 90 degrees right
boolean turnFinished        = true;
int stopDutyCycle           =   0;   //   0% of 256
int slowDutyCycle           =  64;   //  25% of 256
int halfDutyCycle           = 128;   //  50% of 256
int fullDutyCycle           = 255;   // 100% of 256

void MotorControl()
{
  if (emergencyStop || forwardStopCommand){         // Emergency stop or manually stop
    analogWrite(motor1PWM, stopDutyCycle);          // *******************************
    analogWrite(motor2PWM, stopDutyCycle);
    analogWrite(motor3PWM, stopDutyCycle);
    analogWrite(motor4PWM, stopDutyCycle);
    forwardStopCommand = false;
    forwardSlowCommand = false;
    forwardHalfCommand = false;
    forwardFullCommand = false;
    turnSlow45LeftCommand = false;
    turnSlow45RightCommand = false;
    turnSlow90LeftCommand = false;
    turnSlow90RightCommand = false;
    turnFinished = true;
    startAngle = 0;
  }
  
  if (forwardSlowCommand){                      // Forward slow
    digitalWrite(motor1Direction,forward);      // ************    
    digitalWrite(motor2Direction,forward);
    digitalWrite(motor3Direction,forward);
    digitalWrite(motor4Direction,forward);
    analogWrite(motor1PWM, slowDutyCycle);
    analogWrite(motor2PWM, slowDutyCycle);
    analogWrite(motor3PWM, slowDutyCycle);
    analogWrite(motor4PWM, slowDutyCycle);
  }
  
  if (forwardHalfCommand){                      // Forward half
  digitalWrite(motor1Direction,forward);        // ************    
  digitalWrite(motor2Direction,forward);
  digitalWrite(motor3Direction,forward);
  digitalWrite(motor4Direction,forward);
  analogWrite(motor1PWM, halfDutyCycle);
  analogWrite(motor2PWM, halfDutyCycle);
  analogWrite(motor3PWM, halfDutyCycle);
  analogWrite(motor4PWM, halfDutyCycle);
  }
  
  if (forwardFullCommand){                      // Forward full
  digitalWrite(motor1Direction,forward);        // ************    
  digitalWrite(motor2Direction,forward);
  digitalWrite(motor3Direction,forward);
  digitalWrite(motor4Direction,forward);
  analogWrite(motor1PWM, fullDutyCycle);
  analogWrite(motor2PWM, fullDutyCycle);
  analogWrite(motor3PWM, fullDutyCycle);
  analogWrite(motor4PWM, fullDutyCycle);
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
  if ( LT_PHASE_A ) val = 3;
  if ( LT_PHASE_B ) val ^= 1; // convert gray to binary
  diff = lastLt - val; // difference last - new
  if ( diff & 1 ) { // bit 0 = value (1)
    lastLt = val; // store new as next last
    encDeltaLt += (diff & 2) - 1; // bit 1 = direction (+/-)
  }
  val = 0;                                      // ?
  if ( RT_PHASE_A ) val = 3;
  if ( RT_PHASE_B ) val ^= 1; // convert gray to binary
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
  if (LT_PHASE_A) val = 3;                    // works correct
  if (LT_PHASE_B) val ^= 1;   
  lastLt = val;
  encDeltaLt = 0;
  
  val = 0;
  if (RT_PHASE_A) val = 3;  
  if (RT_PHASE_B) val ^= 1;
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
  ByteCount =  Serial.readBytesUntil('\0',Buffer,bSize);  
  if (ByteCount  > 0) strcpy(Command,strtok(Buffer,"\0"));   
  memset(Buffer, 0, sizeof(Buffer));   // Clear contents of Buffer
  Serial.flush();
}

void setup() {

  Serial.begin(250000);
  Serial.setTimeout(10);
  pinMode(ledPin, OUTPUT);

  // Encoder software initiation
  // ************************************************************************************************************
  QuadratureEncoderInit();


}

void loop() {

  // LED heart beat
  // *********************************************************************************************************************************
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin by XOR
  delay(250);   // regulation of cycle time because of compass intermediateAngle calculation

  // Read encoders
  // *************************************************************************************************************************************
  encLt += QuadratureEncoderReadLt(); 
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

  // Get command from USB interface
  // *************************************************************************************************************************************

  SerialParser();
  if (ByteCount  > 0) {
    CommandString = Command;
    if (CommandString.startsWith("Stop")) forwardStopCommand = true;
    if (CommandString.startsWith("ForwardSlow")) forwardSlowCommand = true;
    if (CommandString.startsWith("ForwardHalf")) forwardHalfCommand = true;
    if (CommandString.startsWith("ForwardFull")) forwardFullCommand = true;
    if (CommandString.startsWith("EncoderReset")) {encLt = 0; encRt = 0;}           // Encoder reset
  } 

}
