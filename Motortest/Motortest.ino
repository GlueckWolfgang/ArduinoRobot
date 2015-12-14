// LED
// *************************************************************************************************************************
#define ledPin 13            // LED for heart beat

// Motor
// *******************************************************************************************************************************
// Motor right front
#define motor1Direction 30
#define motor1PWM 6
#define motor1CurrentProbe A15
int motor1RawValue = 0;
float motor1FinalValue = 0.0;
float motorStallLimit = 2.0;  // Stall limit for all motors
boolean motor1Stall = false;


// Motor control
// *******************************************************************************************************************************
boolean emergencyStop = false;
boolean forward             = 1;
boolean backward            = 0;
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
boolean turnFinished        = false; // Sent to Raspberry after turn has been finished
int stopDutyCycle           =   0;   //   0% of 256
int slowDutyCycle           =  64;   //  25% of 256
int halfDutyCycle           = 128;   //  50% of 256
int fullDutyCycle           = 255;   // 100% of 256

void MotorControl()
{
  if (emergencyStop || forwardStopCommand){         // Emergency stop or manually stop
    analogWrite(motor1PWM, stopDutyCycle);          // *******************************
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
    turnedAngle = 0;
  }
  
  if (forwardSlowCommand){                      // Forward slow
    digitalWrite(motor1Direction,forward);      // ************    
    analogWrite(motor1PWM, slowDutyCycle);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  Serial.setTimeout(10);
  pinMode(ledPin, OUTPUT);

  pinMode(motor1Direction, OUTPUT);

  pinMode(motor1PWM, OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:

  // LED heart beat
  // *********************************************************************************************************************************
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin by XOR

  forwardSlowCommand = true;

  MotorControl();

  delay(2000);
}
