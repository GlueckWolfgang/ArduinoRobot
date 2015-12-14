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

const int bSize = 20;
int ByteCount;
char Buffer[bSize];  // Serial buffer
char Command[18];    // Arbitrary Value for command size
String CommandString = "";

void SerialParser(void) 
{
  //  One command per line.  
  //  Command Format: "up to 18 Letter command <\n>"
  //  count will  be below Zero on a timeout.
  //  read up to X chars or until EOT - in this case "\n" 
  ByteCount = -1;
  ByteCount =  Serial.readBytesUntil('\0',Buffer,bSize);  
  if (ByteCount  > 0) strcpy(Command,strtok(Buffer,"\0"));   
  memset(Buffer, 0, sizeof(Buffer));   // Clear contents of Buffer
  Serial.flush();
}

// LED
// *************************************************************************************************************************
#define ledPin 13            // LED for heart beat

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  Serial.setTimeout(10);
  pinMode(ledPin, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  // LED heart beat
  // *********************************************************************************************************************************
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin by XOR
  delay(1000);

  if (emergencyStop)             Serial.println("EmergencyStop! ");
  if (forwardStopCommand)        Serial.println("Stop!");
  if (forwardSlowCommand)        Serial.println("ForwardSlow!"); 
  if (forwardHalfCommand)        Serial.println("ForwardHalf!");
  if (forwardFullCommand)        Serial.println("ForwardFull!");
  if (turnSlow45LeftCommand)     Serial.println("TurnSlow45Left!");
  if (turnSlow45RightCommand)    Serial.println("TurnSlow45Right!");
  if (turnSlow90LeftCommand)     Serial.println("TurnSlow90Left!");
  if (turnSlow90RightCommand)    Serial.println("TurnSlow90Right!");

  // Get command from USB interface
  // *************************************************************************************************************************************

  SerialParser();
  if (ByteCount  > 0) {
    Serial.println(Command);
    CommandString = Command;
    if (CommandString.startsWith("ForwardStop")) forwardStopCommand = true;
    if (CommandString.startsWith("ForwardSlow")) forwardSlowCommand = true;
    if (Command == "ForwardHalf") forwardHalfCommand = true;
    if (Command == "ForwardFull") forwardFullCommand = true;
    if (Command == "TurnSlow45Left") turnSlow45LeftCommand = true;
    if (Command == "TurnSlow45Right") turnSlow45RightCommand = true;
    if (Command == "TurnSlow90Left") turnSlow90LeftCommand = true;
    if (Command == "TurnSlow90Right") turnSlow90RightCommand = true;
  } 
}
