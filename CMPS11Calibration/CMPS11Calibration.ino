//***********************************************************************************
// *** Arduino CMPS11 test- and calibration program
// *** Version: 2016.10.17
// *** Developer: Wolfgang Glück
// ***
// *** Supported hardware:
// ***   - Arduino mega   (Spider)
// ***   - Compass        (CMPS11 via 10cm I2C bus, using internal pullups by default)
// *** Commands:
// ***   - Read           (default after start)
// ***   - Calibrate      (waits for Stop)
// ***   - Stop           (turns back to Read after stopping calibration)
// *** After starting calibration:
// *** Turn the robot (CMPS11) around 360 degrees slowly for all 3 axis
// *** keep the robot calm for 1 second on every edge (6)
// *** then stop calibration
// *** Attention!
// *** For calibration use an area like a garden without disturbing iron materials or magnetic fields
// *** In addition you need a deviation table for each room where the robot should be used

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

// Communication
// *************************************************************************************************************************
boolean usbDisturbance = false;   // if USB interface is not ready

// Others
// *************************************************************************************************************************
int i;
#define ledPin 13                 // LED for heart beat
#define baud 38400                // Transmission speed for serial
String status = "read";

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
void setup() {

  pinMode(ledPin, OUTPUT);
  pinMode(CMPS11_VCC, OUTPUT);
   
  // CMPS11 startup
  // ************************************************************************************************************
  digitalWrite(CMPS11_VCC, 1);
  // Wait for CMPS has been restarted
  delay(2000);
  Wire.begin();
  
  Serial.begin(baud);
  Serial.setTimeout(10);
}

void loop() {
  // LED heart beat and cycle time control
  // *********************************************************************************************************************************
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin by XOR
  // regulation of cycle time because of intermediateAngle calculation  total cycletime should be <= 500 ms
  delay(400);
  
  if (status == "calibrate"){
    // Start Calibration
    // ************************************************************************************************************
  
    Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
    Wire.write(byte(0x00));                  //set register pointer to command register
    Wire.write(CMPS11_Byte1);                //Send command
    Wire.endTransmission();
    delay(20);
  
    Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
    Wire.write(byte(0x00));                  //set register pointer to command register
    Wire.write(CMPS11_Byte2);                //Send command
    Wire.endTransmission();
    delay(20);
  
    Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
    Wire.write(byte(0x00));                  //set register pointer to command register
    Wire.write(CMPS11_Byte3);                //Send command
    Wire.endTransmission();
    delay(20);
    status = "wait";
  }
  
  if (status == "stop"){
    // Stop calibration
    // ****************
  
    Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
    Wire.write(byte(0x00));                  //set register pointer to command register
    Wire.write(CMPS11_Byte4);                //Send command
    Wire.endTransmission();
    delay(20);
    status = "read";
  }

  if (status == "read"){
    // Read CMPS11 values
    // ***********************************************************************************************************************************
  
    Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
    Wire.write(ANGLE_8);                     //Sends the register we wish to start reading from
    Wire.endTransmission();
    delay(20);
  
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

    if (Serial) { usbDisturbance = false;                                            // USB ok
      Serial.print ("MV@Version: V ");
      Serial.println(ARDUINO);
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
    }
    else {usbDisturbance = true;                                                        // USB disturbance
      Serial.end();
      Serial.begin(baud);
      Serial.setTimeout(10);
      delay(2000);                                                                      // wait for next trial
    }
  } 
      
    // Get command from USB interface
    // *************************************************************************************************************************************
  
  SerialParser();
  if (ByteCount  > 0) {
    CommandString = Command;
    CommandStringS = CommandString;

    if (CommandString.startsWith("Read")) {
      status = "read";
    }
    if (CommandString.startsWith("Stop")) {
      status = "stop";
    }
    if (CommandString.startsWith("Calibrate")) {
      status = "calibrate";
    }
  }        
}
