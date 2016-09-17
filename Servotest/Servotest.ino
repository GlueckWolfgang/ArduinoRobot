// Definitions
// *************************************************************************************************************************
// *************************************************************************************************************************

#include <Servo.h>

// Servo
// *************************************************************************************************************************
Servo steeringservo;
#define ServoPWM 7
#define ServoMiddle 90
#define ServoExtreme 35
#define ServoLeft ServoMiddle-ServoExtreme
#define ServoRight ServoMiddle+ServoExtreme
#define ServoPmin 850
#define ServoPmax 2000

// Others
// *************************************************************************************************************************
int j;
#define ledPin 13                 // LED for heart beat
#define baud 38400                // Transmission speed for serial
int testPoint1 = 0;
int testPoint2 = 0;
int testPoint3 = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
//  pinMode(distancefRightTrig, OUTPUT);
//  pinMode(distancefLeftTrig, OUTPUT);
//  pinMode(distancebLeftTrig, OUTPUT);
//  pinMode(distancebRightTrig, OUTPUT);
//  pinMode(distanceFrontTrig, OUTPUT);
//  pinMode(distanceUpTrig, OUTPUT);
//  pinMode(motor1Direction, OUTPUT);
//  pinMode(motor1PWM, OUTPUT);
//  pinMode(motor3Direction, OUTPUT);
//  pinMode(motor3PWM, OUTPUT);
  pinMode(ServoPWM, OUTPUT);
//  pinMode(CMPS11_VCC, OUTPUT);
//  pinMode(amplifier_VCC, OUTPUT);
  // Servo initiation
  // ************************************************************************************************************

  steeringservo.attach(ServoPWM, ServoPmin, ServoPmax);
  steeringservo.write(ServoMiddle);            // middle position


}

void loop() {
  // LED heart beat and cycle time control
  // *********************************************************************************************************************************
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin by XOR
  delay(500);

}
