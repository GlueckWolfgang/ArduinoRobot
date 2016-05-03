//****************************************************************************************************************************************************
// *** US-015 Test
// *** Version: 2016.05.02
// *** Developer: Wolfgang Glück
// ***

// Communication
// *************************************************************************************************************************
boolean usbDisturbance = false;   // if USB interface is not ready

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

// Others
// *************************************************************************************************************************
int i, j;
#define ledPin 13                 // LED for heart beat
#define baud 38400                // Transmission speed for serial
int testPoint1 = 0;
int testPoint2 = 0;
int testPoint3 = 0;

void setup() {
  Serial.begin(baud);
  Serial.setTimeout(10);
}

void loop() {
  // LED heart beat and cycle time control
  // *********************************************************************************************************************************
  digitalWrite(ledPin, digitalRead(ledPin) ^ 1);   // toggle LED pin by XOR
  // regulation of cycle time because of intermediateAngle calculation  total cycletime should be <= 500 ms
  delay(400);

  // Read distances to obstructions (US-015 contains a temperature compensation)
  // *********************************************************************************************************************************
                                                                        // Front
                                                                        // *****
  for (j = 0; j < 2; j++){                                              // 2 new measures per cycle, keep 1 value from earlier cycle                                             
    digitalWrite(distanceFrontTrig, LOW);                               
    delayMicroseconds(20);                                              
    digitalWrite(distanceFrontTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(distanceFrontTrig, LOW);
    distanceFrontPulseTime = pulseIn(distanceFrontEcho, HIGH, 47200);   // Range limited to 2 x 400cm (timeout delivers 0)
    if (distanceFrontPulseTime > 60) {                                  // filter for timeout and small values
      distanceFrontCm = distanceFrontPulseTime / 59;                    // 340m/s = 29.4 us/cm for two ways
  
      for (i = 3; i > 0; i --){
        distanceFrontReg[i] = distanceFrontReg[i-1];                    // shift register
      }
      distanceFrontReg[0] = distanceFrontCm;                            // put new value to register
    }
  }
  distanceFrontCm = 0;                                                  // calculate new value
  for (i= 0; i < 3; i ++){
    distanceFrontCm = distanceFrontCm + distanceFrontReg[i];
  }
  distanceFrontCm = int(distanceFrontCm/3);                             // mean value from last 3 valid values
 
  distanceFrontObstruction = (distanceFrontCm < distanceFrontLimit);    // Obstruction detected front side


  

  // check serial interface
  // *************************************************************************************************************************************
  if (Serial) { usbDisturbance = false;                               // USB ok
    // Send values to USB interface if no turn command is running (shorten cycle time)
    // ***********************************************************************************************************************************
//    Serial.print("MV@Distance front: V ");
    Serial.println(distanceFrontCm);
//    Serial.print("MV@Distance front: LL_Exceeded ");
//    Serial.println(distanceFrontObstruction);
  }

  else {usbDisturbance = true;                                                        // USB disturbance
  Serial.end();
  Serial.begin(baud);
  Serial.setTimeout(10);
  delay(2000);                                                                        // wait for next trial
  }
}
