// ---------------------------------------------------------------------------------------------
// IKMath — Standalone inverse kinematics implementation, decoupled from the hexapod robot 
//          to simplify debugging.
//
//          Christoph Streit - 2026
// ---------------------------------------------------------------------------------------------

#include <Streaming.h>
#include "ServoBus.h"
#include "MathUtilities.h"
#include "Leg.h"

constexpr int cDXLDirPin = 16; 
constexpr int cDXL_RX    = 18; 
constexpr int cDXL_TX    = 17;

ServoBus servoBus(cDXLDirPin, cDXL_RX, cDXL_TX);

const int32_t cMinCoxa  = 1000, cMaxCoxa  = 3100;
const int32_t cMinFemur = 800,  cMaxFemur = 3700;
const int32_t cMinTibia = 700,  cMaxTibia = 3000; 

Leg leftFront(LegId::LF,  servoBus, 0, 1, 2);
Leg leftMiddle(LegId::LM, servoBus, 3, 4, 5);
Leg leftRear(LegId::LR,   servoBus, 6, 7, 8);

Leg rightFront(LegId::RF,  servoBus, 9, 10, 11); 
Leg rightMiddle(LegId::RM, servoBus, 12, 13, 14);
Leg rightRear(LegId::RR,   servoBus, 15, 16, 17);


// ---------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(115200);

  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 1000))
    delay(10);

  Serial << "\n\nSW version from " << __DATE__ << " " << __TIME__ << endl;
  
  esp_log_level_set("*", ESP_LOG_VERBOSE);

  servoBus.configureServo(0, 41, cMinCoxa,  cMaxCoxa);  // LF
  servoBus.configureServo(1, 42, cMinFemur, cMaxFemur);
  servoBus.configureServo(2, 43, cMinTibia, cMaxTibia);

  servoBus.configureServo(3, 51, cMinCoxa,  cMaxCoxa);  // LM
  servoBus.configureServo(4, 52, cMinFemur, cMaxFemur);
  servoBus.configureServo(5, 53, cMinTibia, cMaxTibia);

  servoBus.configureServo(6, 61, cMinCoxa,  cMaxCoxa);  // LR
  servoBus.configureServo(7, 62, cMinFemur, cMaxFemur);
  servoBus.configureServo(8, 63, cMinTibia, cMaxTibia);

  servoBus.configureServo(9,  11, cMinCoxa,  cMaxCoxa);  // RF
  servoBus.configureServo(10, 12, cMinFemur, cMaxFemur);
  servoBus.configureServo(11, 13, cMinTibia, cMaxTibia);

  servoBus.configureServo(12, 21, cMinCoxa,  cMaxCoxa);  // RM
  servoBus.configureServo(13, 22, cMinFemur, cMaxFemur);
  servoBus.configureServo(14, 23, cMinTibia, cMaxTibia);

  servoBus.configureServo(15, 31, cMinCoxa,  cMaxCoxa);  // RR
  servoBus.configureServo(16, 32, cMinFemur, cMaxFemur);
  servoBus.configureServo(17, 33, cMinTibia, cMaxTibia);

  servoBus.begin();
  servoBus.setSafeTorque();
  servoBus.syncReadPresentPosition();  
}

// ---------------------------------------------------------------------------------------------
void loop() 
{
// Sitting 2048, 3213, 2351
//  LF  208.94,  135.67, 41.60
//  LM  0.00,    238.13, 41.60
//  LR  -208.94, 135.67, 41.60
// Standing 1 2048, 2274, 1932
//  LF  247.70, 160.85,  -85.22
//  LM  0.00,   284.35,  -85.22
//  LR  -247.70, 160.85, -85.22
// Standing 2 2048, 2426, 2222
//  LF  201.14,  130.61, -75.33
//  LM  0.00,    228.84, -75.33
//  LR  -201.14, 130.61, -75.33


  // Sitting 
  leftFront.setPosition(Vector3(227, 147.5, 50));
  leftMiddle.setPosition(Vector3(0,  260, 50));
  leftRear.setPosition(Vector3(-227, 147.5, 50));

  rightFront.setPosition(Vector3(227, -147.5, 50));
  rightMiddle.setPosition(Vector3(0,  -260, 50));
  rightRear.setPosition(Vector3(-227, -147.5, 50));

  // Standing
  // leftFront.setPosition(Vector3(227, 147.5, -90));
  // leftMiddle.setPosition(Vector3(0,  260, -90));
  // leftRear.setPosition(Vector3(-227, 147.5, -90));

  // rightFront.setPosition(Vector3(227, -147.5, -90));
  // rightMiddle.setPosition(Vector3(0,  -260, -90));
  // rightRear.setPosition(Vector3(-227, -147.5, -90));

  // Standing twisted
  // leftFront.setPosition(Vector3(186, 150, -90));
  // leftMiddle.setPosition(Vector3(0,  229, -90));
  // leftRear.setPosition(Vector3(-186, 150, -90));

  // rightFront.setPosition(Vector3(186, -150, -90));
  // rightMiddle.setPosition(Vector3(0,  -229, -90));
  // rightRear.setPosition(Vector3(-186, -150, -90));

  // 10 deg twisted -> more reach
  // rightFront.setPosition(Vector3(209, -109, -75));
  // rightRear.setPosition(Vector3(-209, -109, -75));

  // Standing wider
  // leftFront.setPosition(Vector3(243, 158, -75));
  // leftMiddle.setPosition(Vector3(0, 279, -75));
  // leftRear.setPosition(Vector3(-243, 158, -75));

  // rightFront.setPosition(Vector3(243, -158, -75));
  // rightMiddle.setPosition(Vector3(0, -279, -75));
  // rightRear.setPosition(Vector3(-243, -158, -75));  

  servoBus.syncWrite();    

  Serial << leftFront.computePositionFromAngles(0*deg2rad, 25.91*deg2rad, -4.98*deg2rad) << endl;
  Serial << leftMiddle.computePositionFromAngles(0*deg2rad, 25.91*deg2rad, -4.98*deg2rad) << endl;
  Serial << leftRear.computePositionFromAngles(0*deg2rad, 25.91*deg2rad, -4.98*deg2rad) << endl;

  Serial << rightFront.computePositionFromAngles(0*deg2rad, 100.12*deg2rad, 16.26*deg2rad) << endl;
  Serial << rightMiddle.computePositionFromAngles(0*deg2rad, 100.12*deg2rad, 16.26*deg2rad) << endl;
  Serial << rightRear.computePositionFromAngles(0*deg2rad, 100.12*deg2rad, 16.26*deg2rad) << endl;


  while (true)
  {
    // stay here, do not continue; uncomment for the test movement below
  }

  // Test simple leg movement
  for (int y=240; y<=280; y++)
  {
    leftFront.setPosition(Vector3(leftFront.getOffset().x, y, 0));
    leftMiddle.setPosition(Vector3(leftMiddle.getOffset().x, y, 0));
    leftRear.setPosition(Vector3(leftRear.getOffset().x, y, 0));

    rightFront.setPosition(Vector3(rightFront.getOffset().x, -y, 0));
    rightMiddle.setPosition(Vector3(rightMiddle.getOffset().x, -y, 0));
    rightRear.setPosition(Vector3(rightRear.getOffset().x, -y, 0));

    servoBus.syncWrite();    
    delay(20);
  }


  for (int x=0; x<=60; x++)
  {
    leftFront.setPosition(Vector3(leftFront.getOffset().x + x, 280, 0));
    leftMiddle.setPosition(Vector3(leftMiddle.getOffset().x + x, 280, 0));
    leftRear.setPosition(Vector3(leftRear.getOffset().x + x, 280, 0));

    rightFront.setPosition(Vector3(rightFront.getOffset().x + x, -280, 0));
    rightMiddle.setPosition(Vector3(rightMiddle.getOffset().x + x, -280, 0));
    rightRear.setPosition(Vector3(rightRear.getOffset().x + x, -280, 0));

    servoBus.syncWrite();    
    delay(20);
  }

  for (int x=60; x>=0; x--)
  {
    leftFront.setPosition(Vector3(leftFront.getOffset().x + x, 280, 0));
    leftMiddle.setPosition(Vector3(leftMiddle.getOffset().x + x, 280, 0));
    leftRear.setPosition(Vector3(leftRear.getOffset().x + x, 280, 0));

    rightFront.setPosition(Vector3(rightFront.getOffset().x + x, -280, 0));
    rightMiddle.setPosition(Vector3(rightMiddle.getOffset().x + x, -280, 0));
    rightRear.setPosition(Vector3(rightRear.getOffset().x + x, -280, 0));

    servoBus.syncWrite();    
    delay(20);
  }

  for (int y=280; y>=240; y--)
  {
    leftFront.setPosition(Vector3(leftFront.getOffset().x, y, 0));
    leftMiddle.setPosition(Vector3(leftMiddle.getOffset().x, y, 0));
    leftRear.setPosition(Vector3(leftRear.getOffset().x, y, 0));

    rightFront.setPosition(Vector3(rightFront.getOffset().x, -y, 0));
    rightMiddle.setPosition(Vector3(rightMiddle.getOffset().x, -y, 0));
    rightRear.setPosition(Vector3(rightRear.getOffset().x, -y, 0));

    servoBus.syncWrite();    
    delay(20);
  }
}

