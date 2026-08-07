// ---------------------------------------------------------------------------------------------
// PresentPosition — Connects to one leg (three servos) and prints the present positions, 
//                   and min/max for coxa, femur and tibia to Serial
//           
//                   Christoph Streit - 2025
// ---------------------------------------------------------------------------------------------

#include <Dynamixel2Arduino.h>
#include <Streaming.h>

using namespace ControlTableItem;
using namespace std;

// #define DXL_SERIAL Serial2 
#define DXL_SERIAL Serial1 

constexpr int cDXLDirPin = 16; 
constexpr int cRX        = 18; 
constexpr int cTX        = 17;

const float cDXLProtocolVersion = 2.0;

const uint8_t cDXL_ID_COXA  = 21; // RM
const uint8_t cDXL_ID_FEMUR = 22;
const uint8_t cDXL_ID_TIBIA = 23;

Dynamixel2Arduino dxl(DXL_SERIAL, cDXLDirPin);

float minCoxa  = 2048, maxCoxa  = 2048;
float minFemur = 2048, maxFemur = 2048;
float minTibia = 2048, maxTibia = 2048;

float coxa, femur, tibia;

void setup() 
{
  Serial.begin(115200);

  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 1000))
    delay(10);

  Serial << "\n\nSW version from " << __DATE__ << " " << __TIME__ << endl;
  
  esp_log_level_set("*", ESP_LOG_VERBOSE);
   
  dxl.begin(3000000);
  DXL_SERIAL.begin(3000000, SERIAL_8N1, cRX, cTX); 
  dxl.setPortProtocolVersion(cDXLProtocolVersion);

  dxl.torqueOff(cDXL_ID_COXA);
  dxl.setOperatingMode(cDXL_ID_COXA, OP_POSITION);

  dxl.torqueOff(cDXL_ID_FEMUR);
  dxl.setOperatingMode(cDXL_ID_FEMUR, OP_POSITION);

  dxl.torqueOff(cDXL_ID_TIBIA);
  dxl.setOperatingMode(cDXL_ID_TIBIA, OP_POSITION);

  Serial.println("Setup Complete. Starting ...");
}


void loop() 
{
  coxa  = dxl.getPresentPosition(cDXL_ID_COXA);
  femur = dxl.getPresentPosition(cDXL_ID_FEMUR);
  tibia = dxl.getPresentPosition(cDXL_ID_TIBIA);
  
  minCoxa  = min(coxa, minCoxa);
  maxCoxa  = max(coxa, maxCoxa);
  minFemur = min(femur, minFemur);
  maxFemur = max(femur, maxFemur);
  minTibia = min(tibia, minTibia);
  maxTibia = max(tibia, maxTibia);
  
  Serial << "C:"   << coxa  << " minC:" << minCoxa  << " maxC:" << maxCoxa
          << " F:" << femur << " minF:" << minFemur << " maxF:" << maxFemur
          << " T:" << tibia << " minT:" << minTibia << " maxT:" << maxTibia
          << endl;

  delay(300);
}
