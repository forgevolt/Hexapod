// ---------------------------------------------------------------------------------------------
// DefaultPosition — Sends the default position (2048) to all servos on the bus.
//                   Useful when assembling or calibrating a hexapod leg.
//           
//                   Christoph Streit - 2026
// ---------------------------------------------------------------------------------------------

#include <Dynamixel2Arduino.h>
#include <Streaming.h>

using namespace ControlTableItem;
using namespace std;

#define DXL_SERIAL Serial2 // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
// #define DXL_SERIAL Serial1 // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
const int   cDXLDirPin          = 26; //16; // 26; 
const int   cRX                 = 16; // 18; // 16;
const int   cTX                 = 17;
const float cDXLProtocolVersion = 2.0;

const float cDefaultPosition    = 2048;

Dynamixel2Arduino dxl(DXL_SERIAL, cDXLDirPin);

void setup() 
{
  Serial.begin(115200);

  // Waits for a serial monitor on purpose, with no timeout: setup() writes to the servos
  // straight away, and this keeps it from doing so before anyone can see the output.
  while (!Serial)
    delay(10);
  delay(500);

  Serial << "\n\nSW version from " << __DATE__ << " " << __TIME__ << endl;
  
  esp_log_level_set("*", ESP_LOG_VERBOSE);
   
  dxl.begin(3000000);
  DXL_SERIAL.begin(3000000, SERIAL_8N1, cRX, cTX);
  dxl.setPortProtocolVersion(cDXLProtocolVersion);

  for (uint8_t i=0; i<100; i++)
  {
    dxl.torqueOff(i);
    dxl.setOperatingMode(i, OP_POSITION);
    dxl.writeControlTableItem(MIN_POSITION_LIMIT, i, 0);
    dxl.writeControlTableItem(MAX_POSITION_LIMIT, i, 4096);
    dxl.torqueOn(i);
    dxl.setGoalPosition(i, cDefaultPosition);
  }

  Serial << "All servos set to position " << cDefaultPosition << endl;
}

void loop() 
{}
