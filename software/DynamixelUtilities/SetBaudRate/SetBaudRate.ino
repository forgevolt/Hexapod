// ---------------------------------------------------------------------------------------------
// SetBaudRate — Scans the bus for Dynamixel servos, detects their current baud rate, and 
//               updates it to the desired value.
//           
//               Christoph Streit - 2025
// ---------------------------------------------------------------------------------------------

#include <Dynamixel2Arduino.h>
#include <Streaming.h>
#include <vector>

using namespace std;

//#define DXL_SERIAL Serial2 // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
#define DXL_SERIAL Serial1 // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
const int   cDXLDirPin          = 16; // 26; 
const int   cRX                 = 18; // 16;
const int   cTX                 = 17;
const float cDXLProtocolVersion = 2.0;

const int cBaudRates[] = { 9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000 };
const int cNumBauds    = sizeof(cBaudRates)/sizeof(cBaudRates[0]);

const uint8_t cMinID = 0;
const uint8_t cMaxID = 253;

int cTargetBaudRate = 3000000; // Set this to the desired value; valid options are listed in cBaudRates.

Dynamixel2Arduino dxl(DXL_SERIAL, cDXLDirPin);

vector<uint8_t> foundIDs;

void setup() 
{
  Serial.begin(115200);
  while (!Serial);
    delay(500);  

  Serial << endl << "SW version from " << __DATE__ << " " << __TIME__ << endl;
  
  esp_log_level_set("*", ESP_LOG_VERBOSE);

  Serial << "Default pins:" << endl
         << "\tRX1: " << RX1 << "\tTX1: " << TX1 << endl
         << "\tRX2: " << RX2 << "\tTX2: " << TX2 << endl;

  dxl.begin(57600);
  DXL_SERIAL.begin(57600, SERIAL_8N1, cRX, cTX); // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
  dxl.setPortProtocolVersion(cDXLProtocolVersion);

  // For all baud rates
  for (int b = 0; b < cNumBauds; b++) 
  {
    int baud = cBaudRates[b];
    
    Serial << endl << "Trying baud: " << baud << endl; 

    dxl.begin(baud);
    DXL_SERIAL.begin(baud, SERIAL_8N1, cRX, cTX); // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
    delay(100);

    // Scanning all servos
    for (uint8_t id = cMinID; id <= cMaxID; id++) 
    {
      if (dxl.ping(id) == true) 
      {
        Serial << "FOUND SERVO — ID: " << id << " at baud: " << baud << endl;
        foundIDs.push_back(id);

        if (baud == cTargetBaudRate)
        {
           Serial << "Baud rate already set to " << cTargetBaudRate << endl;
        }
        else 
        {
          // Set new baud rate
          Serial << "Setting new baud rate to " << cTargetBaudRate << endl;
          dxl.torqueOff(id);
          dxl.setBaudrate(id, cTargetBaudRate);
          dxl.torqueOn(id);
        }
      }
    }
  }
}


void loop() 
{
  bool ledState = false;

  dxl.begin(cTargetBaudRate);
  
  while (true)
  {
    ledState = !ledState;

    for (auto id : foundIDs)
    {
      ledState ? dxl.ledOn(id) : dxl.ledOff(id);
    } 
    delay(300);
  }
}