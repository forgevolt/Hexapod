// ---------------------------------------------------------------------------------------------
// SetID —  Scans the bus for Dynamixel servos, detects their current baud rate, and 
//          changes its ID to a desired value.
//           
//          Christoph Streit - 2025
// ---------------------------------------------------------------------------------------------

#include <Dynamixel2Arduino.h>
#include <Streaming.h>

//using namespace ControlTableItem;
using namespace std;

// #define DXL_SERIAL   Serial2 // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
#define DXL_SERIAL   Serial1 // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
const int   cDXLDirPin          = 16; // 26; 
const int   cRX                 = 18; // 16;
const int   cTX                 = 17;
const float cDXLProtocolVersion = 2.0;

const int cBaudRates[] = { 9600, 57600, 115200, 1000000, 2000000, 3000000, 4000000, 4500000 };
const int cNumBauds    = sizeof(cBaudRates)/sizeof(cBaudRates[0]);

const uint8_t cMinID = 0;
const uint8_t cMaxID = 253;

Dynamixel2Arduino dxl(DXL_SERIAL, cDXLDirPin);

bool notFound = true;
uint8_t foundID;
int foundBaud = 57600;

void setup() 
{
  Serial.begin(115200);
  while (!Serial);
    delay(500);  

  Serial << "\n\nSW version from " << __DATE__ << " " << __TIME__ << endl;
  
  esp_log_level_set("*", ESP_LOG_VERBOSE);
   
  dxl.begin(57600);
  DXL_SERIAL.begin(57600, SERIAL_8N1, cRX, cTX); // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
  dxl.setPortProtocolVersion(cDXLProtocolVersion);

  Serial << "Searching Dynamixel servo ..."       << endl 
         << "-----------------------------------" << endl; 

  // For all baud rates
  for (int b = 0; b < cNumBauds && notFound == true; b++) 
  {
    int baud = cBaudRates[b];
    
    Serial << "Trying baud: " << baud << endl; 

    dxl.begin(baud);
    DXL_SERIAL.begin(baud, SERIAL_8N1, cRX, cTX); // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
    delay(100);

    // Scanning all servos
    for (uint8_t id = cMinID; id <= cMaxID; id++) 
    {
      if (dxl.ping(id) == true) 
      {
        Serial << "FOUND SERVO — ID: " << id << " at baud: " << baud << endl;
        foundID = id;
        foundBaud = baud;
        notFound = false;
        break;
      }
    }
  }

  if (notFound == true)
  {
    Serial << endl << "No servo found" << endl;
  }
  else 
  {
    Serial << "Found servo with ID: " << foundID << " using baud rate " << foundBaud << endl;
  }
}


void loop() 
{
  Serial << "Enter new ID: ";

  while (true)
  {
    if (Serial.available() > 0) 
    {
      int id = Serial.parseInt();
      // Read the newline. That's the end of the entry
      Serial.read();
      Serial << id << endl;
   
      if (id < cMinID || id > cMaxID)
      {
        Serial << endl << "*** ID is out of range [" << cMinID << ", " << cMaxID << "] ***" << endl;
      }
      else 
      {
        Serial << "Changing ID to " << id << endl;
        dxl.torqueOff(foundID);
        if (dxl.setID(foundID, id) == true)
        {
          foundID = id;
          Serial << "ID has been successfully changed to " << id << endl;
          break;
        }
        else 
        {
          Serial << "Failed to change ID to " << id << endl
                 << "Trying again ..." << endl;
          ESP.restart();
        }
      }
    }
  }

  bool ledState = false;

  dxl.begin(foundBaud);
  
  while (true)
  {
    ledState = !ledState;
    ledState ? dxl.ledOn(foundID) : dxl.ledOff(foundID);
    delay(300);
  }
}