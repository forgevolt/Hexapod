// ---------------------------------------------------------------------------------------------
// Position — Changes the position of a single servo with a smooth transition between points 
//            without stopping (decelerating).
//            Method: Send GoalPosition with high frequency (100-200Hz)
//           
//            Christoph Streit - 2025
// ---------------------------------------------------------------------------------------------

#include <Dynamixel2Arduino.h>
#include <Streaming.h>
#include "Trajectory.h"

using namespace ControlTableItem;
using namespace std;

#define DXL_SERIAL Serial2 // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
const int   cDXLDirPin          = 26; 
const int   cRX                 = 16;
const int   cTX                 = 17;
const float cDXLProtocolVersion = 2.0;

const uint8_t cDXL_ID = 13;

// A list of positions (0 - 4095 for standard resolution)
const int32_t waypoints[] = { 0, 500, 1000, 1200, 1500, 2000, 2500, 3000, 3500, 4000, 4095 }; 
const int total_points = sizeof(waypoints) / sizeof(waypoints[0]);

Dynamixel2Arduino dxl(DXL_SERIAL, cDXLDirPin);

void setup() 
{
  Serial.begin(115200);
  while (!Serial);
    delay(500);  

  Serial << "\n\nSW version from " << __DATE__ << " " << __TIME__ << endl;
  
  esp_log_level_set("*", ESP_LOG_VERBOSE);
   
  dxl.begin(2000000);
  DXL_SERIAL.begin(2000000, SERIAL_8N1, 16, 17); // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
  dxl.setPortProtocolVersion(cDXLProtocolVersion);

  dxl.ping(cDXL_ID);

  dxl.torqueOff(cDXL_ID);
  dxl.setOperatingMode(cDXL_ID, OP_POSITION);

  dxl.writeControlTableItem(PROFILE_ACCELERATION, cDXL_ID, 0);
  dxl.writeControlTableItem(PROFILE_VELOCITY, cDXL_ID, 0);

  dxl.torqueOn(cDXL_ID);

  Serial.println("Setup Complete. Starting Motion...");
}

const float duration = 1.0;
const float updateRate = 200.0; // Hz
const float dt = 1.0 / updateRate;

float t = 0;
unsigned long lastMicros = 0;


void loop() 
{
  dxl.setGoalPosition(cDXL_ID, 0);
  delay(3000);

  Trajectory traj;
   
  for (int i=0; i<total_points-1; i++)
  {
    // traj.set(MINJERK, waypoints[i], waypoints[i+1], duration);
    traj.set(LINEAR,  waypoints[i], waypoints[i+1], duration);
    // traj.set(BEZIER3, waypoints[i], waypoints[i+1], duration, 2200, 2900);
    // traj.set(SPLINE,  waypoints[i], waypoints[i+1], duration, 50, -20);

    t = 0;
    lastMicros = 0;

    while (true)
    {
      if (micros() - lastMicros >= (1e6/updateRate)) 
      {
        lastMicros = micros();

        t += dt;
        if (t > duration) t = duration;
        
        float pos = traj.evaluate(t);

        if (pos >= waypoints[i+1])
          break;

        dxl.setGoalPosition(cDXL_ID, (int)pos);
      }
    }
  }

  dxl.torqueOff(cDXL_ID);

  while (true) ;
}
