// ---------------------------------------------------------------------------------------------
// Position — Changes the position of a single servo with a smooth transition between points 
//            without stopping (decelerating).
//            Method: way points, blend threshold, dxl.getPresentPosition to check if target 
//                    reached
//           
//            Christoph Streit - 2025
// ---------------------------------------------------------------------------------------------

#include <Dynamixel2Arduino.h>
#include <Streaming.h>

using namespace ControlTableItem;
using namespace std;

#define DXL_SERIAL Serial2 // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
const int   cDXLDirPin          = 26; 
const int   cRX                 = 16;
const int   cTX                 = 17;
const float cDXLProtocolVersion = 2.0;

const uint8_t cDXL_ID = 12;

// --- PATH SETTINGS ---
// A list of positions (0 - 4095 for standard resolution)
const int32_t waypoints[] = { 0, 100, 200, 300, 400, 500, 1000, 2000, 2500, 3000, 3500, 4000 }; 
const int total_points = sizeof(waypoints) / sizeof(waypoints[0]);

int current_index = 0;

// The speed you want to maintain (RPM or units depends on model, usually 0.229 RPM/unit)
// 200 is a moderate speed. Max varies by model.
#define TRAVEL_SPEED  0 

// How close (in encoder ticks) we must be to the target before sending the NEXT target
// Increasing this makes the curve smoother but cuts the corner more.
#define BLEND_THRESHOLD 90 


Dynamixel2Arduino dxl(DXL_SERIAL, cDXLDirPin);

// Address Table for X-series (Check e-manual for your specific model)
// These are standard for XM/XL/XC/XH series
const uint16_t ADDR_OPERATING_MODE = 11;
const uint16_t ADDR_CURRENT_LIMIT  = 38;
const uint16_t ADDR_TORQUE_ENABLE  = 64;
const uint16_t ADDR_POSITION_P_GAIN= 84; 
const uint16_t ADDR_PROFILE_ACCEL  = 108;
const uint16_t ADDR_PROFILE_VEL    = 112;
const uint16_t ADDR_GOAL_POSITION  = 116;
const uint16_t ADDR_PRESENT_POS    = 132;


void setup() 
{
  Serial.begin(115200);
  while (!Serial);
    delay(500);  

  Serial << "\n\nSW version from " << __DATE__ << " " << __TIME__ << endl;
  
  esp_log_level_set("*", ESP_LOG_VERBOSE);
   
  // 1. Serial Setup
  dxl.begin(2000000);
  DXL_SERIAL.begin(2000000, SERIAL_8N1, 16, 17); // In Core 3.x, the default pins are 26 and 27. In Core 2.x, the defaults were 16 and 17.
  dxl.setPortProtocolVersion(cDXLProtocolVersion);

  dxl.ping(cDXL_ID);

  // 2. Turn off torque to configure EEPROM settings
  // dxl.writeControlTableItem(ADDR_TORQUE_ENABLE, DXL_ID, 0);
  dxl.torqueOff(cDXL_ID);

  // 3. Set Operating Mode to Position Control (Value 3)
  dxl.writeControlTableItem(ADDR_OPERATING_MODE, cDXL_ID, 3);

  // 4. MAXIMIZE TORQUE SETTINGS
  // Get the model's maximum rated current (from EEPROM)
  // Note: Be careful. Running at max torque for long periods generates heat.
  // We read the Max Current Limit (Addr 38 is usually the limit setting, 
  // we can read the read-only Max Limit at Addr 16/Model Spec, but here we assume a high value).
  // For an XM430, 1193 is usually max current. We set it high.
  // !!! dxl.writeControlTableItem(ADDR_CURRENT_LIMIT, cDXL_ID, 1193); 

  // Increase Stiffness (P-Gain). Default is often 800. 
  // Higher = more torque to correct small errors, but can oscillate.
  // !!! dxl.writeControlTableItem(ADDR_POSITION_P_GAIN, cDXL_ID, 2000); 

  // 5. MOTION PROFILE SETUP (Crucial for smooth movement)
  // Set Profile Acceleration to 0. 
  // 0 means "Infinite Acceleration" (or max possible). 
  // This ensures the servo does not ramp up/down speed, but jumps to target speed.
  //dxl.writeControlTableItem(ADDR_PROFILE_ACCEL, cDXL_ID, 0);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, cDXL_ID, 0);
  
  // Set the Constant Velocity we want to travel at
  dxl.writeControlTableItem(PROFILE_VELOCITY, cDXL_ID, TRAVEL_SPEED);

  // 6. Enable Torque
  // dxl.writeControlTableItem(ADDR_TORQUE_ENABLE, DXL_ID, 1);
  dxl.torqueOn(cDXL_ID);

  // Set the Constant Velocity we want to travel at
  //dxl.writeControlTableItem(PROFILE_VELOCITY, cDXL_ID, 50);


  Serial.println("Setup Complete. Starting Motion...");
}

void loop() 
{
  static bool move_started = false;
  static int numMoves = 0;

  if (numMoves < 30)
  {
    int32_t current_target = waypoints[current_index];

    // 1. Initiate the move if we haven't yet
    if (!move_started) {
      Serial.print("Moving to waypoint: "); Serial.println(current_target);
      dxl.setGoalPosition(cDXL_ID, current_target);
      move_started = true;
    }

    // 2. Check where we are
    int32_t current_pos = dxl.getPresentPosition(cDXL_ID);
    
    // 3. Calculate distance to target
    int32_t distance_left = abs(current_target - current_pos);

    // 4. "The Hand-off": If we are close to the target, switch to the next one IMMEDIATELY
    // This prevents the internal profile form triggering the deceleration phase.
    // if ((dxl.readControlTableItem(MOVING_STATUS, cDXL_ID)&1) == 1) { 
    if (distance_left < BLEND_THRESHOLD) {
      
      // Advance index
      numMoves++;
      current_index++;
      if (current_index >= total_points) {
        current_index = 0; // Loop back to start
      }
      
      // Update target for next loop iteration
      move_started = false; // Flag to send new command immediately
    }
  }
  else 
  {
    dxl.torqueOff(cDXL_ID);
  }

}