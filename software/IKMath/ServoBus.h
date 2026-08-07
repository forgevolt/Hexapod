#pragma once

#include <Dynamixel2Arduino.h>
#include <atomic>
#include <Streaming.h>
#include "MathUtilities.h"

// ---- Helper ----------------------------------------------------------------------------

// Convert angle (rad) to servo ticks
inline int32_t radToTick(float rad)
{
  // -90 deg / -PI/2 rad = 1024
  //   0 deg / 0 rad     = 2048
  // +90 deg / PI/2 rad  = 3072
  
  // Ensure that rad is in [−π, π]
  rad = wrapPi(rad);

  int32_t tick = static_cast<int32_t>(2048.0f + rad/cTwoPI*4096.0f);

  // 12-bit resolution: valid range is [0, 4095]. wrapPi()'s inclusive upper bound
  // (+π) maps to 4096, one past the top of that range, so this still needs a clamp
  // even though rad itself is already in range.
  return constrain(tick, 0, 4095);
}

// Convert servo ticks to angle (rad)
inline float tickToRad(int32_t tick)
{
  tick = constrain(tick, 0, 4095);
  return static_cast<float>((tick - 2048.0f) * cTwoPI / 4096.0f);
}


// ---- ServoBus --------------------------------------------------------------------------
// The ServoBus class provides a centralized interface for configuring and controlling 
// a group of Dynamixel servos on a single bus. It is responsible for:
//
//   - Initializing the Dynamixel communication interface (baud rate, protocol version, 
//     operating mode).
//   - Performing high-performance synchronous writes (SyncWrite) to update
//     all servos simultaneously, enabling precise multi-servo motion at
//     high update rates (e.g., 200 Hz).
//   - Performing high-performance synchronous reads (SyncRead) for position,
//     velocity, or current feedback if needed.
//
// Hardware: 18 x Dynamixel XC430-W240-T on a single half-duplex bus, powered from a 3S LiPo.
//   Gear ratio     245.22 : 1
//   Stall torque   1.9 [N.m]      (at 12.0 [V])
//   No load speed   70 [rev/min]  (at 12.0 [V]) -> 70 * 4096 / 60 = 4'778 ticks/sec
//   Input voltage  6.5 ~ 14.8 [V] (recommended 12.0 [V])
//
// That ticks/sec figure is the ceiling on how fast a joint can actually follow a command: at
// the 200 Hz control rate it works out to ~24 ticks (~2.1 deg) per tick. Ask for more than
// that in one tick and the servo, not the trajectory, becomes the limiting factor.
//
// Torque: begin() configures every servo but deliberately leaves torque OFF - it never
// calls torqueOn() itself. This is intentional: enabling torque makes a servo hold
// whatever is in its own onboard Goal Position register, which is stale leftover state
// from before this boot, not anything this class has written. The caller is responsible 
// for reading the servo's actual present position, pushing that as the goal via 
// setGoalPosition()/syncWrite(), and only then calling setSafeTorque()/setMaxTorque() - so 
// torque never gets enabled against an unknown target.

class ServoBus
{
  public:
    // Number of servos on the bus.
    static constexpr int cNumServos = 18;

    // Configures half-duplex UART communication for the servo bus.
    //   dirPin - Transceiver direction control pin (TX/RX toggle)
    //   rxPin, txPin - UART pins
    ServoBus(int dirPin, int rxPin, int txPin);

    // Initializes the Dynamixel communication interface and configures the operating mode
    // and various parameters of each servo. Leaves torque OFF for every servo - see the
    // class-level comment above for why. Returns true if successful, false otherwise
    // (including if any single servo's configuration write fails - see class comment).
    bool begin();

    // configureServo() must be called for each servo before calling begin().
    void configureServo(int index, uint8_t id, int32_t minPos, int32_t maxPos);

    void setMaxTorque();
    void setSafeTorque();
    void setTorqueOff();

    // Checks if torque is currently enabled for every servo.
    // Returns true only if torque is ON for ALL servos, false if even one is off (or
    // if servos aren't configured at all). 
    bool isAllTorqueOn();

    void setGoalPosition(int index, int32_t goalPos);
    void setGoalPositionRad(int index, float rad);

    uint8_t id(int index) const;
    int32_t presentPosition(int index) const;
    int32_t currentGoal(int index) const;

    bool syncWrite();                // fast write goal positions to all servos on the bus
    bool syncReadPresentPosition();  // fast read present position for all servos on the bus

    // Conditions the control task detects but cannot usefully act on.
    struct Diagnostics
    {
      uint32_t clampedGoals   = 0; // servo goals clamped to the configured joint limits
      uint32_t syncWriteFails = 0;

      // Worst servo-goal violation since the last fetch, in ticks, and which servo index.
      // Meaningless when clampedGoals == 0.
      int32_t worstGoalOvershoot = 0;
      int     worstGoalServo     = -1;
    };

    // Returns the counts accumulated since the last call, resetting them. Call from loop().
    Diagnostics fetchDiagnostics();

  private:
    // Range-checks index against [0, cNumServos), logging and returning false if not.
    // Centralizes the bounds check that used to be duplicated in five separate methods.
    bool isValidIndex(int index) const;

    // Writes an EEPROM control-table item (address < 64) only when the servo does not
    // already hold `value`. EEPROM survives power cycles and has a finite write endurance,
    // and these values are identical on every boot, so comparing first avoids spending a
    // write cycle per servo per power-up. The read-back also doubles as a check that the
    // servo is present and answering.
    // Requires torque to be OFF, which begin() already ensures.
    bool writeEEPROMItemIfChanged(uint8_t item, uint8_t id, int32_t value);

    // Sets Goal PWM on every servo with one SyncWrite, then enables torque with a single
    // broadcast write - two packets instead of 36 blocking round-trips. goalPWMRaw is in units
    // of PWM Limit, which begin() pins so the value means a known percentage.
    void setTorque(int16_t goalPWMRaw);

  private:
    Dynamixel2Arduino myDXL;
    const int myRXPin, myTXPin;

    struct ServoInfo
    {
      bool isConfigured = false;
      uint8_t id = 0;
      int32_t minPos = 0, maxPos = 0;
      int32_t goalPos = 2048;    // center/neutral tick, so an accidental early syncWrite()
      int32_t presentPos = 2048; // (torque is off at that point regardless - see class
                                  // comment) writes a harmless, defined value rather than
                                  // whatever uninitialized memory happened to contain.
    };

    bool myAllServosAreConfigured = false;
    ServoInfo myServos[cNumServos];

    // Atomic: incremented on the control task, exchanged to 0 from loop().
    std::atomic<uint32_t> myClampedGoals{0};
    std::atomic<uint32_t> mySyncWriteFails{0};

    // Worst-case detail for the clamp counter. Plain members, not atomic: only the control task
    // writes them and only loop() reads them, and the worst consequence of a torn read is one
    // slightly-wrong diagnostic figure - not worth a lock in the 200 Hz path.
    int32_t myWorstGoalOvershoot = 0;
    int     myWorstGoalServo     = -1;
    
    // ---- Data buffers and structures for SyncWrite/SyncRead operations
    struct GoalPosition
    {
      int32_t goal_position;
    };

    GoalPosition mySyncWriteData[cNumServos];
    DYNAMIXEL::InfoSyncWriteInst_t mySyncWriteInfos;
    DYNAMIXEL::XELInfoSyncWrite_t mySyncWriteXels[cNumServos];    

    struct PresentPosition
    {
      int32_t present_position;
    };

    PresentPosition mySyncReadData[cNumServos];
    DYNAMIXEL::InfoSyncReadInst_t mySyncReadInfos;
    DYNAMIXEL::XELInfoSyncRead_t mySyncReadXels[cNumServos];

    // Separate SyncRead descriptor for Torque Enable, used by isAllTorqueOn(). A second set is
    // needed because the address and payload width differ from the present-position read above.
    struct TorqueEnable
    {
      uint8_t torque_enable;
    };

    TorqueEnable myTorqueReadData[cNumServos];
    DYNAMIXEL::InfoSyncReadInst_t myTorqueReadInfos;
    DYNAMIXEL::XELInfoSyncRead_t myTorqueReadXels[cNumServos];

    // SyncWrite descriptor for Goal PWM, used by setTorque(). Separate from the goal-position
    // write above because the address and payload width differ.
    struct GoalPWM
    {
      int16_t goal_pwm;
    };

    GoalPWM myPWMWriteData[cNumServos];
    DYNAMIXEL::InfoSyncWriteInst_t myPWMWriteInfos;
    DYNAMIXEL::XELInfoSyncWrite_t myPWMWriteXels[cNumServos];
};
