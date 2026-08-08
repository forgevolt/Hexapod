#pragma once

#include "ServoBus.h"
#include "Leg.h"
#include "GaitEngine.h"
#include "Gaits.h"
#include "Receiver.h"
#include "IndicatorLeds.h"
#include "StatusDisplay.h"
#include <SoundEngine.h>
#include <MPU6050_light.h>

#include <atomic>
#include <array>
#include <limits>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


// ---- Hexapod ---------------------------------------------------------------------------
// High-level system manager for the robot, coordinating FreeRTOS tasking, state logic, 
// and power-aware servo control.
//
// This class acts as the "Brain" of the robot, integrating several sub-systems:
// - Real-Time Control: Orchestrates a dedicated FreeRTOS task (schedulerTask) pinned 
//   to Core 1, targeting a precise 200Hz update rate for smooth servo motion.
// - State Machine: Manages high-level behaviors (Initializing -> Ready -> Standing -> Walking/Posing)
//   with integrated safety transitions and automatic idle timeouts.
// - Power Management: Monitors activity and automatically disables servo torque 
//   after periods of inactivity (cWaitTimeUntilTorqueOff) to protect hardware.
// - Telemetry & Feedback: Translates internal GaitEngine metrics (Step Height, Length, 
//   Clearance) into user-readable messages sent back to the Receiver/Transmitter.

class Hexapod
{
  public:
    Hexapod(Receiver& receiver);
    ~Hexapod();

    // announceFault: sound the error tone during startup even when initialisation succeeds.
    // For conditions detected before the robot exists - e.g. an abnormal reset reason.
    bool begin(bool announceFault = false);

    // Update leg positions. dt_ms is time passed in milliseconds.    
    void step(float dt_ms);

    Leg& leg(std::size_t index);
    ServoBus& servoBus() { return myServoBus; }
    Receiver& receiver() { return myReceiver; }

    // Gait parameters, for gaits to read and for the telemetry report.
    const GaitEngine::GaitParams& gaitParams() const { return myGaitEngine.params(); }

    // Everything the control task counted but could not usefully act on, gathered from the legs
    // and the servo bus. Counts since the last call, which resets them.
    struct Diagnostics
    {
      // ---- Inverse kinematics, across all legs
      uint32_t unreachableTargets = 0;    // IK targets clamped to the reachable workspace
      float    worstOvershoot     = 0.0f; // mm; positive = too far, negative = too close
      int      worstLeg           = -1;   // idx(LegId); -1 when unreachableTargets == 0

      // ---- Servo bus
      uint32_t clampedGoals       = 0;  // goals clamped to the configured joint limits
      int32_t  worstGoalOvershoot = 0;  // ticks; meaningless when clampedGoals == 0
      int      worstGoalServo     = -1;
      uint32_t syncWriteFails     = 0;
    };

    // Call from loop(), not from the control task.
    Diagnostics fetchDiagnostics();

    // The display and the IMU share one I2C bus and are driven exclusively by an internal
    // peripheralsTask (see .cpp) - update() is called for both automatically. These
    // accessors are for configuration only (setMood(), setLookDirection(), etc.);
    // do not call imu().update() or statusDisplay().update() from outside this class, that
    // would race with peripheralsTask on the same bus.
    MPU6050& imu()                 { return myIMU;           }
    StatusDisplay& statusDisplay() { return myStatusDisplay;  }
    SoundEngine& sound()           { return mySound;          }

  private:
    ServoBus myServoBus;
    Leg myLegs[cNumLegs];

    enum class HexapodState
    {
      eOff,             // Robot in off state; on/off switch set to off; torque is off
      eInitializing,    // Servos homing to folded safety position. 
      eReady,           // Powered and stable; chassis on ground awaiting "Stand" command.
      eStanding,        // Active stance; legs deployed, IK keeping chassis level and upright.
      eStandingLeveled, // Feet planted; onboard IMU actively counters chassis tilt on uneven ground.
      ePosing,          // Feet planted; adjusting shifting body Roll/Pitch/Yaw and XYZ offsets.
      eWalking          // Gait cycle active; coordinating swing/stance phases across all legs.
                        // Dynamically adjusting step height and length and ground clearance.
    };

    // step() methods for each state. They handle operator input and state transitions and take
    // no time delta: time integration belongs to GaitEngine::step(), and the handlers that need
    // time read myTimePassedMS, which step() accumulates.
    void stepOff();
    void stepInitializing();
    void stepReady();
    void stepStanding();
    void stepStandingLeveled();
    void stepPosing();
    void stepWalking();

    void changeState(HexapodState newState);
    const char* toString(HexapodState state);

    // Initializes and manages the servo power enable switch
    void initializeSwitch();
    bool isSwitchOn() const;

    void changeEyeConfig(HexapodState newState);

    // Common handling for "transmitter disconnected or power switch turned off",
    void handleConnectionLoss(bool requestParkGait);

    // Forwards to the receiver only when the labels actually change.
    void setButtonLabels(const char* btn1, const char* btn2);

    // True when it is safe to act on myControlData: the transport layer reports a peer, and
    // a command has arrived recently enough to still be meaningful.
    bool isLinkHealthy();

    HexapodState myState = HexapodState::eOff;
    float myTimePassedMS = 0.0f;

    GaitEngine myGaitEngine;
    ParkGait myParkGait;
    StandUpGait myStandUpGait;
    PosingGait myPosingGait;
    LevelGait myLevelGait;

    // Members for all the walking gait configurations
    static constexpr std::size_t cNumWalkingGaits = 3;
    std::size_t myCurrentWalkingGait = 0;
    std::array<GenericWalkingGait, cNumWalkingGaits> myWalkingGaits;

    Receiver& myReceiver;
    Receiver::ControlData myControlData;

    char myLastBtn1[cLabelLen] = "";
    char myLastBtn2[cLabelLen] = "";

    // Telemetry cache: the values last pushed to the receiver, so step() only calls setMessage()
    // when one of them changes.
    float myLastStepHeight      = std::numeric_limits<float>::max();
    float myLastStepLength      = std::numeric_limits<float>::max();
    float myLastGroundClearance = std::numeric_limits<float>::max();
    char  myStrStepHeight[cLabelLen]      = "";
    char  myStrStepLength[cLabelLen]      = "";
    char  myStrGroundClearance[cLabelLen] = "";
    Gait* myLastGait  = nullptr;
    bool  myLastBlank = false;

    IndicatorLeds myLeds;          // LED strip on the hexapod cover
    StatusDisplay myStatusDisplay; // eyes and fault reporting on the OLED screen
    SoundEngine mySound;           // I2S based sound engine
    MPU6050 myIMU;                 // Onboard IMU, used by LevelGait to keep the body level on uneven terrain

    // schedulerTask calls Hexapod::step() in a fixed interval (target is 200 times/second)
    static void schedulerTask(void* pvParameters);
    std::atomic<bool> myIsTaskRunning;
    TaskHandle_t      myTaskHandle = nullptr;

    // Display + IMU: lower-priority task, same core, so slow I2C transfers can never
    // delay a servo update - see peripheralsTask()'s comment in Hexapod.cpp.
    static void peripheralsTask(void* pvParameters);
    std::atomic<bool> myIsPeripheralsTaskRunning;
    TaskHandle_t      myPeripheralsTaskHandle = nullptr;
};