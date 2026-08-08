#include <Streaming.h>

#include "GaitEngine.h"
#include "Gaits.h"
#include "MathUtilities.h"
#include "Leg.h"
#include "Hexapod.h"
#include "PinMap.h"

#include "sounds/Startup.h"
#include "sounds/Shutdown.h"
#include "sounds/Error.h"

#include <cmath>           // roundf
#include <cstring>         // strcmp
#include <iterator>        // std::size
#include <esp_system.h>    // esp_random()
#include <esp_task_wdt.h>  // esp_task_wdt_add() / esp_task_wdt_reset() / esp_task_wdt_delete()

// ---- Hexapod ---------------------------------------------------------------------------

const int32_t cMinCoxa  = 1200, cMaxCoxa  = 3000; 
const int32_t cMinFemur = 900,  cMaxFemur = 3500; 
const int32_t cMinTibia = 200,  cMaxTibia = 2700; 

// We target 'cTargetUpdateRate' servo position updates per second
constexpr unsigned long cTargetUpdateRate = 200;

// Display + IMU run on a separate, lower-priority task; this bounds it to the IMU's
// intended poll rate. StatusDisplay::update() self-throttles to its own configured fps,
// so this only needs to be at least as fast as the fastest thing being polled.
constexpr unsigned long cPeripheralsUpdateRate = 50;

// The MPU6050's rated maximum. StatusDisplay drives the display faster and U8g2 does not restore the
// previous value, so peripheralsTask sets this before every IMU access.
constexpr uint32_t cIMUBusClockHz = 400000;

// Transition gait durations. Parking travels the full range from the current pose to the folded
// position, and standing up from parked covers the same distance. Returning to a plain stand from
// an already-deployed pose - posing, levelling, or walking - is a shorter move.
constexpr float cParkDurationMs      = 3000.0f;
constexpr float cStandUpFromParkedMs = 3000.0f;
constexpr float cStandUpDeployedMs   = 1000.0f;

constexpr float cWaitTimeUntilTorqueOff = 60000.0; // 60 seconds
constexpr float cIdleTimeout            = 15000.0; // 15 seconds

// The control loop refuses to act on a command older than this, independently of the transport
// layer's pairing state. This is the fast, safety-critical threshold - five missed frames at the
// transmitter's 50 ms cadence. ESPNowUtilities' cPeerTimeoutMs is deliberately longer, so a brief
// dropout parks the robot without tearing down the pairing.
constexpr unsigned long cControlDataTimeoutMs = 250;

AudioClip startupClip(cSoundStartupWAV);
AudioClip shutdownClip(cSoundShutdownWAV);
AudioClip errorClip(cSoundErrorWAV);

// Servo index -> Dynamixel ID and joint limits. Three consecutive entries per leg
// (coxa, femur, tibia), in the same order as the index triples passed to myLegs.
struct ServoConfig { uint8_t id; int32_t minPos, maxPos; };
static constexpr ServoConfig cServoConfig[] = {
  { 41, cMinCoxa, cMaxCoxa }, { 42, cMinFemur, cMaxFemur }, { 43, cMinTibia, cMaxTibia }, // LF
  { 51, cMinCoxa, cMaxCoxa }, { 52, cMinFemur, cMaxFemur }, { 53, cMinTibia, cMaxTibia }, // LM
  { 61, cMinCoxa, cMaxCoxa }, { 62, cMinFemur, cMaxFemur }, { 63, cMinTibia, cMaxTibia }, // LR
  { 11, cMinCoxa, cMaxCoxa }, { 12, cMinFemur, cMaxFemur }, { 13, cMinTibia, cMaxTibia }, // RF
  { 21, cMinCoxa, cMaxCoxa }, { 22, cMinFemur, cMaxFemur }, { 23, cMinTibia, cMaxTibia }, // RM
  { 31, cMinCoxa, cMaxCoxa }, { 32, cMinFemur, cMaxFemur }, { 33, cMinTibia, cMaxTibia }, // RR
};
static_assert(std::size(cServoConfig) == static_cast<std::size_t>(ServoBus::cNumServos),
              "cServoConfig must have one entry per servo");

// ----------------------------------------------------------------------------------------
Hexapod::Hexapod(Receiver& receiver)
: myServoBus(cDXLDirPin, cDXL_RX, cDXL_TX),
  myLegs{ { LegId::LF, myServoBus, 0, 1, 2    },
          { LegId::LM, myServoBus, 3, 4, 5    },
          { LegId::LR, myServoBus, 6, 7, 8    },
          { LegId::RF, myServoBus, 9, 10, 11  },
          { LegId::RM, myServoBus, 12, 13, 14 },
          { LegId::RR, myServoBus, 15, 16, 17 } },
  myGaitEngine(*this),
  myParkGait(*this),
  myStandUpGait(*this),
  myPosingGait(*this),
  myLevelGait(*this),
  myWalkingGaits{ GenericWalkingGait(*this, "Tripod",   cTripodGaitConfig),
                  GenericWalkingGait(*this, "Tetrapod", cTetrapodGaitConfig),
                  GenericWalkingGait(*this, "Ripple",   cRippleGaitConfig) },
  myReceiver(receiver), 
  mySound(cI2S_LRC, cI2S_BCLK, cI2S_DOUT),
  myIMU(Wire),
  myIsTaskRunning(false),
  myIsPeripheralsTaskRunning(false)
{
  for (int i = 0; i < ServoBus::cNumServos; i++)
    myServoBus.configureServo(i, cServoConfig[i].id, cServoConfig[i].minPos, cServoConfig[i].maxPos);

  // cWaveGaitConfig and cCrawlGaitConfig are defined in Gaits.h but not enabled here.
  // Adding either to the initialiser above also requires raising cNumWalkingGaits.

  myReceiver.setName("Hexapod");
  // Slot 1 is SWITCH 1, which opens the transmitter's menu - it shows "Menu" there regardless of
  // what is sent, so nothing is claimed for it here.
  myReceiver.setSwitchLabels("", "Adjust Gait", "Posing", "Balance");
  myReceiver.setButtonLabels("", "");
}

// ----------------------------------------------------------------------------------------
Hexapod::~Hexapod()
{
  // Stop both tasks before deleting anything they touch. Clearing the flags parks the task loops
  // in their idle branch; the delay covers one full period of the slower of the two
  // (peripheralsTask, cPeripheralsUpdateRate) so neither is mid-iteration when vTaskDelete()
  // kills it - vTaskDelete() runs no cleanup in the target task.
  myIsTaskRunning            = false;
  myIsPeripheralsTaskRunning = false;
  vTaskDelay(pdMS_TO_TICKS(50));

  if (myTaskHandle != nullptr)
  {
    // schedulerTask subscribed itself to the task watchdog. Unsubscribe before deleting it, or
    // the TWDT keeps waiting to be fed by a task that no longer exists and reboots the robot.
    esp_task_wdt_delete(myTaskHandle);
    vTaskDelete(myTaskHandle);
    myTaskHandle = nullptr;
  }

  if (myPeripheralsTaskHandle != nullptr)
  {
    vTaskDelete(myPeripheralsTaskHandle);
    myPeripheralsTaskHandle = nullptr;
  }
}

// ----------------------------------------------------------------------------------------
Hexapod::Diagnostics Hexapod::fetchDiagnostics()
{
  Diagnostics result;

  // Every leg is fetched, not just until a worst is found - fetchDiagnostics() is what clears
  // each leg's counters.
  for (std::size_t i = 0; i < cNumLegs; i++)
  {
    const Leg::Diagnostics d = myLegs[i].fetchDiagnostics();

    result.unreachableTargets += d.unreachableTargets;

    if (fabsf(d.worstOvershoot) > fabsf(result.worstOvershoot))
    {
      result.worstOvershoot = d.worstOvershoot;
      result.worstLeg       = static_cast<int>(i);
    }
  }

  const ServoBus::Diagnostics servo = myServoBus.fetchDiagnostics();

  result.clampedGoals       = servo.clampedGoals;
  result.worstGoalOvershoot = servo.worstGoalOvershoot;
  result.worstGoalServo     = servo.worstGoalServo;
  result.syncWriteFails     = servo.syncWriteFails;

  return result;
}

// ----------------------------------------------------------------------------------------
bool Hexapod::begin(bool announceFault)
{
  // Seed the PRNG from the hardware RNG. Without this, Arduino's random() starts from the
  // same state every boot, so the eye blink/idle timing replays an identical sequence.
  randomSeed(esp_random());

  bool result = myServoBus.begin(); // Only successful servo initialization determines the return value of begin().
  myServoBus.setTorqueOff(); 
  myServoBus.syncReadPresentPosition();

  initializeSwitch();

  myLeds.begin<cLedDataPin>();
  myLeds.setEffect(IndicatorLeds::Effect::eNone);

  // Not folded into 'result', for the same reason as the sound engine below: only servo
  // initialisation decides whether begin() succeeds.
  if (myStatusDisplay.begin(30) == false) // 30 fps, i.e. a 33 ms frame interval
    Serial << "ERROR: " << __PRETTY_FUNCTION__ << " -> status display init failed" << endl;

  myStatusDisplay.setAutoBlinker(true, 3, 2); // blink every 3 s ± up to 2 s variation
  myStatusDisplay.setIdleMode(true, 2, 2);    // reposition every 2 s ± up to 2 s variation

  // Deliberately not folded into 'result': only servo initialisation determines whether begin()
  // succeeds. A silent robot is not a reason to refuse to walk.
  if (mySound.begin() == false)
    Serial << "ERROR: " << __PRETTY_FUNCTION__ << " -> sound engine init failed" << endl;  
  mySound.setVolume(100);
  mySound.setVolume(startupClip,  50);
  mySound.setVolume(shutdownClip, 50);

  myIMU.begin();       // shares the hardware I2C bus with the display; different address, no conflict
  myIMU.calcOffsets(); // hexapod must sit flat on the floor when starting up

  changeState(HexapodState::eOff);

  // A failed servo bus is not harmless: the state machine still reaches eReady and can be
  // commanded to stand. changeState() above hid the display, so the fault has to switch it
  // back on, which showError() does itself.
  if (result == false)
    myStatusDisplay.showError("SERVO INIT FAILED", "check servo bus");

  // Sound the error tone for a failed init, and for any fault the caller detected before the
  // robot existed: untethered there is no serial console, so this is the only way either
  // becomes noticeable.
  if (result == false || announceFault == true)
  {
    mySound.play(errorClip);
    // Wait until sound has stopped playing
    while (mySound.isPlaying(errorClip))
      delay (10);
    delay(500);    
  }

  myIsTaskRunning = true;
  if (xTaskCreatePinnedToCore(schedulerTask, "schedulerTask", 8192, this,
                              8,             // Priority of the task
                              &myTaskHandle, // Task handle
                              1)             // Core 1
      != pdPASS)
  {
    Serial << "ERROR: " << __PRETTY_FUNCTION__ << " -> failed to create schedulerTask" << endl;
    myTaskHandle    = nullptr;
    myIsTaskRunning = false;
    result          = false;
  }

  // Display + IMU: same core, but a strictly lower priority than schedulerTask so a
  // slow I2C transfer (e.g. sendBuffer()) can never delay a servo update - it can only
  // ever run in schedulerTask's idle time.
  myIsPeripheralsTaskRunning = true;
  if (xTaskCreatePinnedToCore(peripheralsTask, "peripheralsTask", 4096, this,
                              2,                        // Lower priority than schedulerTask (8)
                              &myPeripheralsTaskHandle, // Task handle
                              1)                        // Core 1
      != pdPASS)
  {
    Serial << "ERROR: " << __PRETTY_FUNCTION__ << " -> failed to create peripheralsTask" << endl;
    myPeripheralsTaskHandle    = nullptr;
    myIsPeripheralsTaskRunning = false;
    result                     = false;
  }

  return result;
}

// ----------------------------------------------------------------------------------------
Leg& Hexapod::leg(std::size_t index)
{
  if (index >= cNumLegs)
  {
    Serial << __PRETTY_FUNCTION__ << " -> leg index out of range: " << index << endl;
    return myLegs[0];
  }

  return myLegs[index];
}

// ----------------------------------------------------------------------------------------
void Hexapod::step(float dt_ms)
{
  // Accumulates until reset by a state change or by new operator input.
  // Known limitation, accepted by design: float has a 24-bit mantissa, so once this
  // passes 2^24 ms (~4.7 h).  That needs a single uninterrupted state lasting longer
  // than the ~15 min battery allows, so it is not reachable on this hardware.
  myTimePassedMS += dt_ms;

  // Check new commands from remote
  if (myReceiver.hasNewControlData())
  {
    myControlData = myReceiver.getControlData();
  }

  switch (myState)
  {
    case HexapodState::eOff:
      stepOff();
      break;

    case HexapodState::eInitializing:
      stepInitializing();
      break;

    case HexapodState::eReady:
      stepReady();
      break;

    case HexapodState::eStanding:
      stepStanding();
      break;

    case HexapodState::eStandingLeveled:
      stepStandingLeveled();
      break;

    case HexapodState::ePosing:
      stepPosing();
      break;

    case HexapodState::eWalking:
      stepWalking();
      break;

    default:
      Serial << __PRETTY_FUNCTION__ << " -> invalid case label: " << static_cast<int>(myState) << endl;

      myGaitEngine.requestGait(&myParkGait, cParkDurationMs);
      changeState(HexapodState::eReady);
    break;
  }

  if (myGaitEngine.currentGait() != nullptr)
  {
    myGaitEngine.step(dt_ms, myControlData);
  }

  // Telemetry is pushed only when a value changes: setMessage() takes Receiver's mutex, which
  // the link task also holds, and the link transmits twice a second.
  const bool blank = (myState == HexapodState::eOff ||
                      myState == HexapodState::eInitializing ||
                      myState == HexapodState::eReady);

  if (blank == true)
  {
    if (myLastBlank == false)
    {
      for (size_t i=0; i<cNumMessages; i++)
        myReceiver.setMessage(i, "", "");
    }
  }
  else
  {
    // Re-send everything after a blank period, since the slots were cleared.
    bool changed = (myLastBlank == true);

    const GaitEngine::GaitParams& params = myGaitEngine.params();

    // Reported in whole millimetres. Rounding before the comparison, rather than only in the
    // format string, means a value drifting within one millimetre does not count as a change -
    // the tuning axes and the clearance trim move these continuously, and every change re-sends
    // all four slots through a mutex the link task also holds.
    const float stepHeight      = roundf(params.stepHeight);
    const float stepLength      = roundf(params.stepLength);
    const float groundClearance = roundf(params.groundClearance);

    if (myLastStepHeight != stepHeight)
    {
      myLastStepHeight = stepHeight;
      snprintf(myStrStepHeight, sizeof(myStrStepHeight), "%.0f", stepHeight);
      changed = true;
    }

    if (myLastStepLength != stepLength)
    {
      myLastStepLength = stepLength;
      snprintf(myStrStepLength, sizeof(myStrStepLength), "%.0f", stepLength);
      changed = true;
    }

    if (myLastGroundClearance != groundClearance)
    {
      myLastGroundClearance = groundClearance;
      snprintf(myStrGroundClearance, sizeof(myStrGroundClearance), "%.0f", groundClearance);
      changed = true;
    }

    if (myLastGait != myGaitEngine.currentGait())
    {
      myLastGait = myGaitEngine.currentGait();
      changed    = true;
    }

    if (changed == true)
    {
      myReceiver.setMessage(0, "Clearance",   myStrGroundClearance);
      myReceiver.setMessage(1, "Step Length", myStrStepLength);
      myReceiver.setMessage(2, "Step Height", myStrStepHeight);
      myReceiver.setMessage(3, "Gait Type",   myLastGait == nullptr ? "" : myLastGait->name().c_str());
    }
  }

  myLastBlank = blank;
}

// ----------------------------------------------------------------------------------------
void Hexapod::stepOff()
{
  setButtonLabels("", "");

  // Wait until on/off switch is set to on
  if (isSwitchOn() == true)
  {
    mySound.play(startupClip); 
    myGaitEngine.requestGait(&myParkGait, cParkDurationMs);
    changeState(HexapodState::eInitializing);
  }
}

// ----------------------------------------------------------------------------------------
void Hexapod::stepInitializing()
{
  setButtonLabels("", "");

  // Wait until gait may be changed
  if (myGaitEngine.canChangeGait() == false)
    return;

  // Back to off state?
  else if (isSwitchOn() == false)
  {
    myServoBus.setTorqueOff();
    mySound.play(shutdownClip);
    changeState(HexapodState::eOff);
  }

  // Wait until a connection to the transmitter is established
  else if (myReceiver.isPaired() == true)
  {
    // First, return to parking position if torque isn't (fully) on. 
    // If even one servo silently failed to torque on, we want to re-park rather than 
    // assume the robot's current pose is fully known.
    if (myServoBus.isAllTorqueOn() == false)
      myGaitEngine.requestGait(&myParkGait, cParkDurationMs);

    changeState(HexapodState::eReady);
  }

  // Idle after 60 sec (turn servo torque off)
  else if (myTimePassedMS > cWaitTimeUntilTorqueOff)
  {
    myTimePassedMS = 0.0f;
    myServoBus.setTorqueOff();
  }
}

// ----------------------------------------------------------------------------------------
void Hexapod::stepReady()
{
  setButtonLabels(myGaitEngine.canChangeGait() ? "Standup" : "", ""); 

  // Lost connection to transmitter or power switch is off?
  if (isLinkHealthy() == false || isSwitchOn() == false)
  {
    handleConnectionLoss(false); // nothing to interrupt: no gait is active in eReady
  }

  // Pressed left button to standup?
  else if (myControlData.joyL == true && myGaitEngine.canChangeGait())
  {
    myControlData.joyL = false; // process button press just once
    myGaitEngine.requestGait(&myStandUpGait, cStandUpFromParkedMs);
    changeState(HexapodState::eStanding);
  }

  // Idle after 'cWaitTimeUntilTorqueOff' seconds, i.e. turn servo torque off
  else if (myTimePassedMS > cWaitTimeUntilTorqueOff)
  {
    myTimePassedMS = 0.0f;
    myServoBus.setTorqueOff();
  }
}

// ----------------------------------------------------------------------------------------
void Hexapod::stepStanding()
{
  // Wait until gait may be changed. Button presses arriving during the transition are
  // deliberately dropped rather than queued: a park gait takes ~3 s, and acting on a press that
  // long after the fact is not what an operator expects. The blank labels below are the
  // feedback - the transmitter shows both buttons as unavailable while this is true.
  if (myGaitEngine.canChangeGait() == false)
  {
    setButtonLabels("", "");
    return;
  }

  setButtonLabels("Park", "");

  // Lost connection to transmitter or power switch is off?
  if (isLinkHealthy() == false || isSwitchOn() == false)
  {
    handleConnectionLoss(true);
  }

  // Start posing?
  else if (myControlData.switch3 == true)
  {
    myGaitEngine.requestGait(&myPosingGait);
    changeState(HexapodState::ePosing);
  }
  
  // Start balancing (auto-level via onboard IMU)?
  else if (myControlData.switch4 == true)
  {
    myGaitEngine.requestGait(&myLevelGait);
    changeState(HexapodState::eStandingLeveled);
  }

  // Back to park?
  else if (myControlData.joyL == true)
  {
    myControlData.joyL = false; // process button press just once
    myGaitEngine.requestGait(&myParkGait, cParkDurationMs);
    changeState(HexapodState::eReady);
  }

  // Start walking?
  else if (myControlData.LX != 0 || myControlData.LY != 0 || myControlData.LZ != 0 ||
           myControlData.RX != 0 || myControlData.RY != 0 || myControlData.RZ != 0 ||
           myControlData.joyR == true || myControlData.switch2 == true)
  {
    myGaitEngine.requestGait(&myWalkingGaits[myCurrentWalkingGait % cNumWalkingGaits]);
    changeState(HexapodState::eWalking);
  }        
  
  // No user input?
  else if (myTimePassedMS > 2.0f * cIdleTimeout)
  {
    myGaitEngine.requestGait(&myParkGait, cParkDurationMs);
    changeState(HexapodState::eReady);
  }  
}

// ----------------------------------------------------------------------------------------
void Hexapod::stepStandingLeveled()
{
  setButtonLabels("Park", "");

  // Wait until gait may be changed
  if (myGaitEngine.canChangeGait() == false)
    return;

  // Lost connection to transmitter or power switch is off?
  if (isLinkHealthy() == false || isSwitchOn() == false)
  {
    handleConnectionLoss(true);
  }

  // Back to park?
  else if (myControlData.joyL == true)
  {
    myControlData.joyL = false; // process button press just once
    myGaitEngine.requestGait(&myParkGait, cParkDurationMs);
    changeState(HexapodState::eReady);
  }

  // Start posing?
  else if (myControlData.switch3 == true)
  {
    myGaitEngine.requestGait(&myPosingGait);
    changeState(HexapodState::ePosing);
  }

  // Back to plain standing?
  else if (myControlData.switch4 == false)
  {
    myGaitEngine.requestGait(&myStandUpGait, cStandUpDeployedMs);
    changeState(HexapodState::eStanding);
  }
}

// ----------------------------------------------------------------------------------------
void Hexapod::stepPosing()
{
  setButtonLabels("Park", "");

  // Wait until gait may be changed
  if (myGaitEngine.canChangeGait() == false)
    return;

  // Lost connection to transmitter or power switch is off?
  if (isLinkHealthy() == false || isSwitchOn() == false)
  {
    handleConnectionLoss(true);
  }

  // Back to park?
  else if (myControlData.joyL == true)
  {
    myControlData.joyL = false; // process button press just once
    myGaitEngine.requestGait(&myParkGait, cParkDurationMs);
    changeState(HexapodState::eReady);
  }

  // Back to standing?
  else if (myControlData.switch3 == false)
  {
    myGaitEngine.requestGait(&myStandUpGait, cStandUpDeployedMs);
    changeState(HexapodState::eStanding);
  }
}

// ----------------------------------------------------------------------------------------
void Hexapod::stepWalking()
{
  // Walking forward?
  if (myControlData.LY > 0)
  {
    // LED strip orientation is rotated by 180deg
    myLeds.setEffect(IndicatorLeds::Effect::eSweepBackward);
  }
  // Walking backward?
  else if (myControlData.LY < 0)
  {
    // LED strip orientation is rotated by 180deg
    myLeds.setEffect(IndicatorLeds::Effect::eSweepForward);
  }
  // Walking sideways or turning?
  else if (myControlData.LX != 0 || (myControlData.RX != 0 && myControlData.switch2 == false))
  {
    myLeds.setEffect(IndicatorLeds::Effect::eSweepMeetInMiddle);
  }
  else
  {
    // No joystick input
    myLeds.setEffect(IndicatorLeds::Effect::eNone);
  }

  // Map joystick input to eye position
  // Note: if switch2 == true then "adjust gait" is active
  if (abs(myControlData.LX) >= abs(myControlData.RX) || myControlData.switch2 == true)
    myStatusDisplay.setLookDirection(mapf(myControlData.LX * -1, cJoystickMin, cJoystickMax, -1.0f, 1.0f),
                                mapf(myControlData.LY,      cJoystickMin, cJoystickMax, -1.0f, 1.0f));
  else
    myStatusDisplay.setLookDirection(mapf(myControlData.RX * -1, cJoystickMin, cJoystickMax, -1.0f, 1.0f),
                                mapf(myControlData.LY,      cJoystickMin, cJoystickMax, -1.0f, 1.0f));


  setButtonLabels("Park", "Change Gait");

  // Reset the idle timer while the operator is actually commanding something. Whether the
  // input *changed* is a different question: holding a stick steady produces no change at all,
  // so the robot used to stop walking after cIdleTimeout with the stick still pushed forward.
  //
  // A plain != 0 test is right here. Joystick3Axis on the transmitter applies its own deadzone
  // and returns exactly 0 at centre, so there is no noise to filter at this end - and a second
  // threshold would be actively wrong, since a small deliberate deflection maps to a small
  // non-zero value and must still count as activity.
  if (myControlData.LX != 0 || myControlData.LY != 0 || myControlData.LZ != 0 ||
      myControlData.RX != 0 || myControlData.RY != 0 || myControlData.RZ != 0)
  {
    myTimePassedMS = 0.0f; // reset timeout timer
  }

  // No canChangeGait() guard here, unlike the other state handlers. 
  // The state changes below therefore take effect immediately while the gait switch is deferred:
  // requestGait() stores the request and GaitEngine applies it at the end of the cycle. The
  // destination handler's entry gate covers the gap.

  // Lost connection to transmitter or power switch is off?
  if (isLinkHealthy() == false || isSwitchOn() == false)
  {
    handleConnectionLoss(true);
  }

  // Back to park?
  else if (myControlData.joyL == true)
  {
    myControlData.joyL = false; // process button press just once
    myGaitEngine.requestGait(&myParkGait, cParkDurationMs);
    changeState(HexapodState::eReady);
  }

  // Change walking gait?
  else if (myControlData.joyR == true)
  {
    myControlData.joyR = false; // process button press just once
    myCurrentWalkingGait++;
    myTimePassedMS = 0.0; // reset timeout timer    
    myGaitEngine.requestGait(&myWalkingGaits[myCurrentWalkingGait % cNumWalkingGaits]); 
  }

  // Start posing?
  else if (myControlData.switch3 == true)
  {
    myGaitEngine.requestGait(&myPosingGait);
    changeState(HexapodState::ePosing);
  }

  // Start posing?
  else if (myControlData.switch4 == true)
  {
    myGaitEngine.requestGait(&myLevelGait);
    changeState(HexapodState::eStandingLeveled);
  }

  // No user input?
  else if (myTimePassedMS > cIdleTimeout)
  {
    myGaitEngine.requestGait(&myStandUpGait, cStandUpDeployedMs);
    changeState(HexapodState::eStanding);
  }
}

// ----------------------------------------------------------------------------------------
void Hexapod::changeState(HexapodState newState)
{
  Serial << "State change: " << toString(myState)  << ":" << int(myState)  << " -> " 
                             << toString(newState) << ":" << int(newState) 
                             << ((myState == newState) ? " (no change)" : "")
                             << endl;
  changeEyeConfig(newState);

  if (newState == HexapodState::eInitializing)
  {
    myGaitEngine.resetParams();
  }

  myState = newState;
  myTimePassedMS = 0.0f; // Reset timeout timer
}

// ----------------------------------------------------------------------------------------
void Hexapod::setButtonLabels(const char* btn1, const char* btn2)
{
  // Compared here rather than inside Receiver: this runs on the control task at 200 Hz, and
  // Receiver::setButtonLabels() takes a mutex the link task also holds. The labels change only
  // on state transitions, so the comparison short-circuits the call almost every tick.
  if (strcmp(myLastBtn1, btn1) == 0 && strcmp(myLastBtn2, btn2) == 0)
    return;

  snprintf(myLastBtn1, sizeof(myLastBtn1), "%s", btn1);
  snprintf(myLastBtn2, sizeof(myLastBtn2), "%s", btn2);

  myReceiver.setButtonLabels(btn1, btn2);
}

// ----------------------------------------------------------------------------------------
bool Hexapod::isLinkHealthy()
{
  // Two independent conditions on purpose. isPaired() is the transport layer's opinion; the
  // staleness test is the control loop's own and does not depend on that layer being correct.
  // It also catches a stalled WiFi task or a stuck myNewData flag - neither of which would
  // ever clear the peer.
  return myReceiver.isPaired() == true &&
         myReceiver.timeSinceLastControlData() <= cControlDataTimeoutMs;
}

// ----------------------------------------------------------------------------------------
void Hexapod::handleConnectionLoss(bool requestParkGait)
{
  myControlData.reset();

  if (requestParkGait == true)
    myGaitEngine.requestGait(&myParkGait, cParkDurationMs);

  changeState(HexapodState::eInitializing);
}

// ----------------------------------------------------------------------------------------
const char* Hexapod::toString(HexapodState state)
{
  switch (state)
  {
    case HexapodState::eOff:             return "Off";
    case HexapodState::eInitializing:    return "Initializing";
    case HexapodState::eReady:           return "Ready";
    case HexapodState::eStanding:        return "Standing";
    case HexapodState::eStandingLeveled: return "StandingLeveled";
    case HexapodState::ePosing:          return "Posing";
    case HexapodState::eWalking:         return "Walking";
  }

  return "Unknown";  
}

// ----------------------------------------------------------------------------------------
void Hexapod::initializeSwitch()
{
  pinMode(cOnOffSwitchPin, INPUT_PULLUP);
}

// ----------------------------------------------------------------------------------------
bool Hexapod::isSwitchOn() const
{
  return !digitalRead(cOnOffSwitchPin);  
}

// ----------------------------------------------------------------------------------------
// Called from schedulerTask, not from peripheralsTask which owns the display. The setters below
// therefore race with update() - see the note in peripheralsTask(). A momentarily torn frame is
// the accepted outcome.
void Hexapod::changeEyeConfig(HexapodState newState)
{
  switch (newState)
  {
    case HexapodState::eOff:
      myLeds.setEffect(IndicatorLeds::Effect::eNone);
      myStatusDisplay.setVisible(false);
      break; 

    case HexapodState::eInitializing:
      myLeds.setEffect(IndicatorLeds::Effect::ePulseRed);
      myStatusDisplay.setMood(StatusDisplay::Mood::eTired);
      myStatusDisplay.setSpaceBetween(10);
      myStatusDisplay.setWidth(36, 36);
      myStatusDisplay.setHeight(36, 36);
      myStatusDisplay.setCuriosity(false);
      myStatusDisplay.setIdleMode(false);
      myStatusDisplay.setVisible(true);
      break;

    case HexapodState::eReady:
      myLeds.setEffect(IndicatorLeds::Effect::ePulseRed);
      myStatusDisplay.setMood(StatusDisplay::Mood::eDefault);
      myStatusDisplay.setSpaceBetween(10);
      myStatusDisplay.setWidth(36, 36);
      myStatusDisplay.setHeight(36, 36);
      myStatusDisplay.setCuriosity(false);
      myStatusDisplay.setIdleMode(true);
      myStatusDisplay.setVisible(true);
      break;

    case HexapodState::eStanding:
      myLeds.setEffect(IndicatorLeds::Effect::ePulseRed);
      myStatusDisplay.setMood(StatusDisplay::Mood::eDefault);
      myStatusDisplay.setSpaceBetween(10);
      myStatusDisplay.setWidth(36, 36);
      myStatusDisplay.setHeight(45, 45);
      myStatusDisplay.setCuriosity(true);
      myStatusDisplay.setIdleMode(true);
      myStatusDisplay.setVisible(true);
      break;

    case HexapodState::eStandingLeveled:
    case HexapodState::ePosing:
      myLeds.setEffect(IndicatorLeds::Effect::ePulseOrange);
      myStatusDisplay.setMood(StatusDisplay::Mood::eHappy);
      myStatusDisplay.setSpaceBetween(10);
      myStatusDisplay.setWidth(45, 45);
      myStatusDisplay.setHeight(45, 45);
      myStatusDisplay.setCuriosity(true);
      myStatusDisplay.setIdleMode(true);
      myStatusDisplay.setVisible(true);
      break;

    case HexapodState::eWalking:
      // led effect is set in stepWalking()
      myStatusDisplay.setMood(StatusDisplay::Mood::eFocused);
      myStatusDisplay.setSpaceBetween(15);
      myStatusDisplay.setWidth(45, 45);
      myStatusDisplay.setHeight(40, 40);
      myStatusDisplay.setCuriosity(false);
      myStatusDisplay.setIdleMode(false);
      myStatusDisplay.setVisible(true);
      break;
  }
}

// ----------------------------------------------------------------------------------------
void Hexapod::schedulerTask(void* pvParameters)
{
  // This task runs on core 1

  Hexapod* h = (Hexapod*) pvParameters;

  constexpr unsigned long dt_ms = 1000 / cTargetUpdateRate;

  // step() is given the *measured* period, not the nominal one. Everything downstream
  // integrates against it - gait phase, the low-pass filters, myTimePassedMS - so a constant
  // means walking speed and every timeout silently become a function of CPU load whenever the
  // loop runs late. Clamped so a one-off stall produces a bounded step instead of a jump.
  constexpr float cMaxDtMS = 20.0f;

  // Offset by one nominal period so the very first tick sees dt_ms rather than 0.
  int64_t lastCycleStart = esp_timer_get_time() - static_cast<int64_t>(dt_ms) * 1000;

  // To compute the time of one update and the delay if we are too fast
  int64_t t0, t1;
  int delayMS;

  // To compute the actual number of servo updates per second to verify if we are fast enough
  int updateCounter = 0; 
  int64_t startTime = 0;

  // Ticks in the current second that missed the dt_ms deadline. A non-zero value means
  // step() is too slow and the loop is no longer running at cTargetUpdateRate.
  int overrunCounter = 0;

  // Register with the task watchdog. Arduino-ESP32's default TWDT already watches the idle
  // tasks, which catches this task *spinning* at priority 8; registering explicitly also catches
  // it *blocking* - stuck on a bus read or a mutex - where the idle task still runs and the idle
  // check stays quiet. Either way a hung control loop reboots rather than leaving the robot
  // torqued up with nothing driving it.
  if (esp_task_wdt_add(nullptr) != ESP_OK)
    Serial << __PRETTY_FUNCTION__ << " -> esp_task_wdt_add failed - control loop is unguarded" << endl;

  while (true)
  {
    // Unconditionally at the top, so it covers the stopped branch below too.
    esp_task_wdt_reset();

    if (h->myIsTaskRunning == true)
    {
      t0 = esp_timer_get_time();

      // Print update rate each second, DEBUG only
      if (t0 - startTime > 1000000) 
      {
        // Serial << "#updates/sec:" << updateCounter << ", #overruns:" << overrunCounter << endl;
        startTime = t0;
        updateCounter = 0;
        overrunCounter = 0;
      }
      ++updateCounter;

      float dt = static_cast<float>(t0 - lastCycleStart) / 1000.0f;
      if (dt > cMaxDtMS)
        dt = cMaxDtMS;

      lastCycleStart = t0;

      h->step(dt);

      // Measure whether we reach the target update rate. If input processing,
      // computation, and servo control are faster than the maximum update rate,
      // add a delay.  
      t1 = esp_timer_get_time();  // us
      delayMS = (dt_ms - (t1 - t0) / 1000);
      if (delayMS > 0)
      {
        vTaskDelay(pdMS_TO_TICKS(delayMS));
      }
      else
      {
        // Behind schedule. Give up at least one full tick.
        ++overrunCounter;
        vTaskDelay(1);
      }
    }
    else
    {
      // Task stopped (e.g. during shutdown, after myIsTaskRunning is cleared but
      // before vTaskDelete). Without a delay this spins at priority 8 and hard-locks
      // core 1.
      vTaskDelay(pdMS_TO_TICKS(10));

      // Do not let the stopped interval count as elapsed control time when the task resumes.
      lastCycleStart = esp_timer_get_time() - static_cast<int64_t>(dt_ms) * 1000;
    }
  }
}

// ----------------------------------------------------------------------------------------
void Hexapod::peripheralsTask(void* pvParameters)
{
  // Runs on core 1, at a strictly lower priority than schedulerTask. Owns the I2C bus: the IMU
  // is touched only from here.
  //
  // StatusDisplay is a deliberate exception. Its header requires every call, including update(), to
  // come from a single task - but update() runs here while schedulerTask sets mood, size and look
  // direction via changeEyeConfig() and stepWalking(). Those setters write several members that
  // drawEyes() then reads, so a frame can render a partially applied change: for example the new
  // mood at the old eye size. That is the entire consequence, it lasts one frame, and serialising
  // it through a queue or atomics costs more than the defect is worth. Do not extend this to
  // anything whose corruption is not purely cosmetic.

  Hexapod* h = (Hexapod*) pvParameters;

  constexpr unsigned long dt_ms = 1000 / cPeripheralsUpdateRate;

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true)
  {
    if (h->myIsPeripheralsTaskRunning == true)
    {
      Wire.setClock(cIMUBusClockHz); // StatusDisplay leaves the bus at its own higher clock
      h->myIMU.update();
      h->myStatusDisplay.update(); // no-op unless its own configured frame interval has elapsed
      h->myLeds.update();     // no-op unless its own frame interval has elapsed
    }

    // Fixed-period wake-up: unlike schedulerTask's measure-then-delay approach, this
    // doesn't need to compensate for work taking longer than dt_ms - update() calls
    // above are cheap when nothing is due, and being a few ms late here has no
    // consequence the way it would for servo timing.
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(dt_ms));
  }
}
