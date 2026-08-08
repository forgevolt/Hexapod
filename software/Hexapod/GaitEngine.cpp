#include "Arduino.h"
#include <cmath>
#include "GaitEngine.h"
#include "Gaits.h"
#include "Hexapod.h"
#include "MathUtilities.h"

#include <Streaming.h>

// ---- Tuning ------------------------------------------------------------------------------

// Right-stick deflection below this is ignored while SWITCH 2 selects the tuning axes. The
// transmitter's Joystick class applies its own deadzone; this one matters because the tuning
// values accumulate rather than track, so a small resting offset would drift them.
constexpr int cTuningDeadzone = 50;

// At full stick deflection a tuning axis crosses its whole range in this many seconds, so all
// three feel alike however wide their range is. Derived from the limits rather than written
// out, so widening a range keeps the sweep time instead of silently changing it.
constexpr float cTuningSweepSeconds = 2.0f;

constexpr float cStepLengthRatePerS = (GaitEngine::cMaxStepLength - GaitEngine::cMinStepLength)
                                      / cTuningSweepSeconds; // 65.0 mm/s

// Min is the numerically larger of the two here - see the note on the clearance limits.
constexpr float cGroundClearanceRatePerS = (GaitEngine::cMinGroundClearance - GaitEngine::cMaxGroundClearance)
                                           / cTuningSweepSeconds; // 45.0 mm/s

constexpr float cStepHeightRatePerS = (GaitEngine::cMaxStepHeight - GaitEngine::cMinStepHeight)
                                      / cTuningSweepSeconds; // 15.0 mm/s

static_assert(cStepLengthRatePerS > 0.0f && cGroundClearanceRatePerS > 0.0f &&
              cStepHeightRatePerS > 0.0f,
              "tuning rates must be positive - check the min/max order for each parameter");


// Exponential stick response for the tuning axes, in the shape RC transmitters use: a cubic
// blended with a little linear. Gentle deflections stay slow and the axis maximum is reached
// only at the stops, which is what makes millimetre corrections possible.
//
// cTuningExpo is the cubic's share of the blend: 0 is a plain linear map, 1 a pure cubic. The
// linear remainder is what keeps the response alive just past the deadzone.
constexpr float cTuningExpo = 0.7f;

static_assert(cTuningExpo >= 0.0f && cTuningExpo <= 1.0f,
              "cTuningExpo outside [0, 1] makes the response non-monotonic near centre");


// Curvature of the stride response to stick deflection. The stride is magDirection raised to
// this power, so 1.0 is a plain linear map, and values below it give more stride for less
// stick - at 0.7 a fifth of travel yields a third of the step length instead of a fifth.
//
// Cadence is not shaped: myDurationMS still follows magnitude linearly, so a small deflection
// means normal-length steps taken slowly rather than short steps.
constexpr float cStrideCurve = 0.7f;

static_assert(cStrideCurve > 0.0f && cStrideCurve <= 1.0f,
              "cStrideCurve outside (0, 1] would shorten the stride or overshoot the maximum");

// Deflection at which the stride reaches the full step length. Past this the stride is
// clamped, so the remaining travel only raises cadence - myDurationMS keeps following the
// stick over the whole range.
constexpr float cStrideFullAt = 0.6f;

static_assert(cStrideFullAt > 0.0f && cStrideFullAt <= 1.0f,
              "cStrideFullAt outside (0, 1] would put full stride out of reach or at rest");


// Time constant for the fall of MotionCmd::demand. Only the fall is filtered; a rise is taken
// immediately, so this delays feet settling on a stop without ever delaying a lift.
constexpr float cDemandReleaseMS = 300.0f;

// How much slower than the gait's fastest cycle to run out a released cycle. 1.0 finishes as
// quickly as the gait allows, which stops abruptly; larger values wind down more gently at the
// cost of taking longer to park.
constexpr float cFinishDurationFactor = 2.0f;

// ----------------------------------------------------------------------------------------
// Signed millimetres per second for the given axis deflection.
static float shapedRate(int axis, float ratePerS)
{
  const float span   = static_cast<float>(cJoystickMax - cTuningDeadzone);
  const float travel = std::clamp((abs(axis) - cTuningDeadzone) / span, 0.0f, 1.0f);

  const float shaped    = cTuningExpo * travel * travel * travel + (1.0f - cTuningExpo) * travel;
  const float magnitude = ratePerS * shaped;

  return (axis < 0) ? -magnitude : magnitude;
}


// ---- GaitEngine ------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
GaitEngine::GaitEngine(Hexapod& hexapod)
: myHexapod(hexapod)
{
  resetParams();
}

// ----------------------------------------------------------------------------------------
void GaitEngine::resetParams()
{
  myParams.groundClearance = cDefaultGroundClearance;
  myParams.stepHeight      = cDefaultStepHeight;
  myParams.stepLength      = cDefaultStepLength;

  myLiveParams    = myParams;
  myClearanceTrim = 0.0f;
}

// ----------------------------------------------------------------------------------------
void GaitEngine::step(float dt_ms, const Receiver::ControlData& input)
{
  // Only logs once per violation (not every call) so a genuine
  // invariant break is still loud without spamming the log at 200Hz forever.
  if (myActiveGait == nullptr) 
  {
    static bool warned = false;
    if (warned == false)
    {
      Serial << __PRETTY_FUNCTION__ << " -> no active gait" << endl;
      warned = true;
    }
    return;
  }

  // ------------------------------------------------------------------------   
  // Posing gaits apply BodyPose transformations (translation + rotation).
  // Control input comes from the transmitter joysticks and its IMU.
  // ------------------------------------------------------------------------ 
  if (myActiveGait->getCategory() == GaitCategory::ePosture)
  {
    float distance, angle;

    distance = mapf(input.LY, cJoystickMin, cJoystickMax, cMinPoseTranslation, cMaxPoseTranslation);
    myCurrentMotion.pose.translation.x = lowPassFilter(myCurrentMotion.pose.translation.x, distance,
                                                       200.0f, dt_ms); // ~1 s to effectively settle (~5×tau) 

    distance = mapf(-input.LX, cJoystickMin, cJoystickMax, cMinPoseTranslation, cMaxPoseTranslation);
    myCurrentMotion.pose.translation.y = lowPassFilter(myCurrentMotion.pose.translation.y, distance,
                                                       200.0f, dt_ms); // ~1 s to effectively settle (~5×tau) 

    if (input.LZ < 0)
      distance = mapf(input.LZ, cJoystickMin, 0, myParams.groundClearance - cMinGroundClearance, 0);
    else
      distance = mapf(input.LZ, 0, cJoystickMax, 0, myParams.groundClearance - cMaxGroundClearance);
    myCurrentMotion.pose.translation.z = lowPassFilter(myCurrentMotion.pose.translation.z, distance,
                                                       200.0f, dt_ms); // ~1 s to effectively settle (~5×tau) 

    // Roll and Pitch IMU based?
    if (input.switch4 == true)
    {
      angle = std::clamp(input.angleX*deg2rad, cMinPoseRotation, cMaxPoseRotation);
      myCurrentMotion.pose.rotation.x = lowPassFilter(myCurrentMotion.pose.rotation.x, angle,
                                                      150.0f, dt_ms); // ~750 ms to effectively settle (~5×tau) 

      angle = std::clamp(-input.angleY*deg2rad, cMinPoseRotation, cMaxPoseRotation);
      myCurrentMotion.pose.rotation.y = lowPassFilter(myCurrentMotion.pose.rotation.y, angle,
                                                      150.0f, dt_ms); // ~750 ms to effectively settle (~5×tau) 
    }
    // No, use RX, RY for Roll and Pitch 
    else
    {
      angle = mapf(input.RX, cJoystickMin, cJoystickMax, cMinPoseRotation, cMaxPoseRotation);
      myCurrentMotion.pose.rotation.x = lowPassFilter(myCurrentMotion.pose.rotation.x, angle,
                                                      150.0f, dt_ms); // ~750 ms to effectively settle (~5×tau)

      angle = mapf(input.RY, cJoystickMin, cJoystickMax, cMinPoseRotation, cMaxPoseRotation);
      myCurrentMotion.pose.rotation.y = lowPassFilter(myCurrentMotion.pose.rotation.y, angle,
                                                      150.0f, dt_ms); // ~750 ms to effectively settle (~5×tau)       
    }

    angle = mapf(-input.RZ, cJoystickMin, cJoystickMax, cMinPoseRotation*1.5f, cMaxPoseRotation*1.5f);
    myCurrentMotion.pose.rotation.z = lowPassFilter(myCurrentMotion.pose.rotation.z, angle,
                                                    100.0f, dt_ms); // ~500 ms to effectively settle (~5×tau)
  }

  // ------------------------------------------------------------------------   
  // Locomotion gaits use translational and rotational motion commands
  // to compute foot trajectories. The normalized gait phase (0.0–1.0)
  // defines each leg’s position within the step cycle.
  // ------------------------------------------------------------------------   
  else if (myActiveGait->getCategory() == GaitCategory::eLocomotion)
  {
    // When SWITCH 2 is true:
    //   RX -> changes step length
    //   RY -> changes ground clearance
    //   RZ -> changes step height
    if (input.switch2 == true)
    {
      // shapedRate() returns zero inside the deadzone, so no guard is needed here.

      // The X-axis of the right joystick sets the step length
      // Valid range: [cMinStepLength, cMaxStepLength]
      myParams.stepLength += shapedRate(input.RX, cStepLengthRatePerS) * (dt_ms * 0.001f);
      myParams.stepLength  = std::clamp(myParams.stepLength, cMinStepLength, cMaxStepLength);

      // The Y-axis sets ground clearance. Negated so that RY positive raises the body, the
      // same direction the axis moves it without SWITCH 2 held.
      // Valid range: [cMaxGroundClearance, cMinGroundClearance]
      myParams.groundClearance += shapedRate(-input.RY, cGroundClearanceRatePerS) * (dt_ms * 0.001f);
      myParams.groundClearance  = std::clamp(myParams.groundClearance,
                                             cMaxGroundClearance, cMinGroundClearance);

      // The Z-axis of the right joystick sets the step height
      // Valid range: [cMinStepHeight, cMaxStepHeight]
      myParams.stepHeight += shapedRate(input.RZ, cStepHeightRatePerS) * (dt_ms * 0.001f);
      myParams.stepHeight  = std::clamp(myParams.stepHeight, cMinStepHeight, cMaxStepHeight);
    }

    // ------------------------------------------------------------------------   
    // Duration of a cycle is controlled by the left joystick, i.e. by the magnitude 
    // of the direction vector and rotation. This results in slower or faster movement 
    // of the legs.
    //   Range: [cMinDurationMS, cMaxDurationMS]
    //   Default value: cMaxDurationMS equals no movement    
    // ------------------------------------------------------------------------   
    
    // Smooth joystick values
    myLinearX = lowPassFilter(myLinearX, input.LX, 25.0f, dt_ms);
    myLinearY = lowPassFilter(myLinearY, input.LY, 25.0f, dt_ms);

    // Invert joystick x-value to match the coordinate system (right handed)
    Vector2 direction = circularNormalization(-myLinearX / cJoystickMax, myLinearY / cJoystickMax);
    float magDirection = direction.length();

    // When SWITCH 2 is false: RX (-> 2-joy transmitter) or LZ (1-joy transmitter) rotate hexapod 
    // around its vertical axis (yaw)
    static ESPNowConnection::PeerInfo peerInfo;
    float inputRotation = 0.0f;
    if (myHexapod.receiver().getPeerInfo(peerInfo) == true && peerInfo.device == eTransmitter1Joy)
      inputRotation = input.LZ;
    else
      inputRotation = input.RX;

    myAngularZ = (input.switch2 == false) ? lowPassFilter(myAngularZ, inputRotation, 25.0f, dt_ms) : 0.0f;
    float magRotation = fabs(myAngularZ) / cJoystickMax;
    
    // Compute magnitude with magDirection as the 'Master'
    // As magDirection approaches 1.0, the influence of magRotation is scaled down.
    float magnitude = magDirection + (magRotation * (1.0f - magDirection));

    // Clamp to ensure floating point precision doesn't exceed 1.0
    magnitude = std::clamp(magnitude, 0.0f, 1.0f);

    // The fastest cycle this gait is allowed to run. Guarded rather than trusted: a config with
    // a zero or negative limit would divide by zero, and one above 1.0 would demand a cycle
    // shorter than the engine's own minimum.
    const float limit       = std::clamp(myActiveGait->speedLimit(), 0.0f, 1.0f);
    const float minDuration = (limit > 0.0f)
                            ? std::clamp(cMinDurationMS / limit, cMinDurationMS, cMaxDurationMS)
                            : cMaxDurationMS;

    // The operator's raw axes, not the filtered magnitude: Joystick3Axis returns exactly 0 at
    // centre, while the filters only approach it asymptotically. Only the raw value can tell
    // that the stick has been released.
    const bool isCommanded = (input.LX != 0 || input.LY != 0 ||
                              (input.switch2 == false && input.RX != 0));

    if (isCommanded == true)
    {
      // Target duration based on current input
      float targetDuration = mapf(magnitude, 0.0f, 1.0f, cMaxDurationMS, minDuration);
      myDurationMS = lowPassFilter(myDurationMS, targetDuration, 500.0f, dt_ms);
    }
    else 
    {
      // Released mid-cycle: run the remaining phase out near the fast end rather than
      // stretching toward cMaxDurationMS. The gait has to reach phase 1.0 before it can park,
      // and at the slow end that takes seconds - ripple worst of all. cFinishDurationFactor
      // holds it back from the gait's actual maximum so the wind-down is not a sprint.
      myDurationMS = lowPassFilter(myDurationMS,
                                   std::clamp(minDuration * cFinishDurationFactor,
                                              minDuration, cMaxDurationMS),
                                   500.0f, dt_ms);
    }

    // Switching gait can leave myDurationMS faster than the new gait allows, and the filter
    // would need half a second to pull it back - during which the servos see the old rate.
    myDurationMS = std::clamp(myDurationMS, minDuration, cMaxDurationMS);

    // ------------------------------------------------------------------------
    // Movement input processing: Maps 2D joystick input to a velocity vector 
    // constrained by max step length (stepLength).
    // ------------------------------------------------------------------------   

    // Map magnitude to a target step length: rescaled so full stride is reached at
    // cStrideFullAt, then curved so that gentle deflections still give a usable stride.
    // powf(0, k) is 0, so this still commands no motion at rest.
    const float reach = std::clamp(magDirection / cStrideFullAt, 0.0f, 1.0f);

    float targetStepLen = powf(reach, cStrideCurve) * myParams.stepLength;

    // Normalize the direction vector (left joystick) to magnitude 1.0
    direction = direction.normalize(); 
    
    // We swap Y/X axes here to align joystick vertical input with robot forward motion    
    Vector2 targetLinear(direction.y * targetStepLen, direction.x * targetStepLen);

    // Smooth the motion
    myCurrentMotion.linear.x = lowPassFilter(myCurrentMotion.linear.x, targetLinear.x, 500.0f, dt_ms);
    myCurrentMotion.linear.y = lowPassFilter(myCurrentMotion.linear.y, targetLinear.y, 500.0f, dt_ms);

    // ------------------------------------------------------------------------
    // Rotation Logic: Maps joystick Z-axis to angular rotation (Yaw).
    // cMaxYaw defines the max rotation per cycle.
    // Lerp alpha (0.01f) defines the "softness" of the acceleration.
    // ------------------------------------------------------------------------

    // Target yaw based on joystick position
    float targetYaw = std::clamp(-myAngularZ / cJoystickMax, -1.0f, 1.0f) * cMaxYaw;

    myCurrentMotion.yaw = lowPassFilter(myCurrentMotion.yaw, targetYaw, 500.0f, dt_ms);
  }

  bool isInputZero = (int(myLinearX) == 0 && int(myLinearY) == 0 && int(myAngularZ) == 0);
  bool isLocomotion = (myActiveGait->getCategory() == GaitCategory::eLocomotion);

  // Stay in the neutral/parked position if we are in a walking gait, input has stopped,
  // and we have successfully completed the current step cycle.
  if (isLocomotion == true && isInputZero == true && floatEquals(myGaitPhase, 0.0f)) 
  {
    // Parked at neutral: nothing to advance. The cycle duration goes back to the slow end so
    // the next one starts from rest rather than inheriting the speed this one was wound down
    // to, which would run the first half-second faster than a gentle stick asks for. Safe to
    // assign rather than filter - nothing moves while parked.
    myDurationMS = cMaxDurationMS;
  } 
  else 
  {
    // Advance Phase
    myGaitPhase += (dt_ms / myDurationMS);

    // Wrap & Completion Logic
    if (myGaitPhase >= 1.0f) 
    {
      // Inform the gait the boundary was hit
      myActiveGait->cycleComplete();

      if (isLocomotion && isInputZero) 
      {
        // If the user let go of the stick, SNAP to 0.0 and stay there
        myGaitPhase = 0.0f;
      } 
      else 
      {
        // Otherwise, wrap around
        myGaitPhase = std::fmod(myGaitPhase, 1.0f);
      }
    }
  }

  // RY always means ground clearance. Held with SWITCH 2 it accumulates into myParams above
  // and persists; on its own it only trims, hinged on the tuned value - full deflection
  // reaches cMinGroundClearance one way and cMaxGroundClearance the other, and centring the
  // stick returns the robot to the tuned clearance.
  //
  // The trim is never written back to myParams. Anything that is not a locomotion gait, and
  // holding SWITCH 2, ease it back to zero.
  float targetTrim = 0.0f;

  if (isLocomotion == true && input.switch2 == false)
  {
    if (input.RY < 0)
      targetTrim = mapf(input.RY, cJoystickMin, 0, cMinGroundClearance - myParams.groundClearance, 0);
    else
      targetTrim = mapf(input.RY, 0, cJoystickMax, 0, cMaxGroundClearance - myParams.groundClearance);
  }

  myClearanceTrim = lowPassFilter(myClearanceTrim, targetTrim,
                                  200.0f, dt_ms); // ~1 s to effectively settle (~5×tau)

  // Clamped because SWITCH 2 can move myParams while a trim is still unwinding.
  myLiveParams = myParams;
  myLiveParams.groundClearance = std::clamp(myParams.groundClearance + myClearanceTrim,
                                            cMaxGroundClearance, cMinGroundClearance);

  // How hard the robot is being asked to move, for the gaits to scale foot lift by. Rises the
  // instant the stride does, but falls slowly. A stride reversing from forward to back passes
  // through zero on the way, and lift keyed to the instantaneous value reads that crossing as a
  // standing robot and puts all six feet on the floor for its duration. The release is slow
  // enough to bridge that and short enough that a robot actually coming to rest still settles.
  const float strideDemand = std::max(myCurrentMotion.linear.length() / myLiveParams.stepLength,
                                      fabsf(myCurrentMotion.yaw) / cMaxYaw);

  myCurrentMotion.demand = (strideDemand > myCurrentMotion.demand)
                         ? strideDemand
                         : lowPassFilter(myCurrentMotion.demand, strideDemand,
                                         cDemandReleaseMS, dt_ms);

  myActiveGait->update(myGaitPhase, myCurrentMotion);

  // Switch to next (desired) gait?
  trySwitchGait();
}

// ----------------------------------------------------------------------------------------
void GaitEngine::requestGait(Gait* gait, float durationMS) 
{
  myDesiredGait = gait;
  myDesiredDurationMS = durationMS;

  // durationMS must be a sane positive value: step() divides dt_ms by myDurationMS
  // with no guard of its own, so this is the only thing preventing a divide-by-zero
  // (or a phase that runs backwards, for a negative value) downstream.
  if (durationMS <= 0.0f || floatEquals(durationMS, 0.0f) == true) 
     myDesiredDurationMS = cMaxDurationMS;

  trySwitchGait();
}

// ----------------------------------------------------------------------------------------
bool GaitEngine::isMorph() const
{
  if (myActiveGait == nullptr || myDesiredGait == nullptr)
    return false;

  if (myActiveGait->getCategory() != GaitCategory::eLocomotion ||
      myDesiredGait->getCategory() != GaitCategory::eLocomotion)
    return false;

  // Both ends have to carry a leg pattern for the handover to mean anything.
  return (myActiveGait->activeConfig() != nullptr && myDesiredGait->activeConfig() != nullptr);
}

// ----------------------------------------------------------------------------------------
bool GaitEngine::canChangeGait() const
{
  if (myActiveGait == nullptr)
    return true;

  // A walking-to-walking change blends and needs no cycle boundary. Everything else does.
  if (isMorph() == true)
    return true;

  return myActiveGait->canChange();
}


// ----------------------------------------------------------------------------------------
void GaitEngine::trySwitchGait()
{
  if (myDesiredGait == nullptr)
    return;

  if (canChangeGait() == true)
  {
    // Entering locomotion from a non-locomotion gait starts a new walking session. myCurrentMotion
    // is only updated while a locomotion gait is active, and step() filters it far more slowly than
    // the input filters that gate the phase snap - so when the gait switches it can still hold a
    // fraction of the previous session's stride. Cleared here so the robot starts from standstill.
    //
    // Not applied when switching between walking gaits. That currently cannot happen mid-stride
    // anyway (canChange() requires phase 0, which requires input to have stopped), but keeps the
    // reset correct if walking gaits ever become switchable mid-cycle.
    const bool wasLocomotion = (myActiveGait != nullptr &&
                                myActiveGait->getCategory() == GaitCategory::eLocomotion);

    // Captured before myActiveGait is reassigned. Null for anything that is not a morph.
    const GaitConfig* handover = (isMorph() == true) ? myActiveGait->activeConfig() : nullptr;

    myActiveGait  = myDesiredGait;
    myDesiredGait = nullptr;

    if (handover == nullptr)
    {
      myDurationMS = myDesiredDurationMS;
      myGaitPhase  = 0;
    }
    // else cadence and phase both carry on unbroken. The phase, because the legs read their
    // position from it directly and restarting it at 0 would move every foot at the instant of
    // the switch. The cadence, because a locomotion request carries no duration of its own -
    // the stick sets it - so myDesiredDurationMS is still cMaxDurationMS here, and taking it
    // would drop a robot walking at full speed to the slowest cycle and leave the filter in
    // step() to wind it back up over the next couple of seconds.

    if (myActiveGait->getCategory() == GaitCategory::eLocomotion && wasLocomotion == false)
    {
      myLinearX = myLinearY = myAngularZ = 0.0f;
      myCurrentMotion = MotionCmd{};
    }

    // A posture gait starts from a neutral body offset rather than wherever the previous posing
    // session left it.
    if (myActiveGait->getCategory() == GaitCategory::ePosture)
      myCurrentMotion.pose = BodyPose{};

    // myActiveGait is guaranteed non-null here: myDesiredGait was checked non-null
    // above and just assigned to it, with nothing in between that could clear it.
    // beginFrom() starts from the pattern the outgoing gait was walking, so no foot moves at
    // the instant of the switch. It reports false if it cannot take that pattern on, in which
    // case the ordinary standing start applies.
    if (handover == nullptr || myActiveGait->beginFrom(*handover, myCurrentMotion, myGaitPhase) == false)
    {
      myDurationMS = myDesiredDurationMS;
      myGaitPhase  = 0;
      myActiveGait->begin();
    }
  }
}
