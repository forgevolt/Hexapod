#pragma once

#include "MathUtilities.h"
#include "Receiver.h"

// ---- GaitEngine ------------------------------------------------------------------------

class Hexapod;
class Gait;

class GaitEngine 
{
  public:
    // ----- Body transformation offsets
    struct BodyPose
    {
      Vector3 translation; // X, Y, Z offset
      Vector3 rotation;    // Roll, Pitch, Yaw in radians
    };

    // ----- Motion information for gaits
    struct MotionCmd
    {
      Vector2 linear;   // Translation velocity: x = Forward/Back, y = Left/Right
      float yaw = 0.0f; // Rotational velocity (rad): Positive = CCW, Negative = CW

      // Body offset for posture gaits. Filtered from operator input the same way linear/yaw are,
      // and delivered through the same channel.
      BodyPose pose;

      // How hard the operator is asking the robot to move, 0 at rest and 1 at full deflection.
      // Carried separately from linear/yaw because it is not recoverable from them: a stride
      // reversing from forward to back passes through zero on the way, and at that instant the
      // stride says "stationary" while the stick says "full ahead". Gaits scale foot lift with
      // this, so reading it off the stride alone puts every foot on the ground mid-reversal.
      float demand = 0.0f;
    };

    static constexpr float cMinPoseTranslation = -80.0f; // mm
    static constexpr float cMaxPoseTranslation =  80.0f; // mm

    static constexpr float cMinPoseRotation = deg2rad*-15.0f; // rad
    static constexpr float cMaxPoseRotation = deg2rad*15.0f;  // rad

    // ----- Gait parameters
    // Adjusted at runtime from the transmitter (see step()) and clamped to the c*-limits below.
    struct GaitParams
    {
      float groundClearance;
      float stepHeight;
      float stepLength;
    };

    // ----- Clearance
    // NOTE: "more negative" = the chassis sits lower (further below the coxa joints),
    // so cMinGroundClearance (-60) is numerically *larger* than cMaxGroundClearance
    // (-150) - the names describe the clearance extremes, not ascending numeric order.
    // Every clamp() call site already passes these in the correct low/high numeric
    // order; if you ever "fix" the apparent min/max swap here, check those call sites.
    static constexpr float cMinGroundClearance     = -60.0f;  // mm
    static constexpr float cDefaultGroundClearance = -95.0f;  // mm
    static constexpr float cMaxGroundClearance     = -150.0f; // mm

    // ----- Motion limits (i.e. min/max speed of the hexapod)
    static constexpr float cMinDurationMS     = 600.0f;  // ms, fastest gait
    static constexpr float cMaxDurationMS     = 5000.0f; // ms, slowest gait

    static constexpr float cMaxYaw      = deg2rad*22.0f; // rad

    // ----- Gait tuning
    static constexpr float cMinStepLength     = 20.0f;   // mm
    static constexpr float cDefaultStepLength = 110.0f;  // mm
    static constexpr float cMaxStepLength     = 150.0f;  // mm

    static constexpr float cMinStepHeight     = 40.0f;   // mm
    static constexpr float cDefaultStepHeight = 55.0f;   // mm
    static constexpr float cMaxStepHeight     = 70.0f;   // mm

    // ----- Default poses
  
    inline static const Vector3 cParkPosition[] = {
      {  227.0f,  147.5f, 50.0f }, // LF
      {    0.0f,  260.0f, 50.0f }, // LM
      { -227.0f,  147.5f, 50.0f }, // LR
      {  227.0f, -147.5f, 50.0f }, // RF
      {    0.0f, -260.0f, 50.0f }, // RM
      { -227.0f, -147.5f, 50.0f }  // RR
    };
    
    inline static const Vector3 cNeutralPosition[] = {
      {  201.0f,  130.0f, cDefaultGroundClearance }, // LF
      {    0.0f,  229.0f, cDefaultGroundClearance }, // LM
      { -201.0f,  130.0f, cDefaultGroundClearance }, // LR
      {  201.0f, -130.0f, cDefaultGroundClearance }, // RF
      {    0.0f, -229.0f, cDefaultGroundClearance }, // RM
      { -201.0f, -130.0f, cDefaultGroundClearance }  // RR
    };
    
  public:
    GaitEngine(Hexapod& hexapod);

    // Update leg positions. 
    //   dt_ms is time passed in milliseconds.
    //   input is the user input received from the transmitter.
    void step(float dt_ms, const Receiver::ControlData& input);

    // Request a new gait
    // Note, that for locomotion gaits, durationMS should be omitted since the phase duration is based on 
    // joystick movements
    void requestGait(Gait* gait, float durationMS = cMaxDurationMS);

    Gait* currentGait() { return myActiveGait; }

    bool canChangeGait() const;

    // The parameters in force this step: the tuned values plus any transient trim from the
    // right stick.
    const GaitParams& params() const { return myLiveParams; }

    // Back to the cDefault* values.
    void resetParams();

  private:
    void trySwitchGait();

    // Is the pending change one walking gait replacing another? Those blend rather than
    // switch: the incoming gait takes over the pattern currently being walked and interpolates
    // to its own, so it needs no cycle boundary - which, for a walking gait, only ever arrives
    // once the operator has let go of the stick.
    bool isMorph() const;

  private:
    Hexapod& myHexapod; // Robot reference

    // ----- Input smoothing
    float myLinearX = 0.0f, myLinearY = 0.0f, myAngularZ = 0.0f;
 
    // ----- Gait switching
    Gait* myActiveGait = nullptr;
    float myDurationMS = 1000.0f;

    Gait* myDesiredGait = nullptr;
    float myDesiredDurationMS = 1000.0f;

    float myGaitPhase = 0.0f;  // Normalized cycle position [0.0 to 1.0]. 0 = Start, 1 = Cycle Complete.

    MotionCmd myCurrentMotion; // Motion state
    GaitParams myParams;       // set by resetParams() from the constructor
    GaitParams myLiveParams;   // myParams with myClearanceTrim applied; see step()

    // Transient ground-clearance offset in mm, driven by RY while a locomotion gait is
    // active and switch 1 is not held. Never written back to myParams.
    float myClearanceTrim = 0.0f;

};

