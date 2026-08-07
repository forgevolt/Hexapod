#pragma once

#include "Leg.h"
#include "GaitEngine.h"

// ---- Gait ------------------------------------------------------------------------------

class Hexapod;
struct GaitConfig;

enum class GaitCategory 
{
  ePosture,           // Body transformation gaits
  ePostureTransition, // One-shot transitions between postures, such as standing up
  eGesture,           // For gestures such as waving a leg
  eLocomotion,        // Walking gaits
  eBalance            // Sensor-driven posture hold; no transmitter input required
};

class Gait 
{
  public:
    Gait(Hexapod& hexapod, 
         const String& gaitName="", 
         GaitCategory category = GaitCategory::ePostureTransition) 
    : myHexapod(hexapod), myName(gaitName), myCategory(category)
    {}
    virtual ~Gait() {}

    const String& name() const { return myName; }

    // Begin with the gait trajectory
    virtual void begin() = 0;

    // Update leg positions
    //   phase: normalized gait cycle position (0.0 - 1.0, wrapping)
    //   cmd:   locomotion commands (translation, rotation)
    virtual void update(float phase, const GaitEngine::MotionCmd& cmd) = 0;

    // Called by the GaitEngine when phase wraps from 1.0 to 0.0
    virtual void cycleComplete() {}

    // Can this gait be interrupted right now?
    virtual bool canChange() const = 0;

    // ---- Mid-stride handover -----------------------------------------------------------
    // The leg pattern this gait is walking at this moment. For a walking gait mid-morph that
    // is a blend, not its own table, which is what lets the next gait pick the blend up where
    // it stands. Gaits that do not walk have no pattern and take no part in this.
    virtual const GaitConfig* activeConfig() const { return nullptr; }

    // Begin from `previous` rather than from this gait's own pattern, blending across over
    // the following couple of cycles. `cmd` is the motion in force and `phase` the engine's
    // current cycle position; together they fix where the feet will be during the blend, and
    // therefore which blend path keeps the robot best supported. Returning false leaves the
    // engine to perform an ordinary switch.
    virtual bool beginFrom(const GaitConfig& /*previous*/,
                           const GaitEngine::MotionCmd& /*cmd*/,
                           float /*phase*/) { return false; }

    // Fraction of the engine's fastest cycle this gait may reach (0.0 - 1.0]. Only the walking
    // gaits limit themselves; everything else runs at whatever duration requestGait() gave it.
    virtual float speedLimit() const { return 1.0f; }

    GaitCategory getCategory() const { return myCategory; }

  protected:
    Hexapod& myHexapod;

    const String myName;
    GaitCategory myCategory;
};


// ---- ParkGait --------------------------------------------------------------------------
// Handles the transition to a static "Parked" pose.
// Interpolates all legs from their current positions to the neutral home coordinates.

class ParkGait : public Gait 
{
  public:
    ParkGait(Hexapod& hexapod);

    // ---- From class Gait
    void begin() override;
    void update(float phase, const GaitEngine::MotionCmd& cmd) override;
    void cycleComplete() override;
    bool canChange() const override { return myCanChange; }

  private:
    Vector3 myStart[cNumLegs]; // Stores the initial leg positions at the moment begin() is called
    bool myCanChange = false;
};


// ---- StandUpGait -----------------------------------------------------------------------
// Manages the vertical transition from a parked/low position to a neutral standing stance.
// Synchronizes leg movement to ensure all feet reach their target posture simultaneously.

class StandUpGait : public Gait 
{
  public:
    StandUpGait(Hexapod& hexapod);

    // ---- From class Gait
    void begin() override;
    void update(float phase, const GaitEngine::MotionCmd& cmd) override;
    void cycleComplete() override;
    bool canChange() const override { return myCanChange; }

  private:
    Vector3 myStart[cNumLegs];  // Initial foot positions captured at the start of the gait
    Vector3 myTarget[cNumLegs]; // Final standing coordinates for each foot in the body frame
    float myMaxDistance = 0.0f; // Maximum path length required for the foot tip to reach its goal
    bool myCanChange = false;
};


// ---- PosingGait ------------------------------------------------------------------------
// Static-foot gait allowing real-time body orientation and height adjustments.
// Keeps feet planted while translating/rotating the chassis based on global 
// GaitEngine::BodyPose settings.

class PosingGait : public Gait 
{
  public:
    PosingGait(Hexapod& hexapod);

    // ---- From class Gait
    void begin() override;
    void update(float phase, const GaitEngine::MotionCmd& cmd) override;
    bool canChange() const override { return true; } 
};


// ---- LevelGait -------------------------------------------------------------------------
// Static-foot gait that keeps the body horizontal on uneven terrain.
// Reads roll/pitch from the hexapod's onboard MPU6050 (Hexapod::imu()) and counter-rotates
// the body to compensate, the same way PosingGait counters transmitter-commanded rotation,
// but driven by the sensor instead of the pilot. Feet stay planted; the robot does not walk
// while active.

class LevelGait : public Gait 
{
  public:
    LevelGait(Hexapod& hexapod);

    // ---- From class Gait
    void begin() override;
    void update(float phase, const GaitEngine::MotionCmd& cmd) override;
    bool canChange() const override { return true; }

  private:
    Vector3 myFilteredTilt = Vector3(0, 0, 0); // extra smoothing on top of the IMU's own filter,
                                               // to keep servo motion gentle even if the sensor
                                               // reading is momentarily jumpy

    // Tuning constants - adjust to taste
    static constexpr float cFilterAlpha    = 0.1f;  // 0..1, higher = snappier but noisier
    static constexpr float cMaxCorrection  = 0.35f; // radians (~20 deg), safety clamp on correction
};


// ---- GenericWalkingGait ----------------------------------------------------------------
// Versatile locomotion engine that implements various walking patterns.
// Uses timing offsets and swing durations to translate MotionCmd into synchronized leg cycles.
 
struct GaitConfig
{
  float swingDuration;        // Percentage of cycle leg is in air (0.0 - 1.0)
  float legOffsets[cNumLegs]; // Start phase for each leg (0.0 - 1.0)

  // Fraction of the engine's fastest cycle this gait may reach (0.0 - 1.0]. A gait that gives
  // each leg a short swing window has to move that leg faster for the same cycle time, so not
  // every gait can follow cMinDurationMS. 1.0 means no limit beyond it.
  float speedLimit;
};

// maximum speed on flat terrain (Fast & efficient)
const GaitConfig cTripodGaitConfig = { 
    0.5f, 
    {
        0.0f,  // LF
        0.5f,  // LM
        0.0f,  // LR
        0.5f,  // RF
        0.0f,  // RM
        0.5f   // RR
    },
    1.0f // swings each leg over half the cycle: the servos keep up at full rate
};

// moderate speed, good stability (Balanced speed vs stability)
const GaitConfig cTetrapodGaitConfig = {
    1.0f / 3.0f,
    {
        0.0f/3.0f,  // LF
        1.0f/3.0f,  // LM
        2.0f/3.0f,  // LR
        1.0f/3.0f,  // RF
        2.0f/3.0f,  // RM
        0.0f/3.0f   // RR
    },
    0.7f 
};

// uneven terrain, moderate speed, carrying payloads (Stable & smooth)
const GaitConfig cRippleGaitConfig = { 
    // 1.0f / 6.0f,
    0.32f,
    // 0.25f,
    {
        0.0f/6.0f,  // LF
        2.0f/6.0f,  // LM
        4.0f/6.0f,  // LR
        3.0f/6.0f,  // RF
        5.0f/6.0f,  // RM
        1.0f/6.0f   // RR
    },
    0.7f // a sixth of the cycle per swing; the servos cannot follow the full rate
};

// maximum stability > speed (Ultra-stable, slow)
const GaitConfig cWaveGaitConfig = {
    1.0f / 6.0f,
    {
        0.0f/6.0f,  // LF
        1.0f/6.0f,  // LM
        2.0f/6.0f,  // LR
        3.0f/6.0f,  // RF
        4.0f/6.0f,  // RM
        5.0f/6.0f   // RR
    },
    0.5f // same swing window as ripple; not enabled in Hexapod.cpp
};

// precise foot placement, climbing, sensors active (Very controlled, precise)
const GaitConfig cCrawlGaitConfig = {
    0.2f,
    {
        0.0f/6.0f,  // LF
        1.0f/6.0f,  // LM
        2.0f/6.0f,  // LR
        3.0f/6.0f,  // RF
        4.0f/6.0f,  // RM
        5.0f/6.0f   // RR
    },
    0.5f // shortest swing window of all; not enabled in Hexapod.cpp
};    


class GenericWalkingGait : public Gait 
{
  public:
    GenericWalkingGait(Hexapod& hexapod, const String& gaitName, const GaitConfig& config);

    // ---- From class Gait
    void begin() override;
    void update(float phase, const GaitEngine::MotionCmd& cmd) override;
    bool canChange() const override; 
    float speedLimit() const override { return myLiveConfig.speedLimit; }

    const GaitConfig* activeConfig() const override { return &myLiveConfig; }
    bool beginFrom(const GaitConfig& previous, const GaitEngine::MotionCmd& cmd,
                   float phase) override;

  private:
    // Where leg `leg` places its foot at `legPhase` of a cycle whose swing fraction is
    // `swing`. Depends on nothing but its arguments, so the blend-path search can evaluate
    // stances the robot is not currently in.
    Vector3 footPosition(std::size_t leg, float legPhase, float swing,
                         const GaitEngine::MotionCmd& cmd,
                         const GaitEngine::GaitParams& params) const;

    // Fills myOffsetDelta with the shift each leg makes to reach this gait's pattern.
    void chooseOffsetPath(const GaitEngine::MotionCmd& cmd, float phase);

    // Smallest support margin met along the blend myOffsetDelta currently describes, starting
    // from engine phase `startPhase`.
    float worstMarginOnPath(const GaitEngine::MotionCmd& cmd, float startPhase) const;

    // Rebuilds myLiveConfig for the current myMorphAlpha.
    void blendConfig();

    // Cycles the blend takes. Long enough that the pattern changes gently, short enough that
    // the robot is not walking an intermediate pattern for any length of time.
    static constexpr float cMorphCycles = 2.0f;

    // A leg whose offset shift is exactly half a cycle has no shorter direction, and the two
    // are not equivalent: one of them can carry the robot through a stance that will not hold
    // it. chooseOffsetPath() decides those by measurement. Ties are rare - at most one leg
    // between any two of the gaits below - and the search costs 2^n stance evaluations.
    static constexpr std::size_t cMaxTiedLegs = 3;

    // Samples taken along the blend when measuring it. The blend is a single path, not a
    // field: alpha and the engine phase advance together, so the stances it actually passes
    // through lie on one line. Sampling the two independently reports stances the robot never
    // adopts, and rates both directions of a tie as impossible.
    static constexpr std::size_t cPathSteps = 96;

    // Half-width of the band that counts as an exact half-cycle tie.
    static constexpr float cPhaseTie = 1.0e-4f;

    // Bank reached when turning hardest at full stride, as a tangent rather than an angle
    // because that is how it is applied. The body rolls toward the inside of the turn. This is
    // chosen by eye, not derived: at this robot's speed the lateral acceleration of even the
    // tightest turn calls for a bank under one degree, which would not be visible.
    static constexpr float cMaxLeanTilt = 0.1228f; // tan(7 degrees)

    GaitConfig myGaitConfig; // this gait's own pattern
    GaitConfig myFromConfig; // the pattern the blend starts from
    GaitConfig myLiveConfig; // the pattern in force this step

    float myOffsetDelta[cNumLegs] = {}; // signed shift from myFromConfig to myGaitConfig
    float myMorphAlpha = 1.0f;          // 0 = myFromConfig, 1 = myGaitConfig

    float myCurrentPhase = 0.0f; // kept for canChange()
    float myLastPhase    = 0.0f; // to recover the phase delta between updates
    bool  myPhaseSeeded  = false;
};

