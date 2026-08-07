#pragma once

#include "MathUtilities.h"
#include "ServoBus.h"
#include <cstddef>   // std::size_t - used at line 36 and below
#include <atomic>


// ---- Leg -------------------------------------------------------------------------------
// The Leg class encapsulates the kinematic model and servo control for one hexapod leg,
// consisting of three joints: coxa, femur, and tibia.
//
// Responsibilities:
// - Compute inverse kinematics to convert a desired foot position into joint angles.
// - Command the corresponding Dynamixel servos via a shared ServoBus.
// - Compute forward kinematics to determine the foot position from given joint angles.
//
// The leg coordinate system is defined relative to the hexapod body frame.
// Each leg has a fixed mounting offset and yaw, allowing the same kinematic
// implementation to be reused for all legs - including the left/right distinction,
// which is fully captured by the sign of the mounting offset and yaw (see the
// constructor) and requires no special-casing anywhere in the IK/FK math itself.
//
// This class does not manage gait generation or timing; it only handles
// per-leg kinematics and servo target updates.

enum class LegId
{
    LF = 0, // left front
    LM,     // left middle
    LR,     // left rear
    RF,     // right front
    RM,     // right middle
    RR,     // right rear
    numLegs // --> 6
};

constexpr std::size_t cNumLegs = static_cast<std::size_t>(LegId::numLegs);

// Coxa, femur, tibia
constexpr std::size_t cJointsPerLeg = 3;

// Each leg drives three servos on the shared bus, with the index triples assigned in Hexapod's
// constructor. Changing either count alone would leave servos unclaimed or point a leg past the
// end of ServoBus' arrays.
static_assert(static_cast<std::size_t>(ServoBus::cNumServos) == cNumLegs * cJointsPerLeg,
              "cNumServos must equal cNumLegs * cJointsPerLeg");

// Helper function to cast LegId to std::size_t for array indexing
constexpr std::size_t idx(LegId id) 
{
  return static_cast<std::size_t>(id);
}


class Leg
{
  public:
    Leg(LegId id, ServoBus& servoBus, size_t coxaServoIdx, size_t femurServoIdx, size_t tibiaServoIdx);

  public:
    // Computes servo angles for 'pos' using an inverse kinematics algorithm.
    // Sets the goal positions of the leg's servos.
    // Returns true if the position is reachable, false if it had to be clamped to the
    // nearest reachable point (the leg is still moved to that clamped position).
    bool setPosition(const Vector3& pos);

    // Computes the leg's end-effector position from the given servo angles
    // using forward kinematics. Does not modify servo targets or state.
    Vector3 computePositionFromAngles(float coxa, float femur, float tibia) const;

    // Returns the leg's mounting origin relative to the body center.
    // x/y: Horizontal offset from chassis center. 
    // z: Vertical offset relative to the femur (thigh) axis.
    const Vector3& getOffset() const { return myOffset; }

    // Performs forward kinematics (FK) to calculate the current foot coordinates.
    // Maps the actual servo angles to a 3D position relative to the leg base.
    // Note: Requires a prior call to ServosBus::syncReadPresentPosition() to refresh local data.
    Vector3 getPresentPosition() const; 

    // Similar to getPresentPosition():
    // Performs Forward Kinematics based on target joint angles.
    // Calculates the expected 3D foot position using the current servo Goal Positions.
    Vector3 getCurrentGoal() const;

    // Conditions setPosition() detects but cannot usefully act on.
    struct Diagnostics
    {
      uint32_t unreachableTargets = 0; // IK targets clamped to the reachable workspace

      // Worst violation since the last fetch, in mm. Positive = beyond cFemurLength+cTibiaLength
      // (too far), negative = inside |cFemurLength-cTibiaLength| (too close).
      // Meaningless when unreachableTargets == 0.
      float worstOvershoot = 0.0f;
    };

    // Returns the counts accumulated since the last call, resetting them. Call from loop().
    Diagnostics fetchDiagnostics();

  private:
    LegId myId;
    Vector3 myOffset;      // Coxa joint position relative to the hexapod center (0,0,0);
                           // Z coordinate is referenced to the femur servo output shaft
    float myYaw    = 0.0f; // mounting yaw (rad) - positive for left-side legs, negative for right

    // cosf(myYaw)/sinf(myYaw), cached once since myYaw is fixed for the lifetime of a
    // Leg but setPosition()/computePositionFromAngles() may be called at high frequency
    // (e.g. once per leg per gait tick) - avoids recomputing the same trig calls on
    // every single call.
    float myCosYaw = 1.0f;
    float mySinYaw = 0.0f;

    ServoBus& myServoBus;

    // Indices of the leg's servos (coxa, femur, tibia)
    size_t myCoxaServoIdx;
    size_t myFemurServoIdx;
    size_t myTibiaServoIdx;

    // Atomic: incremented on the control task, exchanged to 0 from loop().
    std::atomic<uint32_t> myUnreachableTargets{0};

    // Worst-case detail for the counter above. Plain member, not atomic: only the control task
    // writes it and only loop() reads it, and the worst consequence of a torn read is one
    // slightly-wrong diagnostic figure - not worth a lock in the 200 Hz path.
    float myWorstOvershoot = 0.0f;
};
