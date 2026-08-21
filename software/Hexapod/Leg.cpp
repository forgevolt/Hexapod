#include "ServoBus.h"
#include <Streaming.h>
#include "Leg.h"
#include "MathUtilities.h"

// ---- Leg -------------------------------------------------------------------------------

// Leg segment geometry constants (in millimeters) measured between servo output shafts
constexpr float cCoxaLength  = 42.35f;   // Coxa to femur joint distance 
constexpr float cFemurLength = 95.0f;    // Femur length
constexpr float cTibiaLength = 174.981f; // Tibia length (joint to foot)

// Leg geometry angle offsets (radians).
// These angles compensate for bends in the femur and tibia segments
// and are applied relative to the neutral (zero-angle) joint configuration.
constexpr float cFemurOffsetAngleRad = 17.2f * deg2rad;
constexpr float cTibiaOffsetAngleRad = -8.7f * deg2rad;

// ----------------------------------------------------------------------------------------
Leg::Leg(LegId id, ServoBus& servoBus, size_t coxaServoIdx, size_t femurServoIdx, size_t tibiaServoIdx)
: myId(id), 
  myServoBus(servoBus),
  myCoxaServoIdx(coxaServoIdx), 
  myFemurServoIdx(femurServoIdx), 
  myTibiaServoIdx(tibiaServoIdx)
{
  switch (id)
  {
    case LegId::LF:
      myOffset = Vector3(82.399, 53.511, 15.05);
      myYaw    = 33.0f * deg2rad; // == yaw = atan2f(53.511, 82.399)
      break;
    case LegId::LM:
      myOffset = Vector3(0, 87.25, 15.05);
      myYaw    = 90.0f * deg2rad;
      break;
    case LegId::LR:
      myOffset = Vector3(-82.399, 53.511, 15.05);
      myYaw    = 147.0f * deg2rad;
      break;
    case LegId::RF:
      myOffset = Vector3(82.399, -53.511, 15.05); 
      myYaw    = -33.0f * deg2rad; 
      break;
    case LegId::RM:
      myOffset = Vector3(0, -87.25, 15.05); 
      myYaw    = -90.0f * deg2rad;
      break;
    case LegId::RR:
      myOffset = Vector3(-82.399, -53.511, 15.05);
      myYaw    = -147.0f * deg2rad; 
      break;
    default:
      // Invalid ID: fall back to safe, neutral values rather than leaving myOffset/
      // myYaw however the (non-guaranteed) member defaults left them.
      Serial << __PRETTY_FUNCTION__ << " -> invalid leg ID: " << static_cast<int>(id) << endl;
      myOffset = Vector3(0, 0, 0);
      myYaw    = 0.0f;
      break;
  }

  myCosYaw = cosf(myYaw);
  mySinYaw = sinf(myYaw);
}

// ----------------------------------------------------------------------------------------
// Based on the hexapod kinematics tutorial by Jakob Leander:
// https://youtu.be/WAsMAeKDc4U
bool Leg::setPosition(const Vector3& pos)
{
  // -------------------------------------------------------------------------
  // Shift origin: target position with coxa as the origin (world to leg local)
  // -------------------------------------------------------------------------
  float dx = pos.x - myOffset.x;
  float dy = pos.y - myOffset.y;
  float dz = pos.z - myOffset.z;

  // -------------------------------------------------------------------------
  // Rotate into the leg's local frame by undoing the mounting yaw.
  // myYaw (and myOffset above) already carry the correct sign for each leg -
  // positive for the left side, negative for the right, set in the constructor -
  // so this single formula handles all six legs identically. No left/right branch
  // is needed: rotating by a negative angle already does the right thing.
  // -------------------------------------------------------------------------
  Vector3 localPos(dx * myCosYaw + dy * mySinYaw,
                   -dx * mySinYaw + dy * myCosYaw,
                    dz);

  // -------------------------------------------------------------------------
  // Coxa calculation 
  // -------------------------------------------------------------------------
  float coxa = atan2f(localPos.y, localPos.x);

  // -------------------------------------------------------------------------
  // Femur and Tibia calculation
  // -------------------------------------------------------------------------
  float h = sqrtf( sqr(localPos.x) + sqr(localPos.y) ) - cCoxaLength;
  float l = sqrtf( sqr(h) + sqr(localPos.z) );

  // Reachability: l is the distance from the femur joint to the target in the leg's vertical
  // plane, and the reachable set is an annulus of radii |cFemurLength - cTibiaLength| and
  // cFemurLength + cTibiaLength.
  //
  // When the target lies outside it, l is clamped while alpha (computed below from the
  // *unclamped* h and localPos.z) is deliberately left alone. That combination projects the
  // target onto the nearest point of the annulus - same bearing, clamped reach - which is the
  // correct fallback. Do not "fix" this by recomputing h and z from the clamped l.
  bool reachable = true;

  // Check reachability...
  if (l > cFemurLength + cTibiaLength || l < fabsf(cFemurLength - cTibiaLength)) 
  {
    reachable = false;

    // Signed overshoot in mm: positive = beyond cFemurLength + cTibiaLength, negative = inside
    // |cFemurLength - cTibiaLength|. The two cases are mutually exclusive, since the sum of two
    // positive link lengths always exceeds their absolute difference.
    const float overshoot = (l > cFemurLength + cTibiaLength)
                          ? (l - (cFemurLength + cTibiaLength))
                          : (l - fabsf(cFemurLength - cTibiaLength));

    ++myUnreachableTargets;

    // Keep the worst case, not the most recent, so a one-second summary is not dominated by
    // whichever target happened to be reported last.
    if (fabsf(overshoot) > fabsf(myWorstOvershoot))
      myWorstOvershoot = overshoot;

    // Handle gracefully: clamp to the nearest reachable length
    l = std::clamp(l, fabsf(cFemurLength - cTibiaLength), cFemurLength + cTibiaLength);
  }

  // Femur IK
  float alpha  = atan2f(localPos.z, h);
  float beta   = acosf(std::clamp((sqr(cFemurLength) + sqr(l) - sqr(cTibiaLength)) / (2 * cFemurLength * l), -1.0f, 1.0f));
  float theta2 = alpha + beta;
  float femur = theta2 - cFemurOffsetAngleRad;

  // Tibia IK
  float gamma = acosf(std::clamp((sqr(cFemurLength) + sqr(cTibiaLength) - sqr(l)) / (2 * cFemurLength * cTibiaLength), -1.0f, 1.0f));
  float theta3 = gamma - cPI/2; // Offset by -cPI/2 to align tibia angle with neutral/reference position
  float tibia = -theta3 - cFemurOffsetAngleRad + cTibiaOffsetAngleRad;   // Negate theta3 to match tibia rotation direction

  // -------------------------------------------------------------------------
  // Send to Servos
  // -------------------------------------------------------------------------
  myServoBus.setGoalPositionRad(myCoxaServoIdx,  coxa);
  myServoBus.setGoalPositionRad(myFemurServoIdx, femur);
  myServoBus.setGoalPositionRad(myTibiaServoIdx, tibia);

  return reachable;      
}

// ----------------------------------------------------------------------------------------
Vector3 Leg::computePositionFromAngles(float coxa, float femur, float tibia) const
{
  // -------------------------------------------------------------------------
  // Femur/tibia math is identical for every leg regardless of side - it only ever
  // depends on femur/tibia themselves, never on myYaw or myOffset's sign.
  // -------------------------------------------------------------------------
  float femurMath = femur + cFemurOffsetAngleRad;

  // Tibia angle relative to Femur
  float theta3 = -(tibia + cFemurOffsetAngleRad - cTibiaOffsetAngleRad);
  float tibiaMath = femurMath + (theta3 - (cPI / 2.0f));

  // Calculate 2D "Planar" Reach (h) and Height (z)
  float h = cCoxaLength; 
  float z = 0.0f;

  // Add Femur's contribution to horizontal reach and vertical height
  h += cFemurLength * cosf(femurMath);
  z += cFemurLength * sinf(femurMath);

  // Add Tibia's contribution 
  h += cTibiaLength * cosf(tibiaMath);
  z += cTibiaLength * sinf(tibiaMath);

  // -------------------------------------------------------------------------
  // Project into 3D world space: rotate the local (coxa, h) position by the
  // mounting yaw and add the mounting offset back. myYaw/myOffset already carry
  // the correct sign per leg, so - as in setPosition() - no left/right branch
  // is needed here either.
  // -------------------------------------------------------------------------
  float coxaMath = coxa + myYaw;

  Vector3 result;
  result.x = h * cosf(coxaMath) + myOffset.x;
  result.y = h * sinf(coxaMath) + myOffset.y;
  result.z = z + myOffset.z;

  return result;    
}

// ---------------------------------------------------------------------------------------------
Vector3 Leg::getPresentPosition() const
{
  return computePositionFromAngles(tickToRad(myServoBus.presentPosition(myCoxaServoIdx)),
                                   tickToRad(myServoBus.presentPosition(myFemurServoIdx)),
                                   tickToRad(myServoBus.presentPosition(myTibiaServoIdx)));
}

// ---------------------------------------------------------------------------------------------
Vector3 Leg::getCurrentGoal() const
{
  return computePositionFromAngles(tickToRad(myServoBus.currentGoal(myCoxaServoIdx)),
                                   tickToRad(myServoBus.currentGoal(myFemurServoIdx)),
                                   tickToRad(myServoBus.currentGoal(myTibiaServoIdx)));
}

// ---------------------------------------------------------------------------------------------
Leg::Diagnostics Leg::fetchDiagnostics()
{
  Diagnostics d;

  d.unreachableTargets = myUnreachableTargets.exchange(0);
  d.worstOvershoot     = myWorstOvershoot;

  myWorstOvershoot = 0.0f;

  return d;
}
