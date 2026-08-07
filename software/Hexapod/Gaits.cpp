#include "GaitEngine.h"
#include "Gaits.h"
#include "Hexapod.h"
#include "MathUtilities.h"

#include <Streaming.h>
#include <algorithm>


// ---- ParkGait --------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
ParkGait::ParkGait(Hexapod& hexapod)
: Gait(hexapod, "Park") 
{}

// ----------------------------------------------------------------------------------------
void ParkGait::begin() 
{
  // Get present position of each leg
  myHexapod.servoBus().syncReadPresentPosition();
  for (std::size_t i = idx(LegId::LF); i <= idx(LegId::RR); i++)
  {
    myStart[i] = myHexapod.leg(i).getPresentPosition();

    // Seed the goal position with where the leg actually is *before* torque gets
    // enabled below. Otherwise setSafeTorque() would make the servo hold whatever
    // stale goal was left in its onboard register from before this gait (or before
    // this boot) - potentially snapping toward it at full speed for the one scheduler
    // tick before update() gets a chance to compute the real interpolated position.
    myHexapod.leg(i).setPosition(myStart[i]);
  }
  myHexapod.servoBus().syncWrite();

  myCanChange = false;
  myHexapod.servoBus().setSafeTorque();
}

// ----------------------------------------------------------------------------------------
void ParkGait::update(float phase, const GaitEngine::MotionCmd&) 
{
  if (myCanChange == true)
    return;

  phase = std::clamp(phase, 0.0f, 1.0f);

  constexpr float cParkLiftHeight = 50.0f; // Park-specific lift height
  constexpr float flareFactor = 1.2f;      // 1.0 is no flare, 1.2 is 20% further out

  // Smooth the timing for a more natural feel
  float t = smoothstep(phase);

  for (std::size_t i = 0; i < cNumLegs; i++)
  {
    // The path is a cubic Bezier. lerp(), smoothstep() and minjerk() in MathUtilities.h are
    // simpler alternatives over the same start/target pair.

    // Define the trajectory
    Vector3 p0 = myStart[i];
    Vector3 p3 = GaitEngine::cParkPosition[i];

    // Control Point 1: Lifted and Flared from Start
    Vector3 p1 = p0;
    p1.x *= flareFactor;
    p1.y *= flareFactor;
    p1.z += cParkLiftHeight;

    // Control Point 2: Lifted and Flared from Target
    Vector3 p2 = p3;
    p2.x *= flareFactor;
    p2.y *= flareFactor;
    p2.z += cParkLiftHeight;

    // Calculate the 3D arc position
    Vector3 pos = bezier3(p0, p1, p2, p3, t);

    myHexapod.leg(i).setPosition(pos);
  }

  myHexapod.servoBus().syncWrite();
}

// ----------------------------------------------------------------------------------------
void ParkGait::cycleComplete()
{
  myCanChange = true;
}


// ---- StandUpGait -----------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
StandUpGait::StandUpGait(Hexapod& hexapod)
: Gait(hexapod, "Standup") 
{}

// ----------------------------------------------------------------------------------------
void StandUpGait::begin()
{
  const GaitEngine::GaitParams& params = myHexapod.gaitParams();

  for (std::size_t i = 0; i < cNumLegs; i++)
  {
    myTarget[i].x = GaitEngine::cNeutralPosition[i].x;
    myTarget[i].y = GaitEngine::cNeutralPosition[i].y;
    myTarget[i].z = params.groundClearance;
  }

  myMaxDistance = 0.0f;

  if (myHexapod.servoBus().isAllTorqueOn() == false)
  {
    // Get present position of each leg
    myHexapod.servoBus().syncReadPresentPosition();
    for (std::size_t i = idx(LegId::LF); i <= idx(LegId::RR); i++)
    {
      myStart[i] = myHexapod.leg(i).getPresentPosition();
      myMaxDistance = max(myMaxDistance, myStart[i].distance(myTarget[i]));
      myHexapod.leg(i).setPosition(myStart[i]);
    }
    myHexapod.servoBus().syncWrite();
  }
  else 
  {
    for (std::size_t i = idx(LegId::LF); i <= idx(LegId::RR); i++)
    {
      // Get the current goal position for each leg.
      // Using goal positions instead of measured positions yields smoother motion by compensating 
      // for servo backlash and gravity-induced loading.
      myStart[i] = myHexapod.leg(i).getCurrentGoal();
      myMaxDistance = max(myMaxDistance, myStart[i].distance(myTarget[i]));
    }
  }

  myCanChange = false;
  myHexapod.servoBus().setMaxTorque();
}

// ----------------------------------------------------------------------------------------
void StandUpGait::update(float phase, const GaitEngine::MotionCmd&) 
{
  if (myCanChange == true)
    return;

  phase = std::clamp(phase, 0.0f, 1.0f);

  // Use "Smootherstep" for the lift to avoid jerky motor starts
  float t = smootherstep(phase);

  // Below this distance, legs are already close enough to their target that a simple
  // minjerk curve looks fine; above it, use the two-stage bezier lift-and-place path
  // so far-traveling legs don't drag across the ground on the way up.
  constexpr float cMinjerkDistanceThresholdMM = 10.0f;

  for (std::size_t i = 0; i < cNumLegs; i++)
  {
    if (myMaxDistance < cMinjerkDistanceThresholdMM)
    {
      Vector3 pos = minjerk(myStart[i], myTarget[i], t);
      myHexapod.leg(i).setPosition(pos);      
    }
    else 
    {
      Vector3 p0 = myStart[i];
      Vector3 p3 = myTarget[i];

      // P1: Move out and slightly up immediately
      Vector3 p1 = p0;
      p1.x = p3.x;         // Reach target X/Y halfway through the curve
      p1.y = p3.y;
      p1.z = p0.z + 20.0f; // Small lift to avoid dragging

      // P2: Already at the target X/Y, hovering directly above P3
      Vector3 p2 = p3;
      p2.z = p3.z + 30.0f; // Hover 30mm above the final spot

      Vector3 pos = bezier3(p0, p1, p2, p3, t);
      myHexapod.leg(i).setPosition(pos);
    }
  }

  myHexapod.servoBus().syncWrite();
}

// ----------------------------------------------------------------------------------------
void StandUpGait::cycleComplete()
{
  myCanChange = true;
}


// ---- PosingGait ------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
PosingGait::PosingGait(Hexapod& hexapod)
: Gait(hexapod, "Posing", GaitCategory::ePosture) 
{}

// ----------------------------------------------------------------------------------------
void PosingGait::begin()
{
  myHexapod.servoBus().setMaxTorque();
}

// ----------------------------------------------------------------------------------------
void PosingGait::update(float, const GaitEngine::MotionCmd& cmd)
{
  const GaitEngine::GaitParams& params = myHexapod.gaitParams();

  for (std::size_t i = 0; i < cNumLegs; i++)
  {
    Vector3 pos(GaitEngine::cNeutralPosition[i].x,
                GaitEngine::cNeutralPosition[i].y,
                params.groundClearance);

    // bodyPos (rotation and translation) is clamped by GaitEngine. 
    //   cMinPoseTranslation, cMaxPoseTranslation
    //   cMinPoseRotation, cMaxPoseRotation
    // Therefore, the values are not constrained here

    // Apply Rotation (Order: Roll -> Pitch -> Yaw)
    // We rotate the leg in the opposite direction of the desired body tilt.
    // rotateXYZ() applies rotateAroundX/Y/Z in that order.
    pos = rotateXYZ(pos, -cmd.pose.rotation);

    // Apply Translation
    // Subtract the body translation from the leg position
    pos -= cmd.pose.translation;
    
    myHexapod.leg(i).setPosition(pos);
  }

  myHexapod.servoBus().syncWrite();
}


// ---- LevelGait -------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
LevelGait::LevelGait(Hexapod& hexapod)
: Gait(hexapod, "Level", GaitCategory::eBalance) 
{}

// ----------------------------------------------------------------------------------------
void LevelGait::begin() 
{
  // Start from "no correction" so we ease into the current tilt rather than snapping to it
  myFilteredTilt = Vector3(0, 0, 0);

  myHexapod.servoBus().setMaxTorque();
}

// ----------------------------------------------------------------------------------------
void LevelGait::update(float, const GaitEngine::MotionCmd&) 
{
  // Hexapod's internal peripheralsTask calls imu().update() at a fixed rate, so this
  // just reads the latest estimate. This is a cross-task read with no synchronization:
  // formally a data race against peripheralsTask's writes. Accepted deliberately here -
  // aligned 32-bit float read/write is effectively atomic in practice on this platform,
  // and MPU6050_light's getAngleX()/Y() are plain member-variable accessors with no
  // multi-step computation that could be caught mid-update. Revisit if MPU6050_light
  // ever changes, or if this is ported to a platform without that atomicity guarantee.
  float roll  = -myHexapod.imu().getAngleX() * deg2rad;
  float pitch = -myHexapod.imu().getAngleY() * deg2rad;

  // Yaw is deliberately left out: heading drift isn't a "level" problem and correcting it
  // would make the robot twist in place.
  Vector3 rawTilt(roll, pitch, 0.0f);

  // Low-pass filter the reading so servo motion stays smooth even if the IMU is noisy
  // or the robot is vibrating (e.g. from a nearby leg shifting weight).
  myFilteredTilt = lerp(myFilteredTilt, rawTilt, cFilterAlpha);

  // Safety clamp - avoid extreme corrections e.g. if the robot is picked up or on its side
  Vector3 correction = myFilteredTilt;
  correction.x = std::clamp(correction.x, -cMaxCorrection, cMaxCorrection);
  correction.y = std::clamp(correction.y, -cMaxCorrection, cMaxCorrection);

  const GaitEngine::GaitParams& params = myHexapod.gaitParams();

  for (std::size_t i = 0; i < cNumLegs; i++)
  {
    Vector3 pos(GaitEngine::cNeutralPosition[i].x,
                GaitEngine::cNeutralPosition[i].y,
                params.groundClearance);

    // Counter-rotate each leg's attachment point opposite to the measured tilt.
    // This is the same trick PosingGait uses for transmitter-commanded rotation,
    // just fed by the IMU instead of the pilot.
    pos = rotateXYZ(pos, -correction);

    myHexapod.leg(i).setPosition(pos);
  }

  myHexapod.servoBus().syncWrite();
}


// ---- GenericWalkingGait ----------------------------------------------------------------

namespace
{
  // Leg offsets live on a circle, so "how far from a to b" has two answers.
  float wrapPhase(float p)
  {
    p = fmodf(p, 1.0f);
    return (p < 0.0f) ? p + 1.0f : p;
  }

  float shortestPhaseArc(float from, float to)
  {
    const float d = wrapPhase(to - from);
    return (d > 0.5f) ? d - 1.0f : d;
  }

  // Signed distance from the body centre to the edge of the polygon spanned by the planted
  // feet: positive inside, negative outside. Gift wrapping, because this is never called with
  // more than five points and it needs no allocation and no sort.
  float supportMargin(const Vector2* pts, std::size_t n)
  {
    if (n < 3)
      return -1.0e9f;

    std::size_t hull[cNumLegs];
    std::size_t count = 0;

    std::size_t start = 0;
    for (std::size_t i = 1; i < n; i++)
    {
      if (pts[i].x < pts[start].x ||
          (floatEquals(pts[i].x, pts[start].x) && pts[i].y < pts[start].y))
        start = i;
    }

    std::size_t current = start;
    do
    {
      hull[count++] = current;

      std::size_t next = (current + 1) % n;
      for (std::size_t i = 0; i < n; i++)
      {
        // a point to the right of current->next becomes the new candidate, which walks the
        // hull counter-clockwise
        const float cross = (pts[next].x - pts[current].x) * (pts[i].y - pts[current].y)
                          - (pts[next].y - pts[current].y) * (pts[i].x - pts[current].x);
        if (cross < 0.0f)
          next = i;
      }
      current = next;
    }
    while (current != start && count < n);

    if (count < 3)
      return -1.0e9f;

    float best   = 1.0e9f;
    bool  inside = true;

    for (std::size_t i = 0; i < count; i++)
    {
      const Vector2& a = pts[hull[i]];
      const Vector2& b = pts[hull[(i + 1) % count]];

      const float ex  = b.x - a.x;
      const float ey  = b.y - a.y;
      const float len = sqrtf(ex * ex + ey * ey);

      if (len < 1.0e-6f)
        continue;

      // cross of the edge with the vector to the origin; the hull runs counter-clockwise, so
      // a negative value puts the body centre outside this edge
      const float cross = ex * (0.0f - a.y) - ey * (0.0f - a.x);

      if (cross < 0.0f)
        inside = false;

      best = std::min(best, fabsf(cross) / len);
    }

    return inside ? best : -best;
  }
}

// ----------------------------------------------------------------------------------------
GenericWalkingGait::GenericWalkingGait(Hexapod& hexapod, 
                                       const String& gaitName, const GaitConfig& config)
: Gait(hexapod, gaitName, GaitCategory::eLocomotion),
  myGaitConfig(config),
  myFromConfig(config),
  myLiveConfig(config)
{}

// ----------------------------------------------------------------------------------------
void GenericWalkingGait::begin()
{
  myHexapod.servoBus().setMaxTorque();

  myCurrentPhase = 0.0f;
  myLastPhase    = 0.0f;
  myPhaseSeeded  = false;

  // Not blending: this gait walks its own pattern from the first step.
  myFromConfig = myGaitConfig;
  myLiveConfig = myGaitConfig;
  myMorphAlpha = 1.0f;

  for (std::size_t i = 0; i < cNumLegs; i++)
    myOffsetDelta[i] = 0.0f;
}

// ----------------------------------------------------------------------------------------
bool GenericWalkingGait::beginFrom(const GaitConfig& previous, const GaitEngine::MotionCmd& cmd,
                                   float phase)
{
  myHexapod.servoBus().setMaxTorque();

  myFromConfig = previous;
  myLiveConfig = previous; // alpha 0 is the outgoing pattern exactly, so no foot moves here
  myMorphAlpha = 0.0f;

  myPhaseSeeded = false; // the engine's phase carries on; there is no delta to act on yet

  chooseOffsetPath(cmd, phase);

  return true;
}

// ----------------------------------------------------------------------------------------
void GenericWalkingGait::chooseOffsetPath(const GaitEngine::MotionCmd& cmd, float phase)
{
  std::size_t tied[cMaxTiedLegs];
  std::size_t numTied = 0;

  for (std::size_t i = 0; i < cNumLegs; i++)
  {
    myOffsetDelta[i] = shortestPhaseArc(myFromConfig.legOffsets[i], myGaitConfig.legOffsets[i]);

    if (numTied < cMaxTiedLegs && fabsf(fabsf(myOffsetDelta[i]) - 0.5f) < cPhaseTie)
      tied[numTied++] = i;
  }

  if (numTied == 0)
    return;

  // Both directions are the same distance, so the choice is made on which one keeps the most
  // ground under the robot the whole way across.
  float    bestMargin = -1.0e9f;
  unsigned bestMask   = 0;

  for (unsigned mask = 0; mask < (1u << numTied); mask++)
  {
    for (std::size_t k = 0; k < numTied; k++)
    {
      const float d = fabsf(myOffsetDelta[tied[k]]);
      myOffsetDelta[tied[k]] = (((mask >> k) & 1u) != 0u) ? -d : d;
    }

    const float margin = worstMarginOnPath(cmd, phase);

    if (margin > bestMargin)
    {
      bestMargin = margin;
      bestMask   = mask;
    }
  }

  for (std::size_t k = 0; k < numTied; k++)
  {
    const float d = fabsf(myOffsetDelta[tied[k]]);
    myOffsetDelta[tied[k]] = (((bestMask >> k) & 1u) != 0u) ? -d : d;
  }
}

// ----------------------------------------------------------------------------------------
float GenericWalkingGait::worstMarginOnPath(const GaitEngine::MotionCmd& cmd,
                                            float startPhase) const
{
  const GaitEngine::GaitParams& params = myHexapod.gaitParams();

  float worst = 1.0e9f;

  for (std::size_t s = 0; s <= cPathSteps; s++)
  {
    // alpha and the engine phase move together: the blend takes cMorphCycles cycles, so by
    // the time alpha reaches 1 the engine has advanced that many turns.
    const float alpha = float(s) / float(cPathSteps);
    const float phase = wrapPhase(startPhase + alpha * cMorphCycles);
    const float swing = lerpf(myFromConfig.swingDuration, myGaitConfig.swingDuration, alpha);

    Vector2 support[cNumLegs];
    std::size_t n = 0;

    for (std::size_t i = 0; i < cNumLegs; i++)
    {
      const float offset   = myFromConfig.legOffsets[i] + alpha * myOffsetDelta[i];
      const float legPhase = wrapPhase(phase + offset);

      if (legPhase < swing) // in the air, carrying nothing
        continue;

      const Vector3 foot = footPosition(i, legPhase, swing, cmd, params);
      support[n++] = Vector2(foot.x, foot.y);
    }

    worst = std::min(worst, supportMargin(support, n));
  }

  return worst;
}

// ----------------------------------------------------------------------------------------
void GenericWalkingGait::blendConfig()
{
  if (myMorphAlpha >= 1.0f)
  {
    // Snapped rather than left at the end of the interpolation, so the settled gait walks its
    // own table exactly and not a value a hair away from it.
    myLiveConfig = myGaitConfig;
    return;
  }

  myLiveConfig.swingDuration = lerpf(myFromConfig.swingDuration,
                                     myGaitConfig.swingDuration, myMorphAlpha);

  for (std::size_t i = 0; i < cNumLegs; i++)
  {
    myLiveConfig.legOffsets[i] = wrapPhase(myFromConfig.legOffsets[i]
                                           + myMorphAlpha * myOffsetDelta[i]);
  }

  // Mid-blend the pattern is neither gait, and the engine picks one cycle time for the whole
  // robot, so the tighter of the two limits governs: a gait with a short swing window is the
  // one that cannot follow a fast cycle.
  myLiveConfig.speedLimit = std::min(myFromConfig.speedLimit, myGaitConfig.speedLimit);
}

// ----------------------------------------------------------------------------------------
Vector3 GenericWalkingGait::footPosition(std::size_t leg, float legPhase, float swing,
                                         const GaitEngine::MotionCmd& cmd,
                                         const GaitEngine::GaitParams& params) const
{
  // How hard the robot is being asked to move, 0 at rest and 1 at full command. The stride in
  // force and the operator's own demand are both consulted, and the larger wins: the two do not
  // fall to zero at the same moment. Reversing at speed sends the stride through zero a filter
  // time-constant after the stick passed centre, and taking the stride alone would read that as
  // a standing robot and drop all six feet to the floor for the length of the crossing.
  const float demand = max(cmd.demand,
                           max((cmd.linear.length() / params.stepLength),
                               normalize(abs(cmd.yaw), 0.0f, GaitEngine::cMaxYaw)));

  // Demand at which the minimum lift is fully applied. Below this the floor scales down, so a
  // stationary robot puts every foot on the ground.
  const float cLiftRampDemand = 0.2f;

  // The minimum lift ramps in with demand rather than applying flat. A flat floor would keep a
  // leg in the air whenever one is mid-swing at the parked phase: the demand decays to nothing
  // but the clamp holds the lift at cMinStepHeight.
  const float liftRamp = std::clamp(demand / cLiftRampDemand, 0.0f, 1.0f);

  float currentStepHeight = std::clamp(params.stepHeight * demand,
                                       GaitEngine::cMinStepHeight * liftRamp,
                                       params.stepHeight);

  // Safety buffer (mm): the swing aims this far above the ground and the stance settles the
  // last of it, so the foot is never driven into the floor. Faded out by liftRamp along with
  // the lift itself, so a stationary robot rests every foot at groundClearance rather than a
  // few millimetres shy of it. Tune the 5 mm per gait if needed.
  const float cTouchdownOffset = 5.0f * liftRamp;

  // Fraction of the stance phase spent lowering the foot from cTouchdownOffset to the ground.
  const float cTouchdownSettleFraction = 0.1f;

  // Define the step endpoints (Neutral +/- Stride)
  //   SwingEnd (Forward) is where we land; SwingStart (Backward) is where we lift
  //   Swing Start = Back position (Stance End)
  //   Swing End   = Front position (Stance Start)

  Vector3 neutralPosition(GaitEngine::cNeutralPosition[leg].x,
                          GaitEngine::cNeutralPosition[leg].y,
                          params.groundClearance);
  Vector3 swingStart = rotateAroundZ(neutralPosition, -cmd.yaw / 2.0f) - (cmd.linear / 2.0f);
  Vector3 swingEnd   = rotateAroundZ(neutralPosition,  cmd.yaw / 2.0f) + (cmd.linear / 2.0f);

  Vector3 pos;

  if (legPhase < swing)
  {
    // Map legPhase (0 -> swing) to 0.0 -> 1.0
    float t = legPhase / swing;

    // Apply Sinusoidal Easing (Smooth Start/Stop)
    t = 0.5f * (1.0f - cosf(t * cPI)); // Sine ease-in-out

    // Calculate the multiplier to ensure the curve actually peaks at currentStepHeight
    // For a cubic Bezier, 1.333f is the magic number to hit the target at t=0.5
    float controlHeight = (currentStepHeight * 1.333f);

    Vector3 p0 = swingStart;
    Vector3 p3 = swingEnd;
    p3.z += cTouchdownOffset; // Target a point slightly above the floor

    // p1: Lift vertically from start
    Vector3 p1 = p0; 
    p1.z += controlHeight;

    // p2: Pulls the curve up at the end (keeping Z flush with p3 for soft landing)
    Vector3 p2 = p3; 
    p2.z += controlHeight;

    pos = bezier3(p0, p1, p2, p3, t);
  }
  else
  {
    // Map legPhase (swing -> 1.0) to 0.0 -> 1.0
    float tRaw = (legPhase - swing) / (1.0f - swing);

    // Linear, deliberately, where the swing is eased. Every stance foot is on the ground, so
    // they must all travel at the same speed. Easing makes that speed depend on how far
    // through its own stance a leg is, so legs that touched down at different times pull
    // against each other through the ground and scrub.
    //
    // Tripod hides this: its three stance legs always share one phase, so the ease cancels
    // out. Ripple's five stance legs sit at five different points on the curve, and the
    // fastest and slowest differ by 95% of full speed.
    const float t = tRaw;

    // Linear interpolation on the ground (Front to Back). 
    // However, instead of a straight line, we move from End to Start by reversing the command.
    // This ensures the foot follows the same arc it took in the air and is more natural

    float currentYaw = lerpf(cmd.yaw / 2.0f, -cmd.yaw / 2.0f, t);
    Vector2 currentLinear = lerp(cmd.linear / 2.0f, -cmd.linear / 2.0f, t);

    pos = rotateAroundZ(GaitEngine::cNeutralPosition[leg], currentYaw);
    pos.x += currentLinear.x;
    pos.y += currentLinear.y;

    // If we are in the first 10% of the stance, smoothly lower from Offset to Ground.
    // Deliberately keyed off tRaw (not the eased t above) - this is an independent,
    // short fixed-duration touchdown-settling window, not tied to the horizontal
    // ease curve.
    if (tRaw < cTouchdownSettleFraction)
    {
      pos.z = lerpf(params.groundClearance + cTouchdownOffset, params.groundClearance,
                    tRaw / cTouchdownSettleFraction);
    }
    else 
    {
      pos.z = params.groundClearance;
    }
  }

  // Bank into the turn. Applied as a vertical shear rather than a rotation of the body: a
  // rotation would swing a planted foot sideways, while shearing z against the foot's own
  // lateral position tilts the body about its centre line and leaves every footprint exactly
  // where it was. Feet on the inside of the turn end up nearer the body, which sets that side
  // of the body down.
  //
  // Scaled by the signed forward component, so the robot does not bank while turning on the
  // spot and banks the other way when reversing into a turn.
  const float forward  = std::clamp(cmd.linear.x / params.stepLength, -1.0f, 1.0f);
  const float turnRate = std::clamp(cmd.yaw / GaitEngine::cMaxYaw, -1.0f, 1.0f);
  const float lean     = cMaxLeanTilt * forward * turnRate;

  // The bank draws on the same vertical envelope as the ground clearance, and the RY trim is
  // already spending it. Bounded by what is left of that envelope rather than by a figure of
  // its own, so a bank that would ask a leg to stand higher than cMinGroundClearance or reach
  // further than cMaxGroundClearance flattens off on that side instead. Held at full up-trim
  // there is no headroom left at all, and the bank becomes the outer side dropping alone -
  // still a roll, just about the inner feet rather than the centre line.
  // cTouchdownOffset is already spent above the clearance by a foot about to land, so it comes
  // out of the headroom too. Floored at zero because the trim can leave none at all, in which
  // case the bank is whatever the outer side can still give.
  const float headroom = std::max(0.0f, GaitEngine::cMinGroundClearance
                                        - params.groundClearance - cTouchdownOffset);
  const float depth    = std::max(0.0f, params.groundClearance - GaitEngine::cMaxGroundClearance);

  pos.z += std::clamp(lean * pos.y, -depth, headroom);

  return pos;
}

// ----------------------------------------------------------------------------------------
void GenericWalkingGait::update(float phase, const GaitEngine::MotionCmd& cmd)
{
  myCurrentPhase = phase; // Store for canChange() check  

  const GaitEngine::GaitParams& params = myHexapod.gaitParams();

  if (myMorphAlpha < 1.0f)
  {
    float dPhase = 0.0f;

    if (myPhaseSeeded == true)
    {
      dPhase = phase - myLastPhase;

      if (dPhase < 0.0f) // the engine wrapped 1.0 -> 0.0
        dPhase += 1.0f;
    }

    myMorphAlpha = std::min(myMorphAlpha + dPhase / cMorphCycles, 1.0f);
    blendConfig();
  }

  myLastPhase   = phase;
  myPhaseSeeded = true;

  for (std::size_t i = 0; i < cNumLegs; i++)
  {
    const float legPhase = wrapPhase(phase + myLiveConfig.legOffsets[i]);

    myHexapod.leg(i).setPosition(footPosition(i, legPhase, myLiveConfig.swingDuration,
                                              cmd, params));
  }

  myHexapod.servoBus().syncWrite();
}

// ----------------------------------------------------------------------------------------
bool GenericWalkingGait::canChange() const 
{
  // Parking, posing and levelling wait for the cycle boundary. A change to another *walking*
  // gait does not come through here at all: the engine recognises it as a morph and hands
  // this gait's live pattern to the incoming one.
  return floatEquals(myCurrentPhase, 0.0f);   
}
