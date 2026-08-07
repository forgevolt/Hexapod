#include "IndicatorLeds.h"

#include <math.h>

// ---- Timing & effect parameters --------------------------------------------------------

constexpr uint32_t cPulseIntervalMs    = 5;     // Frame interval for pulse effect (ms)
constexpr uint32_t cSweepIntervalMs    = 10;    // Frame interval for sweep effects (ms)

constexpr float    cPulseSpeed         = 0.1f;  // Phase increment per frame (lower = slower)
constexpr uint8_t  cPulseMinBrightness = 20;    // Dimmest point of the pulse (0-255)

constexpr float    cSweepSpeed         = 1.3f;  // Pixels advanced per frame (lower = slower)
constexpr uint8_t  cSweepTailLength    = 7;     // Number of LEDs in the fading tail
constexpr CRGB     cSweepColor         = CRGB(220, 60, 0); // Warm orange-red comet colour


// ---- IndicatorLeds ---------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
IndicatorLeds::IndicatorLeds()
  : myLastFrameTime(0),
    myRequestedEffect(Effect::eNone),
    myActiveEffect(Effect::eNone),
    myPulsePhase(3.0f * M_PI / 2.0f),
    mySweepPos(0.0f)
{}

// ----------------------------------------------------------------------------------------
void IndicatorLeds::setEffect(Effect effect)
{
  myRequestedEffect.store(effect);
}

// ----------------------------------------------------------------------------------------
void IndicatorLeds::update()
{
  const Effect requested = myRequestedEffect.load();
  if (requested != myActiveEffect)
  {
    myActiveEffect = requested;

    // Reset all per-effect state so every effect starts from the beginning of its cycle.
    myPulsePhase = 3.0f * M_PI / 2.0f; // pulse starts at minimum brightness
    mySweepPos   = 0.0f;

    // Clear the strip so there is no leftover frame from the previous effect.
    clearAll();
    FastLED.show();
  }

  switch (myActiveEffect)
  {
    case Effect::ePulseRed:          runPulseRed();          break;
    case Effect::ePulseOrange:       runPulseOrange();       break;
    case Effect::eSweepForward:      runSweepForward();      break;
    case Effect::eSweepBackward:     runSweepBackward();     break;
    case Effect::eSweepMeetInMiddle: runSweepMeetInMiddle(); break;
    default: break; // eNone: do nothing
  }
}


// ---- Effect implementations ------------------------------------------------------------

// ----------------------------------------------------------------------------------------
void IndicatorLeds::runPulseRed()
{
  if (!isFrameDue(cPulseIntervalMs))
    return;

  // Advance the phase and keep it in [0, 2π)
  myPulsePhase += cPulseSpeed;
  if (myPulsePhase >= 2.0f * M_PI)
    myPulsePhase -= 2.0f * M_PI;

  // Map sin() from [-1, 1] to [cPulseMinBrightness, 255]
  float   sineValue  = (sinf(myPulsePhase) + 1.0f) * 0.5f; // 0.0 – 1.0
  uint8_t brightness = cPulseMinBrightness
                     + static_cast<uint8_t>(sineValue * (255 - cPulseMinBrightness));

  fill_solid(myLeds, cNumLeds, CRGB(brightness, 0, 0));
  FastLED.show();
}

// ----------------------------------------------------------------------------------------
void IndicatorLeds::runPulseOrange()
{
  if (!isFrameDue(cPulseIntervalMs))
    return;

  // Advance the phase and keep it in [0, 2π)
  myPulsePhase += cPulseSpeed;
  if (myPulsePhase >= 2.0f * M_PI)
    myPulsePhase -= 2.0f * M_PI;

  // Map sin() from [-1, 1] to [cPulseMinBrightness, 255] and scale the sweep colour.
  float   sineValue  = (sinf(myPulsePhase) + 1.0f) * 0.5f; // 0.0 – 1.0
  uint8_t brightness = cPulseMinBrightness
                     + static_cast<uint8_t>(sineValue * (255 - cPulseMinBrightness));

  // Scale cSweepColor by the brightness factor so hue is preserved across the pulse.
  float scale = static_cast<float>(brightness) / 255.0f;
  CRGB color(
    static_cast<uint8_t>(cSweepColor.r * scale),
    static_cast<uint8_t>(cSweepColor.g * scale),
    static_cast<uint8_t>(cSweepColor.b * scale)
  );

  fill_solid(myLeds, cNumLeds, color);
  FastLED.show();
}

// ----------------------------------------------------------------------------------------
void IndicatorLeds::runSweepForward()
{
  if (!isFrameDue(cSweepIntervalMs))
    return;

  mySweepPos += cSweepSpeed;

  // Wrap: when the head (plus tail) has fully passed the last LED, restart from 0.
  if (mySweepPos > static_cast<float>(cNumLeds - 1) + cSweepTailLength)
    mySweepPos = 0.0f;

  clearAll();
  drawComet(mySweepPos, cSweepColor, cSweepTailLength, /*forward=*/true);
  FastLED.show();
}

// ----------------------------------------------------------------------------------------
void IndicatorLeds::runSweepBackward()
{
  if (!isFrameDue(cSweepIntervalMs))
    return;

  mySweepPos += cSweepSpeed;

  // Wrap: same range as forward; position is mirrored before drawing.
  if (mySweepPos > static_cast<float>(cNumLeds - 1) + cSweepTailLength)
    mySweepPos = 0.0f;

  // Mirror the position so the comet travels from index (N-1) toward 0.
  float mirroredPos = static_cast<float>(cNumLeds - 1) - mySweepPos;

  clearAll();
  drawComet(mirroredPos, cSweepColor, cSweepTailLength, /*forward=*/false);
  FastLED.show();
}

// ----------------------------------------------------------------------------------------
void IndicatorLeds::runSweepMeetInMiddle()
{
  if (!isFrameDue(cSweepIntervalMs * 2))
    return;

  // Left comet travels forward  from index 0   → centre
  // Right comet travels backward from index N-1 → centre
  // Both share mySweepPos as the distance travelled from their respective ends.
  constexpr float cMidPoint = static_cast<float>(cNumLeds) / 2.0f - 0.5f;

  // Once the heads meet at the midpoint they park there while the cycle runs on for a few
  // more frames, shortening the tails so the trails retract into the centre. drawComet()
  // divides by tailLength, so the tail must shorten to 1 and never to 0.
  constexpr float cDrainFrames = static_cast<float>(cSweepTailLength - 1);
  constexpr float cCycleEnd    = cMidPoint + cDrainFrames;

  mySweepPos += cSweepSpeed;

  if (mySweepPos >= cCycleEnd)
    mySweepPos -= cCycleEnd;   // keep the fractional overshoot: no stutter at the restart

  const float   overshoot  = (mySweepPos > cMidPoint) ? (mySweepPos - cMidPoint) : 0.0f;
  const float   headTravel = mySweepPos - overshoot;   // heads stop at the midpoint
  const uint8_t tail       = static_cast<uint8_t>(cSweepTailLength - overshoot);

  const float leftPos  = headTravel;
  const float rightPos = static_cast<float>(cNumLeds - 1) - headTravel;

  clearAll();
  drawComet(leftPos,  cSweepColor, tail, /*forward=*/true);
  drawComet(rightPos, cSweepColor, tail, /*forward=*/false);
  FastLED.show();
}


// ---- Private helpers -------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
bool IndicatorLeds::isFrameDue(uint32_t intervalMs)
{
  uint32_t now = millis();
  if (now - myLastFrameTime < intervalMs)
    return false;

  myLastFrameTime = now;
  return true;
}

// ----------------------------------------------------------------------------------------
void IndicatorLeds::drawComet(float position, CRGB color, uint8_t tailLength, bool forward)
{
  // tailLength is the divisor of the fade factor below, so 0 would produce 0/0 = NaN and an
  // undefined uint8_t cast. The sweep effects vary the tail at runtime, so fail visibly
  // (draw nothing) rather than corrupt the strip.
  if (tailLength == 0)
    return;

  // Head pixel (fractional position — round to nearest integer index).
  int headIdx = static_cast<int>(roundf(position));

  // Draw head + tail. The tail extends behind the direction of travel:
  //   forward = true  → tail at lower  indices (headIdx-1, headIdx-2 …)
  //   forward = false → tail at higher indices (headIdx+1, headIdx+2 …)
  for (uint8_t t = 0; t <= tailLength; t++)
  {
    int idx = forward ? (headIdx - static_cast<int>(t))
                      : (headIdx + static_cast<int>(t));

    if (idx < 0 || idx >= cNumLeds)
      continue;

    // Quadratic fade: head is full brightness, tail dims quickly.
    float fadeFactor = static_cast<float>(tailLength - t) / static_cast<float>(tailLength);
    fadeFactor = fadeFactor * fadeFactor;

    myLeds[idx] = CRGB(
      static_cast<uint8_t>(color.r * fadeFactor),
      static_cast<uint8_t>(color.g * fadeFactor),
      static_cast<uint8_t>(color.b * fadeFactor)
    );
  }
}

// ----------------------------------------------------------------------------------------
void IndicatorLeds::clearAll()
{
  fill_solid(myLeds, cNumLeds, CRGB::Black);
}
