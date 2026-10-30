#include "StatusDisplay.h"

#include <cmath>   // sqrtf, tanf, cosf, sinf, floorf, fabsf
#include <cstring> // strlen, memcpy, memset

// ---- StatusDisplay ---------------------------------------------------------------------

namespace
{

// The 8-pixel text font used for fault messages. Set once in begin() and never changed,
// so getStrWidth() and drawStr() always measure and draw with the same metrics.
const uint8_t* const cTextFont = u8g2_font_helvR08_tr;

} // namespace


// ---- Constructor -----------------------------------------------------------------------

// ----------------------------------------------------------------------------------------
StatusDisplay::StatusDisplay()
  : myDisplay(U8G2_R0, U8X8_PIN_NONE) // U8G2_R0 = no rotation; U8X8_PIN_NONE = no reset pin
{}


// ---- Public: lifecycle -----------------------------------------------------------------

// ----------------------------------------------------------------------------------------
bool StatusDisplay::begin(uint8_t fps)
{
  // fps == 0 would divide by zero, which the core traps rather than turning into a value.
  // One frame per second is the slowest rate this can express.
  if (fps == 0)
    fps = 1;

  myFrameInterval = 1000 / fps;

  // Set before begin(): U8X8_MSG_BYTE_INIT only applies the display's own default while
  // bus_clock is still zero.
  //
  // U8g2 would otherwise use the SH1106 default of 400 kHz and re-assert it before every
  // transfer. A full sendBuffer() pushes 1024 bytes, so this sets the per-frame cost.
  // U8g2 never restores the previous clock, and any device sharing this bus that is rated
  // for less must pull the clock back down before its own transfers.
  myDisplay.setBusClock(cDisplayBusClockHz);

  const bool displayIsUp = myDisplay.begin();

  myDisplay.setFont(cTextFont);

  // Center the gaze, and start with the eyes shut so the first frames tween them open.
  myPosXTarget = constraintX() / 2;
  myPosYTarget = constraintY() / 2;
  myPosX       = myPosXTarget;
  myPosY       = myPosYTarget;
  myHeightL    = cClosedHeight;
  myHeightR    = cClosedHeight;

  myDisplay.clearBuffer();
  myDisplay.sendBuffer();
  myScreenBlank = true;

  const unsigned long now = millis();
  myLastFrameTime = now;
  myNextBlinkTime = now;
  myNextIdleTime  = now;

  return displayIsUp;
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::update()
{
  if (isFrameDue() == false)
    return;

  if (myHasError == true &&
      static_cast<long>(millis() - myErrorRaisedAt) >= static_cast<long>(cErrorTimeoutMs))
    clearError();

  // Hiding is applied here rather than in setVisible(), so the I2C transfer that blanks the
  // screen happens on the task that owns the bus, and only once per hide.
  if (myVisible == false)
  {
    if (myScreenBlank == false)
    {
      myDisplay.clearBuffer();
      myDisplay.sendBuffer();
      myScreenBlank = true;
    }
    return;
  }

  myDisplay.clearBuffer();

  if (myHasError == true)
    drawError();
  else
    drawEyes();

  myDisplay.sendBuffer();
  myScreenBlank = false;
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::setVisible(bool visible)
{
  // A fault owns the screen while it is up, so a visibility change made during one is
  // remembered rather than applied, and takes effect when the fault clears.
  if (myHasError == true)
  {
    myVisibleAfterError = visible;
    return;
  }

  // Only the flag: update() blanks the screen on its next frame. This may be called from a
  // task that does not own the I2C bus, so it must not touch the display itself.
  myVisible = visible;
}


// ---- Public: eye configuration ---------------------------------------------------------

// ----------------------------------------------------------------------------------------
void StatusDisplay::setWidth(uint8_t leftEye, uint8_t rightEye)
{
  myWidthTargetL = leftEye;
  myWidthTargetR = rightEye;
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::setHeight(uint8_t leftEye, uint8_t rightEye)
{
  myHeightTargetL = leftEye;
  myHeightTargetR = rightEye;
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::setSpaceBetween(int space)
{
  mySpaceTarget = space;
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::setMood(Mood mood)
{
  // Sleepy's swap and doze cycles start over each time the mood is entered, so it always
  // opens the same way.
  if (mood == Mood::eSleepy && myMood != Mood::eSleepy)
    mySleepyStart = millis();

  myMood = mood;
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::setAutoBlinker(bool active, int intervalS, int variationS)
{
  myAutoBlink       = active;
  myBlinkIntervalS  = (intervalS  > 0) ? intervalS  : 0;
  myBlinkVariationS = (variationS > 0) ? variationS : 0;

  if (myAutoBlink == false)
    myBlinkFramesLeft = 0; // abandon a blink in progress, so the eyes reopen

  myNextBlinkTime = millis();
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::setIdleMode(bool active, int intervalS, int variationS)
{
  myIdle           = active;
  myIdleIntervalS  = (intervalS  > 0) ? intervalS  : 0;
  myIdleVariationS = (variationS > 0) ? variationS : 0;
  myNextIdleTime   = millis();
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::setCuriosity(bool active)
{
  myCuriosity = active;
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::setLookDirection(float x, float y)
{
  if (x < -1.0f) x = -1.0f;
  if (x >  1.0f) x =  1.0f;
  if (y < -1.0f) y = -1.0f;
  if (y >  1.0f) y =  1.0f;

  myLookX    = x;
  myLookY    = y;
  myLookHeld = true;
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::setPerspective(bool active)
{
  myPerspective = active;
}


// ---- Public: fault reporting -----------------------------------------------------------

// ----------------------------------------------------------------------------------------
void StatusDisplay::showError(const char* title, const char* detail)
{
  snprintf(myErrorTitle,  sizeof(myErrorTitle),  "%s", (title  != nullptr) ? title  : "");
  snprintf(myErrorDetail, sizeof(myErrorDetail), "%s", (detail != nullptr) ? detail : "");

  // Only the first of a run of faults records the visibility to go back to.
  if (myHasError == false)
    myVisibleAfterError = myVisible;

  myVisible       = true;
  myHasError      = true;
  myErrorRaisedAt = millis();
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::clearError()
{
  if (myHasError == false)
    return;

  myHasError = false;

  // Restores the visibility asked for while the fault was up, blanking the screen if that
  // is what the caller wanted.
  setVisible(myVisibleAfterError);
}


// ---- Private: frame timing -------------------------------------------------------------

// ----------------------------------------------------------------------------------------
bool StatusDisplay::isFrameDue()
{
  const unsigned long now = millis();

  if (now - myLastFrameTime < static_cast<unsigned long>(myFrameInterval))
    return false;

  myLastFrameTime = now;
  return true;
}


// ---- Private: geometry -----------------------------------------------------------------

// ----------------------------------------------------------------------------------------
int StatusDisplay::tween(int value, int target)
{
  const int difference = target - value;

  if (difference == 0)
    return value;

  int step = difference / cTweenDivisor;

  // A floor of one pixel in the right direction, so the tween converges instead of
  // stalling once the remaining distance is smaller than the divisor.
  if (step == 0)
    step = (difference > 0) ? 1 : -1;

  return value + step;
}

// ----------------------------------------------------------------------------------------
int StatusDisplay::constraintX() const
{
  const int room = cScreenWidth - myWidthL - mySpace - myWidthR;

  return (room > 0) ? room : 0;
}

// ----------------------------------------------------------------------------------------
int StatusDisplay::constraintY() const
{
  // The target heights are used rather than the current ones, so blinking does not shrink
  // the travel range and pull the gaze upward mid-animation.
  const int tallest = (myHeightTargetL > myHeightTargetR) ? myHeightTargetL : myHeightTargetR;
  const int room    = cScreenHeight - tallest;

  return (room > 0) ? room : 0;
}


// ---- Private: animation timers ---------------------------------------------------------

// ----------------------------------------------------------------------------------------
void StatusDisplay::applyBlinkTimer()
{
  // A blink in progress runs to completion before the next one can be scheduled.
  if (myBlinkFramesLeft > 0)
  {
    myBlinkFramesLeft--;
    return;
  }

  // Sleepy eyes close on their own schedule - see sleepyPose().
  if (myAutoBlink == false || myMood == Mood::eSleepy)
    return;

  const unsigned long now = millis();

  // Signed difference rather than now < deadline, so the timer survives millis() wrapping.
  if (static_cast<long>(now - myNextBlinkTime) < 0)
    return;

  myBlinkFramesLeft = cBlinkFrames;

  // random() is given a bound of at least 1, so a zero variation needs no special case.
  myNextBlinkTime = now
                  + static_cast<unsigned long>(myBlinkIntervalS) * 1000UL
                  + static_cast<unsigned long>(random(myBlinkVariationS + 1)) * 1000UL;
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::applyIdleTimer()
{
  if (myIdle == false)
    return;

  const unsigned long now = millis();

  // Signed difference rather than now < deadline, so the timer survives millis() wrapping.
  if (static_cast<long>(now - myNextIdleTime) < 0)
    return;

  const int rangeX = constraintX();
  const int rangeY = constraintY();

  myPosXTarget = static_cast<int>(random(rangeX + 1));
  myPosYTarget = static_cast<int>(random(rangeY + 1));

  myNextIdleTime = now
                 + static_cast<unsigned long>(myIdleIntervalS) * 1000UL
                 + static_cast<unsigned long>(random(myIdleVariationS + 1)) * 1000UL;
}


// ---- Private: rendering ----------------------------------------------------------------

// ----------------------------------------------------------------------------------------
void StatusDisplay::drawEyes()
{
  applyBlinkTimer();

  // An explicit gaze wins over idle wandering for the frame it was set on.
  if (myLookHeld == true)
  {
    myPosXTarget = static_cast<int>((myLookX + 1.0f) * 0.5f * static_cast<float>(constraintX()));
    myPosYTarget = static_cast<int>((myLookY + 1.0f) * 0.5f * static_cast<float>(constraintY()));
    myLookHeld   = false;
  }
  else
  {
    applyIdleTimer();
  }

  // Sleepy reshapes the eyes through the same tweens as everything else, so entering and
  // leaving the mood is animated.
  const bool sleepy = (myMood == Mood::eSleepy);
  float doze = 1.0f;

  myKL = 0.0f;
  myKR = 0.0f;

  if (sleepy == true)
    sleepyPose(myKL, myKR, doze);

  const int spaceTarget = (sleepy == true && mySpaceTarget > cSleepySpace) ? cSleepySpace
                                                                          : mySpaceTarget;

  myWidthL = tween(myWidthL, myWidthTargetL);
  myWidthR = tween(myWidthR, myWidthTargetR);
  mySpace  = tween(mySpace,  spaceTarget);
  myPosX   = tween(myPosX,   myPosXTarget);
  myPosY   = tween(myPosY,   myPosYTarget);

  // The widths and the gap tween as well, so a position that was reachable when it was
  // chosen can fall outside the range a few frames later. Clamping here keeps the pair of
  // eyes inside the screen for every intermediate frame, not just the settled ones.
  const int rangeX = constraintX();
  const int rangeY = constraintY();

  if (myPosX > rangeX) myPosX = rangeX;
  if (myPosY > rangeY) myPosY = rangeY;
  if (myPosX < 0)      myPosX = 0;
  if (myPosY < 0)      myPosY = 0;

  // Curiosity: the eye on the side the gaze is heading for grows taller, in proportion to
  // how far into the outer cCuriosityEdge of the travel range the gaze has gone.
  int growthL = 0;
  int growthR = 0;

  if (myCuriosity == true)
  {
    const int range = constraintX();

    if (range > 0)
    {
      const float gaze = static_cast<float>(myPosX) / static_cast<float>(range); // 0 = far left

      if (gaze < cCuriosityEdge)
        growthL = static_cast<int>(cCuriosityGrowth * (1.0f - gaze / cCuriosityEdge));
      else if (gaze > 1.0f - cCuriosityEdge)
        growthR = static_cast<int>(cCuriosityGrowth * (gaze - (1.0f - cCuriosityEdge)) / cCuriosityEdge);
    }
  }

  int heightTargetL = myHeightTargetL + growthL;
  int heightTargetR = myHeightTargetR + growthR;
  int dropTargetL   = 0;
  int dropTargetR   = 0;

  if (sleepy == true)
  {
    const auto closed = [doze](int height, float k) {
      const float open = 1.0f - (cSleepyLidClose + (cSleepySlitClose - cSleepyLidClose) * k);
      const int   h    = static_cast<int>(static_cast<float>(height) * open * doze + 0.5f);
      return (h > cClosedHeight) ? h : cClosedHeight;
    };
    const auto drop = [](int height, float k) {
      return static_cast<int>(static_cast<float>(height)
                              * (cSleepyLidDrop + (cSleepySlitDrop - cSleepyLidDrop) * k) + 0.5f);
    };

    heightTargetL = closed(myHeightTargetL, myKL);
    heightTargetR = closed(myHeightTargetR, myKR);
    dropTargetL   = drop(myHeightTargetL, myKL);
    dropTargetR   = drop(myHeightTargetR, myKR);
  }

  // A blink snaps shut in a single frame and tweens back open. Tweening it shut as well
  // would never get there: the blink lasts cBlinkFrames, and each frame only covers a
  // fraction of the remaining distance, so the eye would reopen while still half height.
  if (myBlinkFramesLeft > 0)
  {
    myHeightL = cClosedHeight;
    myHeightR = cClosedHeight;
  }
  else
  {
    myHeightL = tween(myHeightL, heightTargetL);
    myHeightR = tween(myHeightR, heightTargetR);
  }

  myDropL = tween(myDropL, dropTargetL);
  myDropR = tween(myDropR, dropTargetR);

  // Perspective: the eyes are always drawn where a centred gaze puts them, and the gaze
  // turns the shell instead - so the front view is the flat view, pixel for pixel. The
  // centre is the same integer position a centred setLookDirection() targets.
  int   posX  = myPosX;
  int   posY  = myPosY;
  float gazeX = 0.0f;
  float gazeY = 0.0f;

  if (myPerspective == true)
  {
    const int centreX = rangeX / 2;
    const int centreY = rangeY / 2;

    if (rangeX > 1) gazeX = static_cast<float>(myPosX - centreX) / (static_cast<float>(rangeX) / 2.0f);
    if (rangeY > 1) gazeY = static_cast<float>(myPosY - centreY) / (static_cast<float>(rangeY) / 2.0f);

    gazeX = (gazeX < -1.0f) ? -1.0f : (gazeX > 1.0f) ? 1.0f : gazeX;
    gazeY = (gazeY < -1.0f) ? -1.0f : (gazeY > 1.0f) ? 1.0f : gazeY;

    posX = centreX;
    posY = centreY;
  }

  // Each eye keeps a fixed center line derived from its target height, so blinking and
  // curiosity growth expand and collapse symmetrically instead of dragging the eye down.
  const int centreYL = posY + myHeightTargetL / 2 + myDropL;
  const int centreYR = posY + myHeightTargetR / 2 + myDropR;

  const int leftX  = posX;
  const int rightX = posX + myWidthL + mySpace;

  drawEye(leftX,  centreYL, myWidthL, myHeightL, /*isLeftEye=*/true);
  drawEye(rightX, centreYR, myWidthR, myHeightR, /*isLeftEye=*/false);

  // A centred gaze needs no warp: it would map every pixel onto itself.
  if (myPerspective == true && (gazeX != 0.0f || gazeY != 0.0f))
    warpToShell(gazeX, gazeY);
}

// ----------------------------------------------------------------------------------------
// The sleepy cycle, from the time since the mood was entered. The slit lingers on one eye
// for a few seconds, then crosses to the other in about one; the second harmonic keeps the
// rhythm from looking mechanical. Once per cSleepyDozeMs both eyes droop shut, hold, and the
// tweens jolt them open again.
void StatusDisplay::sleepyPose(float& kLeft, float& kRight, float& doze) const
{
  const unsigned long elapsed = millis() - mySleepyStart;

  const float phase   = TWO_PI * static_cast<float>(elapsed % cSleepySwapMs) / static_cast<float>(cSleepySwapMs);
  float       balance = -1.6f * cosf(phase + 0.35f * sinf(2.3f * phase)); // -1: right eye is the slit
  balance = (balance < -1.0f) ? -1.0f : (balance > 1.0f) ? 1.0f : balance;

  kLeft  = (1.0f + balance) / 2.0f;
  kRight = (1.0f - balance) / 2.0f;

  const unsigned long inCycle = elapsed % cSleepyDozeMs;
  doze = 1.0f;

  if (inCycle >= cSleepyDozeAtMs && inCycle < cSleepyDozeAtMs + cSleepyShutMs)
    doze = 1.0f - static_cast<float>(inCycle - cSleepyDozeAtMs) / static_cast<float>(cSleepyShutMs);
  else if (inCycle >= cSleepyDozeAtMs + cSleepyShutMs && inCycle < cSleepyDozeAtMs + cSleepyShutMs + cSleepyHoldMs)
    doze = 0.0f;
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::drawEye(int x, int centreY, int width, int height, bool isLeftEye)
{
  if (width <= 0 || height <= 0)
    return;

  // Happy draws an arc instead of the eye, and blinks by squashing it flat.
  if (myMood == Mood::eHappy)
  {
    drawHappyArc(x, centreY, width, height, isLeftEye);
    return;
  }

  int y = centreY - height / 2;

  // Clip to the screen. Eyes configured wider or taller than the display, and eyes grown
  // by curiosity while already at an edge, are trimmed instead of drawn outside it.
  if (x < 0) { width  += x; x = 0; }
  if (y < 0) { height += y; y = 0; }

  if (x + width  > cScreenWidth)  width  = cScreenWidth  - x;
  if (y + height > cScreenHeight) height = cScreenHeight - y;

  if (width <= 0 || height <= 0)
    return;

  int radius = cCornerRadius;

  if (myMood == Mood::eSleepy)
    radius = static_cast<int>(static_cast<float>(cCornerRadius) * cSleepyRadiusScale + 0.5f);

  // drawRBox() requires each side to be longer than twice the corner radius, so the limit
  // is (side - 1) / 2 rather than side / 2.
  if (radius > (height - 1) / 2) radius = (height - 1) / 2;
  if (radius > (width  - 1) / 2) radius = (width  - 1) / 2;

  myDisplay.setDrawColor(cMainColor);

  // Sleepy: the slit droops towards the outside of the face.
  const float k    = (isLeftEye == true) ? myKL : myKR;
  const int   tilt = (myMood == Mood::eSleepy)
                   ? static_cast<int>(tanf(cSleepySlitTiltDeg * DEG_TO_RAD) * static_cast<float>(width) * k + 0.5f)
                   : 0;

  if (tilt > 0)
    drawTiltedBox(x, y, width, height, radius, tilt, /*lowLeft=*/isLeftEye);
  else if (radius > 0)
    myDisplay.drawRBox(x, y, width, height, radius);
  else
    myDisplay.drawBox(x, y, width, height);

  drawMoodMask(x, y, width, height, isLeftEye);

  myDisplay.setDrawColor(cMainColor);
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::drawMoodMask(int x, int y, int width, int height, bool isLeftEye)
{
  if (myMood == Mood::eDefault)
    return;

  // Nothing meaningful to mask on an eye that is shut.
  if (height <= cClosedHeight * 2)
    return;

  myDisplay.setDrawColor(cBgColor);

  // The mask reaches a little beyond the eye, so the rasterized edge of a diagonal cannot
  // leave a lit sliver in the rounded corner it is meant to cut away. The overshoot is kept
  // inside the buffer: u8g2 takes unsigned coordinates, and a negative one would wrap to a
  // huge value rather than clip.
  const int topEdge   = (y - cMaskOvershoot > 0) ? y - cMaskOvershoot : 0;
  const int leftEdge  = (x - cMaskOvershoot > 0) ? x - cMaskOvershoot : 0;
  const int farRight  = x + width - 1 + cMaskOvershoot;
  const int rightEdge = (farRight < cScreenWidth - 1) ? farRight : cScreenWidth - 1;

  const int depth = height / cLidDepthDivisor;

  switch (myMood)
  {
    // A lid sloping down toward the outer edge of the face: the corner away from the
    // other eye is cut away.
    case Mood::eTired:
      if (isLeftEye == true)
        myDisplay.drawTriangle(leftEdge, topEdge, rightEdge, topEdge, leftEdge, y + depth);
      else
        myDisplay.drawTriangle(leftEdge, topEdge, rightEdge, topEdge, rightEdge, y + depth);
      break;

    // A brow lowered toward the other eye, reading as concentration: the inner corner is
    // cut away.
    case Mood::eFocused:
      if (isLeftEye == true)
        myDisplay.drawTriangle(leftEdge, topEdge, rightEdge, topEdge, rightEdge, y + depth);
      else
        myDisplay.drawTriangle(leftEdge, topEdge, rightEdge, topEdge, leftEdge, y + depth);
      break;

    // Happy never gets here: drawEye() draws the arc instead of the eye.
    case Mood::eHappy:
      break;

    // The lid's outer top corner sags; the slit has none.
    case Mood::eSleepy:
    {
      const float k   = (isLeftEye == true) ? myKL : myKR;
      const int   sag = static_cast<int>(static_cast<float>(height) * cSleepyLidSag * (1.0f - k));

      if (sag > 0)
      {
        if (isLeftEye == true)
          myDisplay.drawTriangle(leftEdge, topEdge, rightEdge, topEdge, leftEdge, y + sag);
        else
          myDisplay.drawTriangle(leftEdge, topEdge, rightEdge, topEdge, rightEdge, y + sag);
      }
      break;
    }

    case Mood::eDefault:
    default:
      break;
  }
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::drawTiltedBox(int x, int y, int width, int height, int radius, int drop, bool lowLeft)
{
  const float last = static_cast<float>((width > 1) ? width - 1 : 1);

  for (int column = 0; column < width; column++)
  {
    // Rounded corners: how far this column is pulled in from the top and the bottom.
    const int fromEdge = (column < width - 1 - column) ? column : width - 1 - column;
    int       inset    = 0;

    if (fromEdge < radius)
    {
      const float d = static_cast<float>(radius - fromEdge);
      inset = radius - static_cast<int>(sqrtf(static_cast<float>(radius * radius) - d * d));
    }

    // The shear, centred on the eye so it tilts about its middle.
    const float along = static_cast<float>(column) / last;               // 0 at the left end
    const float lower = (lowLeft == true) ? (0.5f - along) : (along - 0.5f);
    const int   top   = y + inset + static_cast<int>(lower * static_cast<float>(drop) + (lower >= 0 ? 0.5f : -0.5f));
    int         from  = top;
    int         to    = top + height - 2 * inset - 1;

    // u8g2 takes unsigned coordinates, so clip here rather than let a negative one wrap.
    if (from < 0) from = 0;
    if (to > cScreenHeight - 1) to = cScreenHeight - 1;

    const int columnX = x + column;
    if (to >= from && columnX >= 0 && columnX < cScreenWidth)
      myDisplay.drawVLine(columnX, from, to - from + 1);
  }
}

// ----------------------------------------------------------------------------------------
// Perspective, as an image warp: every screen pixel is traced back through the camera onto
// the shell as the gaze has turned it, the shell is turned back to face the camera, and the
// pixel takes the value the flat frame has at that spot. Seen straight on, this maps every
// pixel onto itself, so the turned view is a true 3D view of exactly the flat drawing - moods,
// blinks and all.
//
// Per pixel: one square root and one division, no trigonometry. About 8,000 pixels a frame.
void StatusDisplay::warpToShell(float gazeX, float gazeY)
{
  uint8_t* const buffer = myDisplay.getBufferPtr();

  memcpy(mySource, buffer, sizeof(mySource));
  memset(buffer, 0, sizeof(mySource));

  const float R  = cShellRadius;
  const float D  = cCameraDistance * R;
  const float f  = D - R;               // focal length that draws the shell's front at 1:1
  const float c0 = D * D - R * R;

  const float yaw   = gazeX * cMaxYawDeg   * DEG_TO_RAD;
  const float pitch = gazeY * cMaxPitchDeg * DEG_TO_RAD;
  const float cosYaw   = cosf(yaw),   sinYaw   = sinf(yaw);
  const float cosPitch = cosf(pitch), sinPitch = sinf(pitch);

  const float halfW = static_cast<float>(cScreenWidth)  / 2.0f;
  const float halfH = static_cast<float>(cScreenHeight) / 2.0f;

  for (int py = 0; py < cScreenHeight; py++)
  {
    const float b = (static_cast<float>(py) + 0.5f - halfH) / f;

    for (int px = 0; px < cScreenWidth; px++)
    {
      const float a = (static_cast<float>(px) + 0.5f - halfW) / f;

      // The camera ray through this pixel, (a*t, b*t, D - t), meets the shell where
      // (a^2 + b^2 + 1) t^2 - 2 D t + (D^2 - R^2) = 0. The smaller root is the near side.
      const float k    = a * a + b * b + 1.0f;
      const float disc = D * D - k * c0;

      if (disc < 0.0f)
        continue; // the ray passes beside the shell

      const float t = (D - sqrtf(disc)) / k;
      const float X = a * t;
      const float Y = b * t;
      const float Z = D - t;

      // Turn the point back to the front view: undo the pitch, then the yaw.
      const float y0 = Y * cosPitch - Z * sinPitch;
      const float z1 = Y * sinPitch + Z * cosPitch;
      const float x0 = X * cosYaw - z1 * sinYaw;
      const float z0 = X * sinYaw + z1 * cosYaw;

      if (z0 <= 0.0f)
        continue; // on the far side of the shell

      // Where the camera sees that point when the shell faces it - the flat frame.
      const float s  = f / (D - z0);
      const int   sx = static_cast<int>(floorf(halfW + x0 * s));
      const int   sy = static_cast<int>(floorf(halfH + y0 * s));

      if (sx < 0 || sx >= cScreenWidth || sy < 0 || sy >= cScreenHeight)
        continue;

      // u8g2's full buffer is page-ordered: one byte holds 8 vertical pixels.
      if ((mySource[(sy >> 3) * cScreenWidth + sx] & (1u << (sy & 7))) != 0)
        buffer[(py >> 3) * cScreenWidth + px] |= static_cast<uint8_t>(1u << (py & 7));
    }
  }
}

// ----------------------------------------------------------------------------------------
// The happy arc, drawn in place of the eye: a band that follows a circle whose centre lies
// below the eye, with a round cap at each end. It tapers from cHappyThickness at the top to
// cHappyEndThickness at the caps. Measuring the width across the curve, rather than straight
// down, keeps the stroke smooth on the steep sides.
//
// The arc is laid out for the eye fully open. While the eye is shorter - a blink, or opening
// again after one - each column is squashed towards the arc's middle line, so a blink flattens
// the arc into a line and springs it back, instead of cutting it off at the eye's edges.
//
// Drawn column by column as vertical runs rather than with u8g2's circles, which take
// unsigned coordinates and could not be squashed. The arc is centred on the eye's true middle
// column, (width - 1) / 2, so it is symmetric.
void StatusDisplay::drawHappyArc(int x, int centreY, int width, int height, bool isLeftEye)
{
  if (width <= 1 || height <= 0)
    return;

  const int target = (isLeftEye == true) ? myHeightTargetL : myHeightTargetR;
  const int open   = (height > target) ? height : target;
  const int eyeTop = centreY - open / 2;

  const float w = static_cast<float>(width);
  const float h = static_cast<float>(open);

  const float top     = (w * cHappyThickness    > cHappyMinThickness) ? w * cHappyThickness
                                                                      : cHappyMinThickness;
  const float end     = (w * cHappyEndThickness > cHappyMinThickness) ? w * cHappyEndThickness
                                                                      : cHappyMinThickness;
  const float halfTop = top / 2.0f;
  const float halfEnd = end / 2.0f;
  const float sinSpan = sinf(cHappySpanDeg * DEG_TO_RAD);
  const float cosSpan = cosf(cHappySpanDeg * DEG_TO_RAD);

  // The radius of the band's centre line, chosen so the caps just reach the eye's sides,
  // and the circle's centre, placed so the arc's middle sits at cHappyCentre.
  const float radius    = (w / 2.0f - halfEnd) / sinSpan;
  const float arcHeight = radius * (1.0f - cosSpan) + halfTop + halfEnd;
  const float centreX   = (w - 1.0f) / 2.0f;
  const float centreY0  = h * cHappyCentre - arcHeight / 2.0f + halfTop + radius;

  // The right cap's centre, relative to the circle's centre; the left one mirrors it.
  const float capX = radius * sinSpan;
  const float capY = -radius * cosSpan;

  // Whether a pixel of the open eye belongs to the arc; dx is the distance from the middle
  // column.
  const auto isLit = [&](float dx, int row) {
    const float dy = static_cast<float>(row) + 0.5f - centreY0;

    // Within the arc's span: above the centre and no further round than the caps. along runs
    // from 0 at the top to 1 at the caps, and the half-width narrows with it.
    if (dy < 0.0f && dx * cosSpan <= -dy * sinSpan)
    {
      const float distance = sqrtf(dx * dx + dy * dy);
      const float along    = dx / (distance * sinSpan);
      const float half     = halfEnd + (halfTop - halfEnd) * (1.0f - along * along);
      if (fabsf(distance - radius) <= half)
        return true;
    }

    const float cx = dx - capX;
    const float cy = dy - capY;
    return (cx * cx + cy * cy <= halfEnd * halfEnd);
  };

  // The squash: 1 when open, near 0 when shut, about the arc's middle line.
  const float squash = static_cast<float>(height) / h;
  const float pivot  = static_cast<float>(eyeTop) + h * cHappyCentre;

  for (int column = 0; column < width; column++)
  {
    const int screenX = x + column;
    if (screenX < 0 || screenX >= cScreenWidth)
      continue;

    const float dx = fabsf(static_cast<float>(column) - centreX);

    // Each column of the arc is a single run; find its ends.
    int first = 0;
    while (first < open && isLit(dx, first) == false)
      first++;

    if (first == open)
      continue;

    int last = open - 1;
    while (last > first && isLit(dx, last) == false)
      last--;

    // Squash the run towards the pivot. At least one pixel stays, so a shut eye is a line.
    const float runTop    = pivot + (static_cast<float>(eyeTop + first)    - pivot) * squash;
    const float runBottom = pivot + (static_cast<float>(eyeTop + last + 1) - pivot) * squash;

    int from = static_cast<int>(floorf(runTop    + 0.5f));
    int to   = static_cast<int>(floorf(runBottom + 0.5f)) - 1;
    if (to < from) to = from;

    // u8g2 takes unsigned coordinates, so clip here rather than let one wrap.
    if (from < 0) from = 0;
    if (to > cScreenHeight - 1) to = cScreenHeight - 1;

    if (to >= from)
      myDisplay.drawVLine(screenX, from, to - from + 1);
  }
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::drawError()
{
  const int totalWidth = 2 * cErrorGlyphSize + cErrorGlyphGap;
  const int firstX     = (cScreenWidth - totalWidth) / 2;
  const int glyphY     = (cErrorFaceHeight - cErrorGlyphSize) / 2;

  myDisplay.setDrawColor(cMainColor);

  drawCross(firstX,                                    glyphY, cErrorGlyphSize);
  drawCross(firstX + cErrorGlyphSize + cErrorGlyphGap, glyphY, cErrorGlyphSize);

  drawTextCentred(myErrorTitle, cErrorTitleBaseline);

  if (myErrorDetail[0] != '\0')
    drawTextCentred(myErrorDetail, cErrorDetailBaseline);
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::drawCross(int x, int y, int size)
{
  myDisplay.drawLine(x, y, x + size - 1, y + size - 1);
  myDisplay.drawLine(x, y + size - 1, x + size - 1, y);
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::drawTextCentred(const char* text, int baselineY)
{
  char buffer[cMaxErrorChars];
  snprintf(buffer, sizeof(buffer), "%s", text);

  // Drop trailing characters until the string fits the screen width. getStrWidth() returns
  // an unsigned type, so it is taken as int to keep the comparison and the centring signed.
  size_t length = strlen(buffer);

  while (length > 0 && static_cast<int>(myDisplay.getStrWidth(buffer)) > cScreenWidth)
    buffer[--length] = '\0';

  const int x = (cScreenWidth - static_cast<int>(myDisplay.getStrWidth(buffer))) / 2;

  myDisplay.setDrawColor(cMainColor);
  myDisplay.drawStr(x, baselineY, buffer);
}
