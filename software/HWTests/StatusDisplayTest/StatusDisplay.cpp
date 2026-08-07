#include "StatusDisplay.h"

#include <cmath>   // sqrtf
#include <cstring> // strlen

// ---- StatusDisplay ---------------------------------------------------------------------

// The 8-pixel text font used for fault messages. Set once in begin() and never changed,
// so getStrWidth() and drawStr() always measure and draw with the same metrics.
static const uint8_t* const cTextFont = u8g2_font_helvR08_tr;


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

  // Centre the gaze, and start with the eyes shut so the first frames tween them open.
  myPosXTarget = constraintX() / 2;
  myPosYTarget = constraintY() / 2;
  myPosX       = myPosXTarget;
  myPosY       = myPosYTarget;
  myHeightL    = cClosedHeight;
  myHeightR    = cClosedHeight;

  myDisplay.clearBuffer();
  myDisplay.sendBuffer();

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

  if (myVisible == false)
    return;

  myDisplay.clearBuffer();

  if (myHasError == true)
    drawError();
  else
    drawEyes();

  myDisplay.sendBuffer();
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

  myVisible = visible;

  if (myVisible == false)
  {
    myDisplay.clearBuffer();
    myDisplay.sendBuffer();
  }
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

  if (myAutoBlink == false)
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

  myWidthL = tween(myWidthL, myWidthTargetL);
  myWidthR = tween(myWidthR, myWidthTargetR);
  mySpace  = tween(mySpace,  mySpaceTarget);
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
    myHeightL = tween(myHeightL, myHeightTargetL + growthL);
    myHeightR = tween(myHeightR, myHeightTargetR + growthR);
  }

  // Each eye keeps a fixed centre line derived from its target height, so blinking and
  // curiosity growth expand and collapse symmetrically instead of dragging the eye down.
  const int centreYL = myPosY + myHeightTargetL / 2;
  const int centreYR = myPosY + myHeightTargetR / 2;

  const int leftX  = myPosX;
  const int rightX = myPosX + myWidthL + mySpace;

  drawEye(leftX,  centreYL, myWidthL, myHeightL, /*isLeftEye=*/true);
  drawEye(rightX, centreYR, myWidthR, myHeightR, /*isLeftEye=*/false);
}

// ----------------------------------------------------------------------------------------
void StatusDisplay::drawEye(int x, int centreY, int width, int height, bool isLeftEye)
{
  if (width <= 0 || height <= 0)
    return;

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

  // drawRBox() requires each side to be longer than twice the corner radius, so the limit
  // is (side - 1) / 2 rather than side / 2.
  if (radius > (height - 1) / 2) radius = (height - 1) / 2;
  if (radius > (width  - 1) / 2) radius = (width  - 1) / 2;

  myDisplay.setDrawColor(cMainColor);

  if (radius > 0)
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

  // The mask reaches a little beyond the eye, so the rasterised edge of a diagonal cannot
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

    // The lower part of the eye is reshaped into a smiling squint. See drawHappyLid() for
    // the candidate shapes.
    case Mood::eHappy:
      drawHappyLid(x, y, width, height);
      break;

    case Mood::eDefault:
    default:
      break;
  }
}

// ----------------------------------------------------------------------------------------
// The happy arc: a band of constant thickness whose lower edge is lifted furthest from the
// eye's bottom edge at the centre, so it arches. Everything above and below it is masked.
//
// Drawn as one background column per pixel of eye width rather than with u8g2's ellipse,
// which takes unsigned coordinates and would wrap for an eye near the left edge. The profile
// is measured from the eye's true centre, (width - 1) / 2, so the arc is symmetric: width / 2
// would leave it one column off centre.
void StatusDisplay::drawHappyLid(int x, int y, int width, int height)
{
  if (width <= 1 || height <= 0)
    return;

  const int fromRatio = height / cArchThickDivisor;
  const int thickness = (fromRatio > cArchMinThickness) ? fromRatio : cArchMinThickness;
  const int lift      = (height - thickness) * cArchLiftNumerator / cArchLiftDenominator;

  const float centre = static_cast<float>(width - 1) / 2.0f;
  const int   bottom = y + height - 1;

  for (int column = 0; column < width; column++)
  {
    const float offset  = (static_cast<float>(column) - centre) / centre;
    const int   lowEdge = bottom - static_cast<int>(lift * (1.0f - offset * offset));
    const int   topEdge = lowEdge - thickness + 1;

    if (topEdge > y)
      myDisplay.drawBox(x + column, y, 1, topEdge - y);

    if (lowEdge < bottom)
      myDisplay.drawBox(x + column, lowEdge + 1, 1, bottom - lowEdge);
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
