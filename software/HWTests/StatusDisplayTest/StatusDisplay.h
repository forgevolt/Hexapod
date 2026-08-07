#pragma once

/*
 * StatusDisplay - animated eyes and fault reporting on a 128x64 SH1106 OLED
 *
 * This library is based on the ideas of the third-party RoboEyes library by Dennis Hoelscher
 * (https://github.com/FluxGarage/RoboEyes).
 *
 * Text rendering uses the U8g2 font u8g2_font_helvR08_tr, derived from HELVR08.BDF:
 *   Copyright 1984-1989, 1994 Adobe Systems Incorporated.
 *   Copyright 1988, 1994 Digital Equipment Corporation.
 *   Permission to use, copy, modify, distribute and sell this software and its
 *   documentation for any purpose and without fee is hereby granted, provided that the
 *   above copyright notices appear in all copies. Adobe Systems and Digital Equipment
 *   Corporation make no representations about the suitability of this software for any
 *   purpose. It is provided "as is" without express or implied warranty.
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>


// ---- StatusDisplay ---------------------------------------------------------------------
// Owns the OLED and renders one of two things:
//
// - Eyes: two rounded rectangles that tween toward their configured size and position,
//   blink on a timer, and take on a mood by masking parts of the eye with background.
// - Error: a fault face (crossed eyes) above two lines of text, entered by showError()
//   and held until clearError(). Errors override the eyes entirely.
//
// Animation is frame-based: update() renders at most one frame per frame interval and
// returns immediately otherwise, so it is safe to call as fast as the owning task loops.
//
// Not thread-safe by design. The OLED shares its I2C bus with other devices, so every
// call including update() must come from the single task that owns that bus.

class StatusDisplay
{
  public:
    // Mood expressions. Mutually exclusive; eDefault clears any mask.
    enum class Mood : uint8_t
    {
      eDefault = 0, // plain eyes
      eTired,       // upper outer corners masked, giving a drooping lid
      eFocused,     // upper inner corners masked, giving brows drawn together
      eHappy        // reshaped into an arc, giving a smiling squint
    };

    explicit StatusDisplay();

    // Bring up the display, centre the eyes, and push the first frame. Returns false if
    // the display does not come up, in which case update() still runs harmlessly.
    // fps: target frame rate; clamped to at least 1 to keep the frame interval finite.
    bool begin(uint8_t fps);

    // Render and push one frame if the frame interval has elapsed, otherwise return
    // immediately. Call once per iteration of the task that owns the I2C bus.
    void update();

    // Show or hide all output. While hidden the screen is cleared and update() renders
    // nothing; animation state (mood, position, timers, pending error) is preserved.
    void setVisible(bool visible);


    // ---- Eye configuration -----------------------------------------------------------

    // Target eye size in pixels, per eye. The rendered size tweens toward the target.
    void setWidth(uint8_t leftEye, uint8_t rightEye);
    void setHeight(uint8_t leftEye, uint8_t rightEye);

    // Gap between the eyes in pixels. Negative values overlap them.
    void setSpaceBetween(int space);

    void setMood(Mood mood);

    // Enable automatic blinking. intervalS is the minimum time between blinks and
    // variationS the maximum random time added to it, both in whole seconds.
    void setAutoBlinker(bool active, int intervalS = 3, int variationS = 2);

    // Enable idle wandering: the eyes move to a random reachable position every
    // intervalS seconds plus up to variationS seconds of random delay.
    void setIdleMode(bool active, int intervalS = 2, int variationS = 2);

    // Enable curiosity: whichever eye is nearer the screen edge grows taller as the gaze
    // approaches that edge.
    void setCuriosity(bool active);

    // Aim the gaze directly. x and y are clamped to [-1, +1], where -1 is left/top and
    // +1 is right/bottom of the reachable range. Overrides idle mode for that frame.
    void setLookDirection(float x, float y = 0.0f);


    // ---- Fault reporting -------------------------------------------------------------

    // Replace the eyes with the fault face and the given text until clearError().
    // The fault also clears itself after cErrorTimeoutMs, so a transient condition does not
    // hold the screen for the rest of the session.
    //
    // Showing a fault makes the display visible: it would be pointless otherwise. A
    // setVisible() call made while a fault is up is remembered and applied when the fault
    // clears, so the eyes return in whatever state the caller last asked for.
    // Both strings are copied, so they need not outlive the call. Each is truncated to
    // cMaxErrorChars and then to whatever fits the screen width. detail may be nullptr
    // for a title-only report.
    void showError(const char* title, const char* detail = nullptr);

    // Return to the eyes. Safe to call when no error is showing.
    void clearError();

    bool hasError() const { return myHasError; }

  private:
    // ---- Screen and layout -----------------------------------------------------------
    static constexpr int      cScreenWidth       = 128;    // pixels
    static constexpr int      cScreenHeight      = 64;     // pixels
    static constexpr uint32_t cDisplayBusClockHz = 800000;
    static constexpr uint8_t  cBgColor           = 0;      // pixel off
    static constexpr uint8_t  cMainColor         = 1;      // pixel on

    // ---- Eye appearance --------------------------------------------------------------
    static constexpr int cCornerRadius = 8; // clamped to half the current width/height

    // Eye height while blinking. Reached in a single frame rather than tweened, so a blink
    // actually closes the eye within cBlinkFrames.
    static constexpr int cClosedHeight = 1;

    // How far a mood mask is drawn outside the eye, to keep a rasterised diagonal from
    // leaving a lit sliver in the corner it cuts away.
    static constexpr int cMaskOvershoot = 2;

    // Fraction of the remaining distance covered per frame, plus a floor of one pixel so
    // a tween always reaches its target rather than converging asymptotically.
    static constexpr int cTweenDivisor = 4;

    // Mood masks, as a fraction of the eye's height: how deep the lid cuts in.
    static constexpr int cLidDepthDivisor = 3;

    // Geometry of the happy arc. The band is height/cArchThickDivisor thick, and its lower
    // edge is lifted off the eye's bottom by the given fraction of the room available. Raise
    // the numerator toward the denominator for a taller arc: at 1/1 the apex reaches the top
    // of the eye, which spans the full height but leaves the arc looking like a tunnel.
    static constexpr int cArchThickDivisor    = 4;
    static constexpr int cArchLiftNumerator   = 3;
    static constexpr int cArchLiftDenominator = 4;
    static constexpr int cArchMinThickness    = 3;

    // ---- Blink and idle timing -------------------------------------------------------
    static constexpr uint8_t cBlinkFrames = 2; // frames the eyes stay shut per blink

    // ---- Curiosity -------------------------------------------------------------------
    // Gaze within this fraction of the travel range from an end counts as "at the edge",
    // where the nearer eye has grown by the full cCuriosityGrowth.
    static constexpr float cCuriosityEdge   = 0.25f;
    static constexpr int   cCuriosityGrowth = 15; // pixels of extra height

    // ---- Fault view ------------------------------------------------------------------
    static constexpr int    cErrorFaceHeight     = 30; // rows reserved for the crossed eyes
    static constexpr int    cErrorGlyphSize      = 20; // width and height of one cross
    static constexpr int    cErrorGlyphGap       = 16; // gap between the two crosses
    static constexpr int    cErrorTitleBaseline  = 45;
    static constexpr int    cErrorDetailBaseline = 58;
    static constexpr size_t cMaxErrorChars       = 32; // buffer size, incl. terminator

    // How long a fault stays on screen before clearing itself.
    static constexpr unsigned long cErrorTimeoutMs = 20000;

    // ---- Rendering -------------------------------------------------------------------
    bool isFrameDue();
    void drawEyes();
    void drawError();
    void drawEye(int x, int centreY, int width, int height, bool isLeftEye);
    void drawMoodMask(int x, int y, int width, int height, bool isLeftEye);
    void drawHappyLid(int x, int y, int width, int height);
    void drawCross(int x, int y, int size);

    // Draw text horizontally centred on the given baseline, dropping trailing characters
    // until it fits the screen width.
    void drawTextCentred(const char* text, int baselineY);

    // Advance one tween step of value toward target.
    static int tween(int value, int target);

    // Maximum X for the left eye, and Y for both, that keeps the eyes on screen.
    // Never negative: 0 means the eyes fill that axis and cannot move.
    int constraintX() const;
    int constraintY() const;

    void applyBlinkTimer();
    void applyIdleTimer();

    // ---- Display ---------------------------------------------------------------------
    U8G2_SH1106_128X64_NONAME_F_HW_I2C myDisplay;

    // ---- Frame timing ----------------------------------------------------------------
    int           myFrameInterval = 33; // ms per frame, derived from fps
    unsigned long myLastFrameTime = 0;

    // ---- Visibility and mood ---------------------------------------------------------
    bool myVisible = true;
    Mood myMood    = Mood::eDefault;

    // ---- Eye geometry: target values, and the tweened values actually drawn ----------
    int myWidthTargetL  = 36, myWidthTargetR  = 36;
    int myHeightTargetL = 36, myHeightTargetR = 36;
    int myWidthL        = 36, myWidthR        = 36;
    int myHeightL       = 1,  myHeightR       = 1; // start shut; begin() tweens them open
    int mySpaceTarget   = 10, mySpace         = 10;

    // Left-eye position; the right eye is derived from it each frame.
    int myPosXTarget = 0, myPosYTarget = 0;
    int myPosX       = 0, myPosY       = 0;

    // ---- Blinking --------------------------------------------------------------------
    bool          myAutoBlink       = false;
    int           myBlinkIntervalS  = 3;
    int           myBlinkVariationS = 2;
    unsigned long myNextBlinkTime   = 0;
    uint8_t       myBlinkFramesLeft = 0;

    // ---- Idle ------------------------------------------------------------------------
    bool          myIdle           = false;
    int           myIdleIntervalS  = 2;
    int           myIdleVariationS = 2;
    unsigned long myNextIdleTime   = 0;

    // ---- Gaze ------------------------------------------------------------------------
    bool  myCuriosity = false;
    bool  myLookHeld  = false; // setLookDirection() was called since the last frame
    float myLookX     = 0.0f;
    float myLookY     = 0.0f;

    // ---- Fault state -----------------------------------------------------------------
    bool          myHasError      = false;
    unsigned long myErrorRaisedAt = 0;

    // The visibility to restore when the fault clears - see showError().
    bool myVisibleAfterError = true;

    char myErrorTitle[cMaxErrorChars]  = "";
    char myErrorDetail[cMaxErrorChars] = "";
};
