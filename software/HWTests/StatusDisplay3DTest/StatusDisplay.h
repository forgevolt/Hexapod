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
//   With perspective on, the eyes sit on a round shell that turns with the gaze instead of
//   sliding across the screen - see setPerspective().
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
      eHappy,       // reshaped into an arc, giving a smiling squint
      eSleepy       // one half-closed lid and one thin slit that swap sides; nods off now and then
    };

    // How the gaze is shown.
    enum class Perspective : uint8_t
    {
      eFlat = 0, // the eyes slide across the screen, as they always have
      eResize    // the shell moves and resizes each eye, which is then drawn as usual
    };

    explicit StatusDisplay();

    // Bring up the display, center the eyes, and push the first frame. Returns false if
    // the display does not come up, in which case update() still runs harmlessly.
    // fps: target frame rate; clamped to at least 1 to keep the frame interval finite.
    bool begin(uint8_t fps);

    // Render and push one frame if the frame interval has elapsed, otherwise return
    // immediately. Call once per iteration of the task that owns the I2C bus.
    void update();

    // Show or hide all output. Only sets a flag: the next frame of update() clears the screen,
    // and while hidden update() renders nothing. Animation state (mood, position, timers,
    // pending error) is preserved.
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

    // Show the eyes as if painted on a round shell that turns with the gaze, instead of
    // sliding them across the screen. With the gaze centred both look exactly the same.
    void setPerspective(Perspective perspective);


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

    // How far a mood mask is drawn outside the eye, to keep a rasterized diagonal from
    // leaving a lit sliver in the corner it cuts away.
    static constexpr int cMaskOvershoot = 2;

    // Fraction of the remaining distance covered per frame, plus a floor of one pixel so
    // a tween always reaches its target rather than converging asymptotically.
    static constexpr int cTweenDivisor = 4;

    // Mood masks, as a fraction of the eye's height: how deep the lid cuts in.
    static constexpr int cLidDepthDivisor = 3;

    // Geometry of the happy arc: a curved band with rounded ends, bending down from its top to
    // the eye's sides, thickest at the top and tapering towards the ends (equal values give an
    // even band). It is sized from the eye's width, so it keeps its shape when the eyes are
    // made taller.
    static constexpr float cHappyThickness    = 0.27f; // stroke width at the top, x eye width
    static constexpr float cHappyEndThickness = 0.12f; //                 at the ends
    static constexpr float cHappyMinThickness = 3.0f;  // pixels
    static constexpr float cHappySpanDeg      = 62.0f; // how far each half bends round from the top
    static constexpr float cHappyCentre       = 0.40f; // middle of the arc, x eye height from its top

    // ---- Blink and idle timing -------------------------------------------------------
    static constexpr uint8_t cBlinkFrames = 2; // frames the eyes stay shut per blink

    // ---- Curiosity -------------------------------------------------------------------
    // Gaze within this fraction of the travel range from an end counts as "at the edge",
    // where the nearer eye has grown by the full cCuriosityGrowth.
    static constexpr float cCuriosityEdge   = 0.25f;
    static constexpr int   cCuriosityGrowth = 15; // pixels of extra height

    // ---- Perspective -----------------------------------------------------------------
    // The shell the eyes are wrapped onto, and how far the gaze turns it. The camera sits
    // cCameraDistance shell radii in front of the shell's centre; nearer exaggerates how much
    // the near eye grows. The front of the shell is drawn at 1:1.
    static constexpr float cShellRadius    = 100.0f; // pixels
    static constexpr float cMaxYawDeg      = 20.0f;  // gaze fully left or right
    static constexpr float cMaxPitchDeg    = 10.0f;  // gaze fully up or down
    static constexpr float cCameraDistance = 3.1f;   // shell radii

    // eResize: a turned eye is taller on its near side than on its far side. 1 draws that
    // taper true to the shell - about 2 pixels at full gaze; 2 doubles it, 0 turns it off.
    static constexpr float cKeystoneGain = 1.0f;

    // ---- Sleepy ----------------------------------------------------------------------
    // One eye is a half-closed lid, the other a thin slit; the slit wanders from one eye to
    // the other and back. k is 0 for the lid and 1 for the slit, and everything below blends
    // between the two with it.
    static constexpr float cSleepyLidClose     = 0.42f; // fraction of the height closed: lid
    static constexpr float cSleepySlitClose    = 0.78f; //                                slit
    static constexpr float cSleepyLidDrop      = 0.04f; // centre line lowered, x height: lid
    static constexpr float cSleepySlitDrop     = 0.16f; //                                slit
    static constexpr float cSleepySlitTiltDeg  = 6.0f;  // the slit droops towards the outside
    static constexpr float cSleepyLidSag       = 0.2f;  // the lid's outer top corner, x height
    static constexpr float cSleepyRadiusScale  = 0.4f;  // crisper corners on the narrow shapes
    static constexpr int   cSleepySpace        = 4;     // the eyes move closer together
    static constexpr unsigned long cSleepySwapMs   = 7000;  // slit there and back
    static constexpr unsigned long cSleepyDozeMs   = 10500; // one nod-off cycle
    static constexpr unsigned long cSleepyDozeAtMs = 8200;  // when in the cycle it nods off
    static constexpr unsigned long cSleepyShutMs   = 1200;  // eyes droop shut over this
    static constexpr unsigned long cSleepyHoldMs   = 200;   // and stay shut this long

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
    void drawHappyArc(int x, int centreY, int width, int height, bool isLeftEye);

    // A rounded box drawn column by column, sheared so one end sits drop pixels lower, and
    // tapered so its left and right sides are the given fractions of its height.
    void drawTiltedBox(int x, int y, int width, int height, int radius, int drop, bool lowLeft,
                       float leftSide = 1.0f, float rightSide = 1.0f);

    // Sleepy: k per eye (0 = lid, 1 = slit) and the doze factor (1 = open, near 0 = shut).
    void sleepyPose(float& kLeft, float& kRight, float& doze) const;

    // eResize: where an eye drawn at x..x+width, centred on centreY, lands once the shell
    // has turned by the gaze (each -1..+1). Replaces all four values; never shrinks a side
    // below 1 pixel. sides, if given, receives the heights of its left and right side as
    // fractions of its height, with cKeystoneGain applied.
    void projectEye(float gazeX, float gazeY, int& x, int& centreY, int& width, int& height,
                    float* sides = nullptr) const;
    void drawCross(int x, int y, int size);

    // Draw text horizontally centered on the given baseline, dropping trailing characters
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
    bool myVisible     = true;
    bool myScreenBlank = true; // the screen currently shows nothing; set and read by update()
    Mood myMood        = Mood::eDefault;

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

    // ---- Perspective and sleepy ------------------------------------------------------
    Perspective   myPerspective = Perspective::eFlat;
    unsigned long mySleepyStart = 0;
    float         myKL = 0.0f, myKR = 0.0f; // sleepy slit weight of the eye being drawn
    int           myDropL = 0, myDropR = 0; // sleepy: centre lines lowered, tweened, pixels

    // The height each eye has fully open, as drawn - the happy arc is laid out for it.
    int           myOpenHeightL = 0, myOpenHeightR = 0;

    // eResize: each eye's left and right side as a fraction of its height. 1 and 1 is an
    // upright box, as in the flat view.
    float         mySidesL[2] = { 1.0f, 1.0f };
    float         mySidesR[2] = { 1.0f, 1.0f };

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
