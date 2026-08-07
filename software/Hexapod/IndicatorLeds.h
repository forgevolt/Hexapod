#pragma once

#include <FastLED.h>
#include <atomic>

// ---- Constants -------------------------------------------------------------------------

constexpr uint8_t  cNumLeds       = 19;
constexpr uint8_t  cLedBrightness = 20;  // Global brightness cap (0-255)


// ---- IndicatorLeds ---------------------------------------------------------------------
// Non-blocking LED effect engine for a single WS2812B strip.
//
// Usage:
//   IndicatorLeds leds;
//   void setup() { leds.begin(); leds.setEffect(Effect::eSweepForward); }
//   void loop()  { leds.update(); }
//
// Switching effects at any time is safe — the cycle always restarts cleanly.

class IndicatorLeds
{
  public:

    // ---- Effect selection -----------------------------------------------------------

    enum class Effect { eNone, ePulseRed, ePulseOrange, eSweepForward, eSweepBackward, eSweepMeetInMiddle };

    IndicatorLeds();

    // Call once in setup() to register the LED strip with FastLED.
    template <int DATA_PIN> 
    void begin() 
    {
      FastLED.addLeds<WS2812B, DATA_PIN, GRB>(myLeds, cNumLeds);
      FastLED.setBrightness(cLedBrightness);
      FastLED.clear(true);
    }


    // Request an effect. Callable from any task: the switch is applied by the next update(),
    // which restarts the cycle from the beginning. Does not touch the strip itself.
    void setEffect(Effect effect);

    // Applies a pending setEffect(), then advances and renders the active effect. Every FastLED
    // call happens here, so this must be called from one task only.
    // Returns immediately if no effect is set or the frame interval has not elapsed.
    void update();


  private:
    // ---- Effect implementations -----------------------------------------------------

    void runPulseRed();
    void runPulseOrange();
    void runSweepForward();
    void runSweepBackward();
    void runSweepMeetInMiddle();

    // ---- Helpers --------------------------------------------------------------------

    // Returns true when the next frame interval has elapsed and resets the timer.
    bool isFrameDue(uint32_t intervalMs);

    // Draws a comet at fractional position [0.0, N-1] in the given colour.
    // tailLength controls how many LEDs form the fading trail.
    // forward=true: tail extends toward lower indices; false: toward higher indices.
    void drawComet(float position, CRGB color, uint8_t tailLength, bool forward);

    // Clears all LEDs without calling FastLED.show().
    void clearAll();

    // ---- State ----------------------------------------------------------------------

    CRGB myLeds[cNumLeds];

    uint32_t myLastFrameTime = 0; // millis() at the last rendered frame

    // Written by setEffect() from any task, consumed by update().
    std::atomic<Effect> myRequestedEffect{Effect::eNone};

    // Owned by the task that calls update().
    Effect myActiveEffect = Effect::eNone;

    // Pulse state
    float myPulsePhase = 3.0f * M_PI / 2.0f; // starts at minimum brightness (sin = -1)

    // Sweep state (shared by all sweep effects)
    float mySweepPos = 0.0f; // distance travelled from the starting end (pixels)
};
