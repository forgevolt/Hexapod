// ---- StatusDisplay3DTest ---------------------------------------------------------------
// Tries out the 3D eyes and the sleepy mood on their own, with no hexapod, no servos and no
// sound. StatusDisplay.h/.cpp here are a copy of the hexapod's, extended with
// setPerspective() and Mood::eSleepy; nothing in software/Hexapod uses them yet.
//
// With perspective on, the eyes are drawn exactly as before and then wrapped onto a round
// shell that turns with the gaze. With the gaze centred they must look exactly like the
// current eyes; the first step checks that.
//
// Wiring: an SH1106 128x64 OLED on the I2C pins below. Nothing else is required.
//
// The sketch walks through the steps below, printing what each one shows to Serial, plus
// the slowest update() of the step just finished - the cost of the warp.
//
// Serial commands (115200 baud):
//   n  advance to the next step immediately
//   p  pause or resume the automatic walkthrough
//   d  switch perspective off and on, to compare with the flat eyes
//   ?  print this list

#include <Wire.h>
#include "StatusDisplay.h"

// Adjust to match your wiring.
constexpr int cI2C_SDA = 8;
constexpr int cI2C_SCL = 9;

constexpr uint8_t cTargetFps = 30;

StatusDisplay myDisplay;

enum class Step : uint8_t
{
  eFrontView = 0,
  eIdle,
  eGazeCircle,
  eSideToSide,
  eTired,
  eFocused,
  eHappy,
  eTallEyes,
  eSleepy,
  eNumSteps
};

Step          myStep          = Step::eFrontView;
unsigned long myStepStartedAt = 0;
bool          myPaused        = false;
bool          myPerspective   = true;
unsigned long mySlowestUs     = 0;

// ----------------------------------------------------------------------------------------
const char* stepName(Step step)
{
  switch (step)
  {
    case Step::eFrontView:  return "gaze centred - must look exactly like the current eyes";
    case Step::eIdle:       return "idle wandering";
    case Step::eGazeCircle: return "setLookDirection() sweeping a circle";
    case Step::eSideToSide: return "gaze sweeping left to right";
    case Step::eTired:      return "mood: tired, gaze sweeping";
    case Step::eFocused:    return "mood: focused, gaze sweeping";
    case Step::eHappy:      return "mood: happy, gaze sweeping";
    case Step::eTallEyes:   return "36x45 eyes as when walking, gaze sweeping";
    case Step::eSleepy:     return "mood: sleepy - slit swaps sides, nods off every ~10 s";
    default:                return "unknown";
  }
}

// ----------------------------------------------------------------------------------------
unsigned long stepDurationMs(Step step)
{
  return (step == Step::eSleepy) ? 22000 : 6000;
}

// ----------------------------------------------------------------------------------------
void applyStep(Step step)
{
  // Every step starts from a known configuration, so each one only sets what it changes.
  myDisplay.setMood(StatusDisplay::Mood::eDefault);
  myDisplay.setWidth(36, 36);
  myDisplay.setHeight(36, 36);
  myDisplay.setSpaceBetween(10);
  myDisplay.setCuriosity(false);
  myDisplay.setIdleMode(false);
  myDisplay.setAutoBlinker(true, 3, 2);

  switch (step)
  {
    case Step::eIdle:     myDisplay.setIdleMode(true, 2, 2);                   break;
    case Step::eTired:    myDisplay.setMood(StatusDisplay::Mood::eTired);      break;
    case Step::eFocused:  myDisplay.setMood(StatusDisplay::Mood::eFocused);    break;
    case Step::eHappy:    myDisplay.setMood(StatusDisplay::Mood::eHappy);      break;
    case Step::eTallEyes: myDisplay.setHeight(45, 45);                         break;
    case Step::eSleepy:   myDisplay.setMood(StatusDisplay::Mood::eSleepy);     break;
    default:                                                                   break;
  }

  Serial.print("step ");
  Serial.print(static_cast<int>(step));
  Serial.print(": ");
  Serial.println(stepName(step));
}

// ----------------------------------------------------------------------------------------
void nextStep()
{
  Serial.print("  slowest update(): ");
  Serial.print(mySlowestUs);
  Serial.println(" us");
  mySlowestUs = 0;

  const uint8_t next = static_cast<uint8_t>(myStep) + 1;

  myStep          = (next < static_cast<uint8_t>(Step::eNumSteps)) ? static_cast<Step>(next)
                                                                  : Step::eFrontView;
  myStepStartedAt = millis();

  applyStep(myStep);
}

// ----------------------------------------------------------------------------------------
void printHelp()
{
  Serial.println("commands: n = next step, p = pause/resume, d = perspective off/on, ? = help");
}

// ----------------------------------------------------------------------------------------
void handleSerial()
{
  if (Serial.available() == 0)
    return;

  switch (Serial.read())
  {
    case 'n':
      nextStep();
      break;

    case 'p':
      myPaused = !myPaused;
      Serial.println(myPaused ? "paused" : "resumed");
      break;

    case 'd':
      myPerspective = !myPerspective;
      myDisplay.setPerspective(myPerspective);
      Serial.println(myPerspective ? "perspective on" : "perspective off (flat, as today)");
      break;

    case '?':
      printHelp();
      break;

    default:
      break;
  }
}

// ----------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(500); // let the USB CDC port attach before the first print

  Wire.begin(cI2C_SDA, cI2C_SCL);

  if (myDisplay.begin(cTargetFps) == false)
    Serial.println("ERROR: display did not come up - check wiring and I2C address");

  myDisplay.setPerspective(myPerspective);

  Serial.println("StatusDisplay 3D test");
  printHelp();

  myStepStartedAt = millis();
  applyStep(myStep);
}

// ----------------------------------------------------------------------------------------
void loop()
{
  handleSerial();

  // Drive the gaze for the steps that test it. setLookDirection() must be called every
  // frame, since it only overrides idle mode for the frame it was set on.
  const float phase = static_cast<float>(millis() % 4000) / 4000.0f * TWO_PI;

  switch (myStep)
  {
    case Step::eFrontView:  myDisplay.setLookDirection(0.0f, 0.0f);                         break;
    case Step::eGazeCircle: myDisplay.setLookDirection(cosf(phase), sinf(phase));           break;
    case Step::eSleepy:     myDisplay.setLookDirection(0.2f * sinf(phase / 2.0f), 0.2f);    break;
    case Step::eIdle:                                                                        break;
    default:                myDisplay.setLookDirection(cosf(phase), 0.0f);                  break;
  }

  const unsigned long start = micros();
  myDisplay.update();
  const unsigned long took = micros() - start;

  if (took > mySlowestUs)
    mySlowestUs = took;

  if (myPaused == false && millis() - myStepStartedAt >= stepDurationMs(myStep))
    nextStep();
}
