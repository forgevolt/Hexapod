// ---- StatusDisplayTest -----------------------------------------------------------------
// Exercises StatusDisplay on its own, with no hexapod, no servos and no sound.
//
// Wiring: an SH1106 128x64 OLED on the I2C pins below. Nothing else is required.
//
// The sketch walks through every feature of the library, holding each step for
// cStepDurationMs so it can be watched, and printing what it is showing to Serial.
//
// Serial commands (115200 baud):
//   n  advance to the next step immediately
//   p  pause or resume the automatic walkthrough
//   e  raise a fault, replacing the eyes with the fault view (clears itself after 20 s)
//   c  clear the fault
//   ?  print this list

#include <Wire.h>
#include "StatusDisplay.h"

// Adjust to match your wiring.
constexpr int cI2C_SDA = 8;
constexpr int cI2C_SCL = 9;

constexpr uint8_t       cTargetFps      = 30;
constexpr unsigned long cStepDurationMs = 4000;

StatusDisplay myDisplay;

enum class Step : uint8_t
{
  eDefaultEyes = 0,
  eTired,
  eFocused,
  eHappy,
  eWideEyes,
  eGazeSweep,
  eCuriosity,
  eBlinkFast,
  eHidden,
  eFault,
  eFaultLongText,
  eRecovered,
  eNumSteps
};

Step          myStep          = Step::eDefaultEyes;
unsigned long myStepStartedAt = 0;
bool          myPaused        = false;

// ----------------------------------------------------------------------------------------
const char* stepName(Step step)
{
  switch (step)
  {
    case Step::eDefaultEyes:   return "default eyes, idle wandering, slow blink";
    case Step::eTired:         return "mood: tired";
    case Step::eFocused:       return "mood: focused";
    case Step::eHappy:         return "mood: happy, 45x45 held still";
    case Step::eWideEyes:      return "45x45 eyes, gap 20";
    case Step::eGazeSweep:     return "setLookDirection() sweeping a circle";
    case Step::eCuriosity:     return "curiosity on, gaze sweeping left to right";
    case Step::eBlinkFast:     return "blink every second, no variation";
    case Step::eHidden:        return "setVisible(false) - screen should be blank";
    case Step::eFault:         return "showError() - short title and detail";
    case Step::eFaultLongText: return "showError() - text longer than the screen";
    case Step::eRecovered:     return "clearError() - eyes return in their last state";
    default:                   return "unknown";
  }
}

// ----------------------------------------------------------------------------------------
void applyStep(Step step)
{
  // Every step starts from a known configuration, so each one only sets what it changes.
  myDisplay.clearError();
  myDisplay.setVisible(true);
  myDisplay.setMood(StatusDisplay::Mood::eDefault);
  myDisplay.setWidth(36, 36);
  myDisplay.setHeight(36, 36);
  myDisplay.setSpaceBetween(10);
  myDisplay.setCuriosity(false);
  myDisplay.setIdleMode(true, 2, 2);
  myDisplay.setAutoBlinker(true, 3, 2);

  switch (step)
  {
    case Step::eDefaultEyes:
      break;

    case Step::eTired:
      myDisplay.setMood(StatusDisplay::Mood::eTired);
      break;

    case Step::eFocused:
      myDisplay.setMood(StatusDisplay::Mood::eFocused);
      break;

    case Step::eHappy:
      myDisplay.setMood(StatusDisplay::Mood::eHappy);
      break;

    case Step::eWideEyes:
      myDisplay.setWidth(45, 45);
      myDisplay.setHeight(45, 45);
      myDisplay.setSpaceBetween(20);
      break;

    case Step::eGazeSweep:
      // Gaze is driven from loop(); idle would fight it for the target position.
      myDisplay.setIdleMode(false);
      break;

    case Step::eCuriosity:
      myDisplay.setIdleMode(false);
      myDisplay.setCuriosity(true);
      myDisplay.setHeight(40, 40);
      break;

    case Step::eBlinkFast:
      myDisplay.setIdleMode(false);
      myDisplay.setAutoBlinker(true, 1, 0);
      break;

    case Step::eHidden:
      myDisplay.setVisible(false);
      break;

    case Step::eFault:
      myDisplay.showError("SERVO INIT", "ping failed id 43");
      break;

    case Step::eFaultLongText:
      myDisplay.showError("CONFIGURATION FAULT", "joint limits out of order");
      break;

    case Step::eRecovered:
      myDisplay.setMood(StatusDisplay::Mood::eHappy);
      break;

    default:
      break;
  }

  Serial.print("step ");
  Serial.print(static_cast<int>(step));
  Serial.print(": ");
  Serial.println(stepName(step));
}

// ----------------------------------------------------------------------------------------
void nextStep()
{
  const uint8_t next = static_cast<uint8_t>(myStep) + 1;

  myStep          = (next < static_cast<uint8_t>(Step::eNumSteps)) ? static_cast<Step>(next)
                                                                  : Step::eDefaultEyes;
  myStepStartedAt = millis();

  applyStep(myStep);
}

// ----------------------------------------------------------------------------------------
void printHelp()
{
  Serial.println("commands: n = next step, p = pause/resume, e = fault, c = clear, ? = help");
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

    case 'e':
      myDisplay.showError("MANUAL FAULT", "raised from serial");
      Serial.println("fault raised");
      break;

    case 'c':
      myDisplay.clearError();
      Serial.println("fault cleared");
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

  Serial.println("StatusDisplay test");
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
  const float phase = static_cast<float>(millis() % 3000) / 3000.0f * TWO_PI;

  if (myStep == Step::eGazeSweep)
    myDisplay.setLookDirection(cosf(phase), sinf(phase));
  else if (myStep == Step::eCuriosity)
    myDisplay.setLookDirection(cosf(phase), 0.0f);

  myDisplay.update();

  if (myPaused == false && millis() - myStepStartedAt >= cStepDurationMs)
    nextStep();
}
