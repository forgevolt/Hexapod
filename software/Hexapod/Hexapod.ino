// ---------------------------------------------------------------------------------------------
// Hexapod — This application implements the control logic for a hexapod robot with 18 servos.
//           It receives motion and control commands from a remote controller via ESP-NOW
//           and translates them into coordinated leg movements.
//
//           Christoph Streit - 2026
// ---------------------------------------------------------------------------------------------

#include <Streaming.h>
#include <esp_system.h>   // esp_reset_reason()
#include "PinMap.h"
#include "Receiver.h"
#include "Hexapod.h"

Receiver receiver;
Hexapod hexapod(receiver);

// ---------------------------------------------------------------------------------------------
static const char* resetReasonName(esp_reset_reason_t reason)
{
  switch (reason)
  {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external reset pin";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "panic / exception";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "other watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep wake";
    case ESP_RST_BROWNOUT:  return "brownout - supply voltage dipped";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "unknown";
  }
}

// ---------------------------------------------------------------------------------------------
void setup()
{
  // Read the USB sense pin before anything else: it tells us whether a host is attached,
  // which decides both how long we wait for the serial port and how much we log.
  pinMode(cUSBSensePin, INPUT);
  const bool usbConnected = (digitalRead(cUSBSensePin) == HIGH);

  Serial.begin(115200);

  // Allow a maximum of 1 second for a USB connection, otherwise boot anyway.
  // Only worth waiting at all if a host is actually there.
  if (usbConnected)
  {
    unsigned long startTime = millis();
    while (!Serial && (millis() - startTime < 1000))
      delay(10);
  }

  Serial << "\n\nSW version from " << __DATE__ << " " << __TIME__ << endl;
  Serial << (usbConnected ? "USB connected" : "USB not connected") << endl;

  // Why did the last run end? Anything other than a power-on or a deliberate reset means it ended
  // abnormally - a panic, a watchdog timeout, or the supply dipping. Unchecked, such a reboot is
  // indistinguishable from a normal start. With no battery monitoring in hardware,
  // ESP_RST_BROWNOUT is also the only evidence that a run ended because the pack gave out rather
  // than because of a firmware fault. Logged here, before anything else can report an error of
  // its own.
  const esp_reset_reason_t resetReason = esp_reset_reason();
  const bool abnormalReset = (resetReason != ESP_RST_POWERON &&
                              resetReason != ESP_RST_EXT &&
                              resetReason != ESP_RST_SW);

  Serial << "Reset reason: " << resetReasonName(resetReason)
         << (abnormalReset ? "   *** previous run ended abnormally ***" : "") << endl;

  // Verbose IDF logging only when a host is attached to receive it. Untethered there is
  // nobody reading, but the messages would still be formatted and pushed into the same
  // UART that the control loop shares - so keep it down to warnings.
  esp_log_level_set("*", usbConnected ? ESP_LOG_VERBOSE : ESP_LOG_WARN);

  Wire.begin(cI2C_SDA, cI2C_SCL); // better to explicitly set them

  if (receiver.begin(ESPNowConnection::cDefaultWifiChannel) == false)
  {
    // Continuing is harmless here: without a link, isLinkHealthy() never becomes true and the
    // control loop keeps the robot parked.
    Serial << "ERROR: " << __PRETTY_FUNCTION__ << " -> receiver.begin() failed" << endl;
  }

  const bool hexapodIsUp = hexapod.begin(abnormalReset);

  if (hexapodIsUp == false)
  {
    // Continuing is deliberate, and unlike the receiver it is not harmless: the state machine will
    // still reach eReady and can be commanded to stand on a servo bus that failed to initialise.
    Serial << "ERROR: " << __PRETTY_FUNCTION__ << " -> hexapod.begin() failed" << endl; 
  }
  else if (abnormalReset == true)
  {
    // Only when begin() succeeded: it raises its own fault on failure, and a dead servo bus
    // matters more than how the last run ended. Clears itself after 20 s.
    hexapod.statusDisplay().showError("ABNORMAL RESET", resetReasonName(resetReason));
  }

  Serial << "Setup Complete. Starting ..." << endl;
}
  
// ---------------------------------------------------------------------------------------------
void loop() 
{
  const unsigned long currentMillis = millis();

  // Report what the control task counted, at most once per second and never from the control
  // task itself. Silence means nothing was clamped and every bus write succeeded.
  static unsigned long lastDiagReport = 0;
  if (currentMillis - lastDiagReport >= 1000)
  {
    lastDiagReport = currentMillis;

    const Hexapod::Diagnostics diag = hexapod.fetchDiagnostics();

    if (diag.unreachableTargets != 0)
      Serial << "IK: " << diag.unreachableTargets << " unreachable target(s), worst: leg "
             << diag.worstLeg << " "
             << (diag.worstOvershoot > 0.0f ? "too far by " : "too close by ")
             << fabsf(diag.worstOvershoot) << " mm" << endl;

    if (diag.clampedGoals != 0)
      Serial << "Servo: " << diag.clampedGoals << " goal(s) clamped, worst: idx "
             << diag.worstGoalServo << " by " << diag.worstGoalOvershoot << " ticks" << endl;

    if (diag.syncWriteFails != 0)
      Serial << "Servo: " << diag.syncWriteFails << " syncWrite failure(s)" << endl;
  }

  // Yield execution to lower-priority tasks / system IDLE task
  vTaskDelay(pdMS_TO_TICKS(10));
}
