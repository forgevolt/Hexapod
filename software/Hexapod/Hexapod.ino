// ---------------------------------------------------------------------------------------------
// Hexapod — This application implements the control logic for a hexapod robot with 18 servos.
//           It receives motion and control commands from a remote controller via ESP-NOW
//           and translates them into coordinated leg movements.
//
//           Christoph Streit - 2026
// ---------------------------------------------------------------------------------------------

#include <Streaming.h>
#include <esp_system.h>   // esp_reset_reason()
#include <esp_core_dump.h> // esp_core_dump_get_summary()
#include <iterator>        // std::size
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
// A panic (including a task watchdog timeout) saves a core dump to the coredump partition. This
// prints its summary: the task that crashed, its program counter and backtrace. Decode the
// addresses against the .elf of the same build, e.g. with the ESP Exception Decoder or
//   xtensa-esp32s3-elf-addr2line -pfiaC -e Hexapod.ino.elf <address> ...
// The dump stays in flash until the next panic overwrites it, so it is erased once it has been
// printed to an open serial monitor; otherwise it would be reported again at every boot.
static void reportCoreDump()
{
  if (esp_core_dump_image_check() != ESP_OK)
    return; // no dump stored, or a corrupted one

  esp_core_dump_summary_t summary;
  if (esp_core_dump_get_summary(&summary) != ESP_OK)
    return;

  Serial << "Core dump from an earlier crash: task " << summary.exc_task
         << ", PC 0x" << _HEX(summary.exc_pc) << endl;

  Serial << "  Backtrace:";
  for (uint32_t i = 0; i < summary.exc_bt_info.depth && i < std::size(summary.exc_bt_info.bt); i++)
    Serial << " 0x" << _HEX(summary.exc_bt_info.bt[i]);
  Serial << (summary.exc_bt_info.corrupted ? " (corrupted)" : "") << endl;

  if (Serial)
    esp_core_dump_image_erase();
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

  // Not only after an abnormal reset: a crash on battery is usually followed by a power-on
  // once the robot is back on the bench.
  reportCoreDump();

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
    // Continuing is deliberate: it lets a test board without servos run the state machine, the
    // display and the LEDs. It is also safe - ServoBus refuses every move, torque and read until
    // all servos are configured, so the legs stay limp whatever state the robot reaches.
    Serial << "ERROR: " << __PRETTY_FUNCTION__ << " -> hexapod.begin() failed" << endl; 
  }
  else if (abnormalReset == true)
  {
    // Only when begin() succeeded: it raises its own fault on failure, and a dead servo bus
    // matters more than how the last run ended. Clears itself after 20 s.
    hexapod.statusDisplay().showError("ABNORMAL RESET", resetReasonName(resetReason));
  }

  Serial << "Setup complete. Starting ..." << endl;
}
  
// ---------------------------------------------------------------------------------------------
void loop() 
{
  const unsigned long currentMillis = millis();

  // Report what the control task counted, at most once per second and never from the control
  // task itself. Silence means nothing was clamped, every bus write succeeded and the control
  // loop kept its rate.
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

    if (diag.torqueRefusals != 0)
      Serial << "Servo: torque-on refused " << diag.torqueRefusals
             << " time(s) - present positions could not be read" << endl;

    if (diag.overruns != 0)
      Serial << "Control loop: " << diag.overruns << " overrun(s), slowest step() "
             << diag.worstStepUs << " us" << endl;
  }

  // Yield execution to lower-priority tasks / system IDLE task
  vTaskDelay(pdMS_TO_TICKS(10));
}
