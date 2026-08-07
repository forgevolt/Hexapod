#pragma once

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "RCProtocol.h"

// ---- Receiver --------------------------------------------------------------------------
// Contains the logic specific to the receiver. It connects to a peer (the transmitter).
// The status is eNotPaired when no transmitter is "on air".

class Receiver : public ESPNowConnection
{
  public:
    // Control data received from transmitter
    struct ControlData 
    {
      void reset()
      {
        joyL = joyR = false;
        LX = LY = LZ = 0;
        RX = RY = RZ = 0;
        switch1 = switch2 = switch3 = switch4 = false;
        angleX = angleY = 0.0f;
      }

      // true if the button was physically pressed (change from released to pressed)      
      bool joyL = false, joyR = false;

      // Values for left and right joysticks. Range is [-1000, +1000]
      int LX = 0, LY = 0, LZ = 0;  // joystick position (left)
      int RX = 0, RY = 0, RZ = 0;  // joystick position (right)

      // true if that switch is on. Numbered as on the transmitter's PCB: switch1 is SWITCH 1,
      // which currently opens the transmitter's menu.
      bool switch1 = false, switch2 = false, switch3 = false, switch4 = false;

      // Values of the IMU 
      float angleX = 0.0f, angleY = 0.0f;
    };

  public:
    Receiver();
    ~Receiver() override;

    // Brings up the ESP-NOW link and starts the maintenance task that keeps it alive. Pairing,
    // the telemetry heartbeat and peer-liveness checking are driven internally from that task -
    // there is nothing for the caller to poll.
    bool begin(uint8_t channel = cDefaultWifiChannel) override;

    // These labels are displayed on the transmitter screen. Use these methods before begin()
    // and whenever they change.
    void setName(const char* receiverName);
    void setSwitchLabels(const char* switch1Lbl, const char* switch2Lbl,
                         const char* switch3Lbl, const char* switch4Lbl);
    void setButtonLabels(const char* btn1Lbl, const char* btn2Lbl);
    void setMessage(size_t idx, const char* label, const char* message); // idx in [0, 3]

    // Returns a copy of the latest control data from the transmitter and clears the joyL/joyR
    // edge latches, so a button press is reported exactly once. Access is thread-safe.
    ControlData getControlData();
    bool hasNewControlData() const { return myNewData; }

    // Milliseconds since the last control frame was accepted from the peer. Unsigned
    // arithmetic, so correct across millis() rollover.
    unsigned long timeSinceLastControlData() const { return millis() - myLastControlDataMs.load(); }

  protected:
    void onPairingResponseMsg(const PairingResponseData& pd, const uint8_t src[6]) override;
    void onAppMsg(uint8_t msgType, const void* data, size_t len, const PeerInfo& peer) override;

  private:
    // Link maintenance. Runs on core 0, so a busy control loop on core 1 cannot delay it.
    static void linkTask(void* param);

    // Driven exclusively by linkTask - scheduling them from outside is what allowed the
    // peer-timeout check to go uncalled.
    void autoPairing();
    void sendHeartbeat();

    enum class EPairingStatus
    { 
      eNotPaired,
      ePairingRequested,
      ePaired
    };

    EPairingStatus myStatus;
    unsigned long myLastPairingRequest = 0;
   
    String myReceiverName;
    String mySwitchLabels[cNumSwitches];
    String myBtnLabels[cNumButtons];
    String myMsgLabels[cNumMessages];
    String myMsgInfo[cNumMessages];

    // Control data received from transmitter 
    ControlData myData;

    std::atomic<bool> myNewData = false; // set to true if new data arrived from transmitter

    // millis() when the last TransmitterData frame was accepted. Atomic rather than
    // myMutex-protected so the 200 Hz control loop can test freshness without taking a lock
    // the WiFi task also holds.
    std::atomic<unsigned long> myLastControlDataMs = 0;

    TaskHandle_t      myLinkTaskHandle = nullptr;
    std::atomic<bool> myIsLinkTaskRunning{false};
    
    // Protects myStatus, myReceiverName, mySwitchLabels/myBtnLabels/myMsgLabels/myMsgInfo,
    // and myData - all of which are written from ESP-NOW callbacks (onPairingResponseMsg,
    // onAppMsg - WiFi task context) and/or read from loop() (isPaired,
    // getControlData, sendHeartbeat). Separate from, and no substitute for, the base class's
    // internal peer lock. Lock ordering rule: never call anything that takes the peer lock
    // (isPaired(), getPeer(), setPeer(), send()) while holding this one, and never hold this
    // one across a blocking radio call.
    SemaphoreHandle_t myMutex = nullptr;
};
