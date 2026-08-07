#include "Receiver.h"

#include <Streaming.h>
#include <algorithm>   // std::clamp
#include <cmath>       // std::isfinite


// ---- Receiver --------------------------------------------------------------------------

// How long to wait for a transmitter to answer a pairing request before broadcasting again.
constexpr unsigned long cPairingResponseTimeoutMs = 250;

// Cadence of the link maintenance task. The heartbeat must beat the peer's timeout with margin,
// or the far end declares us lost between beats.
constexpr unsigned long cLinkTaskPeriodMs    = 50;
constexpr unsigned long cPairingIntervalMs   = 250;
constexpr unsigned long cHeartbeatIntervalMs = 500;

static_assert(cHeartbeatIntervalMs * 2 <= ESPNowConnection::cPeerTimeoutMs,
              "heartbeat interval must beat cPeerTimeoutMs with margin");


// A line printed from an ESP-NOW receive callback costs about 15 ms of wire time at 115200
// baud, and Serial blocks once its TX buffer fills - time the WiFi task spends not processing
// frames. The sites that can fire on every received frame report no more often than this.
static constexpr unsigned long cLogIntervalMs = 1000;

// True at most once per cLogIntervalMs, against the caller's own timestamp. The first call
// always reports, so a fault is never silent.
static bool logDue(unsigned long& lastMs)
{
  const unsigned long now = millis();

  if (lastMs != 0 && now - lastMs < cLogIntervalMs)
    return false;

  lastMs = now;
  return true;
}

// ----------------------------------------------------------------------------------------
Receiver::Receiver()
: myStatus(EPairingStatus::eNotPaired)
{
  myMutex = xSemaphoreCreateMutex();
} 

// ----------------------------------------------------------------------------------------
Receiver::~Receiver()
{
  // Let linkTask observe the flag and exit on its own before the mutex it uses is destroyed.
  myIsLinkTaskRunning = false;
  vTaskDelay(pdMS_TO_TICKS(cLinkTaskPeriodMs * 2));

  vSemaphoreDelete(myMutex);
}

// ----------------------------------------------------------------------------------------
bool Receiver::begin(uint8_t channel)
{
  // linkTask already running - a second call would start a duplicate.
  if (myLinkTaskHandle != nullptr)
    return true;

  // Checked here rather than in the constructor: this object is a global, constructed before
  // Serial is up, so a failure there could not be reported. A null handle would make every
  // later xSemaphoreTake() undefined.
  if (myMutex == nullptr)
  {
    Serial << "ERROR: " << __PRETTY_FUNCTION__ << " -> mutex could not be created" << endl;
    return false;
  }
    
  if (ESPNowConnection::begin(channel) == false)
    return false;

  myIsLinkTaskRunning = true;
  // Task runs on Core 0 (WIFI)
  if (xTaskCreatePinnedToCore(linkTask, "linkTask", 4096, this, 3, &myLinkTaskHandle, 0) != pdPASS)
  {
    Serial << "ERROR: " << __PRETTY_FUNCTION__ << " -> failed to create linkTask" << endl;
    myLinkTaskHandle    = nullptr;
    myIsLinkTaskRunning = false;
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------------------
void Receiver::linkTask(void* param)
{
  Receiver* r = static_cast<Receiver*>(param);

  unsigned long lastPairingAttempt = 0;
  unsigned long lastHeartbeat      = 0;

  while (r->myIsLinkTaskRunning == true)
  {
    const unsigned long currentMillis = millis();

    // Drop the peer if nothing has been heard from it recently. This is the transport layer's
    // only liveness check, so it runs every cycle regardless of pairing state; clearing the peer
    // is what makes isPaired() false.
    r->removeLostPeer();

    // State 1: Not paired -> Attempt pairing at regular intervals
    if (r->isPaired() == false)
    {
      if (currentMillis - lastPairingAttempt >= cPairingIntervalMs)
      {
        lastPairingAttempt = currentMillis;
        r->autoPairing();
      }
    }
    // State 2: Paired -> Send telemetry/heartbeat at regular intervals
    else
    {
      if (currentMillis - lastHeartbeat >= cHeartbeatIntervalMs)
      {
        lastHeartbeat = currentMillis;
        r->sendHeartbeat();
      }
    }

    // Plain vTaskDelay, not vTaskDelayUntil: if a send ever exceeded the period, DelayUntil
    // returns without blocking and this loop spins. The millis() gating above sets the actual
    // cadence.
    vTaskDelay(pdMS_TO_TICKS(cLinkTaskPeriodMs));
  }

  vTaskDelete(nullptr);
}

// ----------------------------------------------------------------------------------------
void Receiver::autoPairing()
{
  // myStatus and myLastPairingRequest are also written from the WiFi task
  // (onPairingResponseMsg), so every access to them here is under myMutex. The lock is
  // deliberately not held across sendToUnpairedAddress() or isPaired(): the first is a
  // blocking radio call, the second takes the base class's peer lock.
  EPairingStatus status;
  unsigned long lastRequest;
  char receiverName[cNameLen];

  xSemaphoreTake(myMutex, portMAX_DELAY);
  status      = myStatus;
  lastRequest = myLastPairingRequest;
  copyStringToBuffer(receiverName, sizeof(receiverName), myReceiverName);
  xSemaphoreGive(myMutex);

  switch (status)
  {
    case EPairingStatus::eNotPaired:
      {
        // Pairing data to send to transmitter(s)
        PairingRequestData pd;
        pd.device = eHexapod;
        memcpy(pd.name, receiverName, sizeof(pd.name));

        // sendToUnpairedAddress() temporarily registers the broadcast address in the
        // ESP-NOW peer table for the duration of the send, then unregisters it again -
        // it does not touch our tracked (single) peer at all.
        bool sent = sendToUnpairedAddress(ESPNowConnection::cBroadcastAddress, &pd, sizeof(PairingRequestData));

        if (sent == false)
          return; // stay eNotPaired; autoPairing() will simply try again next call

        xSemaphoreTake(myMutex, portMAX_DELAY);
        // Only advance if the WiFi task has not paired us while the broadcast was in flight.
        if (myStatus == EPairingStatus::eNotPaired)
        {
          myLastPairingRequest = millis();
          myStatus = EPairingStatus::ePairingRequested;
        }
        xSemaphoreGive(myMutex);
      }
      break;

    case EPairingStatus::ePairingRequested:
      // No response from a transmitter in time?
      if (millis() - lastRequest > cPairingResponseTimeoutMs)
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        if (myStatus == EPairingStatus::ePairingRequested)   // try again
          myStatus = EPairingStatus::eNotPaired;
        xSemaphoreGive(myMutex);
      }
      break;

    case EPairingStatus::ePaired:
      if (isPaired() == false)   // takes the peer lock - must not be nested inside myMutex
      {
        xSemaphoreTake(myMutex, portMAX_DELAY);
        if (myStatus == EPairingStatus::ePaired)
          myStatus = EPairingStatus::eNotPaired;
        xSemaphoreGive(myMutex);
      }
      break;
  }
}

// ----------------------------------------------------------------------------------------
void Receiver::setName(const char* receiverName)
{
  xSemaphoreTake(myMutex, portMAX_DELAY);

  if (myReceiverName != receiverName) myReceiverName = receiverName;

  xSemaphoreGive(myMutex);
}

// ----------------------------------------------------------------------------------------
void Receiver::setSwitchLabels(const char* switch1Lbl, const char* switch2Lbl,
                               const char* switch3Lbl, const char* switch4Lbl)
{
  xSemaphoreTake(myMutex, portMAX_DELAY);

  if (mySwitchLabels[0] != switch1Lbl) mySwitchLabels[0] = switch1Lbl;
  if (mySwitchLabels[1] != switch2Lbl) mySwitchLabels[1] = switch2Lbl;
  if (mySwitchLabels[2] != switch3Lbl) mySwitchLabels[2] = switch3Lbl;
  if (mySwitchLabels[3] != switch4Lbl) mySwitchLabels[3] = switch4Lbl;

  xSemaphoreGive(myMutex);
}

// ----------------------------------------------------------------------------------------
void Receiver::setButtonLabels(const char* btn1Lbl, const char* btn2Lbl)
{
  xSemaphoreTake(myMutex, portMAX_DELAY);

  if (myBtnLabels[0] != btn1Lbl) myBtnLabels[0] = btn1Lbl;
  if (myBtnLabels[1] != btn2Lbl) myBtnLabels[1] = btn2Lbl;

  xSemaphoreGive(myMutex);
}

// ----------------------------------------------------------------------------------------
void Receiver::setMessage(size_t idx, const char* label, const char* message)
{
  if (idx >= cNumMessages)
  {
    Serial << __PRETTY_FUNCTION__  << " -> idx out of range " << idx << endl;
    return; 
  }

  xSemaphoreTake(myMutex, portMAX_DELAY);
  
  if (myMsgLabels[idx] != label)   myMsgLabels[idx] = label;
  if (myMsgInfo[idx]   != message) myMsgInfo[idx]   = message;

  xSemaphoreGive(myMutex);
}

// ----------------------------------------------------------------------------------------
Receiver::ControlData Receiver::getControlData()
{
  xSemaphoreTake(myMutex, portMAX_DELAY);

  ControlData ret(myData);

  // reset button change state
  myData.joyL = myData.joyR = false;
  myNewData = false;

  xSemaphoreGive(myMutex);

  return ret;
}

// ----------------------------------------------------------------------------------------
void Receiver::sendHeartbeat()
{
  TelemetryData td;

  xSemaphoreTake(myMutex, portMAX_DELAY);

  bool paired = (myStatus == EPairingStatus::ePaired);

  if (paired)
  {
    for (size_t i = 0; i < cNumSwitches; i++)
      copyStringToBuffer(td.switchLabels[i], sizeof(td.switchLabels[i]), mySwitchLabels[i]);

    for (size_t i = 0; i < cNumButtons; i++)
      copyStringToBuffer(td.btnLabels[i], sizeof(td.btnLabels[i]), myBtnLabels[i]);

    for (size_t i = 0; i < cNumMessages; i++)
    {
      copyStringToBuffer(td.messages[i][0], sizeof(td.messages[i][0]), myMsgLabels[i]);
      copyStringToBuffer(td.messages[i][1], sizeof(td.messages[i][1]), myMsgInfo[i]);
    }
  }

  xSemaphoreGive(myMutex);

  // send() takes the base class's own peer mutex internally; it is intentionally
  // called outside of myMutex above to avoid holding two locks at once / keep lock
  // ordering simple.
  if (paired)
    send(td);
}

// ----------------------------------------------------------------------------------------
void Receiver::onPairingResponseMsg(const PairingResponseData& pd, const uint8_t src[6])
{
  xSemaphoreTake(myMutex, portMAX_DELAY);

  // Already paired -> ignore
  if (myStatus == EPairingStatus::ePaired)
  {
    xSemaphoreGive(myMutex);
    return;
  }

  xSemaphoreGive(myMutex);

  // pd.name occupies exactly its declared width on the wire with no guaranteed terminator,
  // so it must not be printed or handed to String() directly.
  char peerName[sizeof(pd.name) + 1];
  copyFieldToCString(peerName, sizeof(peerName), pd.name, sizeof(pd.name));

  Serial << __PRETTY_FUNCTION__  << " -> PairingResponseMsg received from "
         << mac2string(src)
         << " (" << pd.device << ":" << peerName << ")" << endl;

  // setPeer() takes the base class's own peer mutex internally - called outside of
  // myMutex here for the same reason as in sendHeartbeat() above.
  if (setPeer(src, pd.device, pd.name) == false)
  {
    Serial << __PRETTY_FUNCTION__ << " -> Failed to set peer " << mac2string(src) << endl;
    return;
  }

  xSemaphoreTake(myMutex, portMAX_DELAY);
  myStatus = EPairingStatus::ePaired;
  xSemaphoreGive(myMutex);
}

// ----------------------------------------------------------------------------------------
// Ingress validation. cJoystickMin/Max and cIMUAngleMin/Max declare the protocol's value domain;
// these are what enforce it, so nothing outside it reaches the kinematics.

static int16_t clampAxis(int16_t v)
{
  return std::clamp(v, static_cast<int16_t>(cJoystickMin), static_cast<int16_t>(cJoystickMax));
}

// A non-finite angle becomes 0 (level) rather than being clamped. NaN compares false against
// everything, so it survives std::clamp() - including the clamp at the point of use in
// GaitEngine::step() - and then poisons lowPassFilter() permanently, because the filter feeds its
// own output back in. This is the only layer that can catch it.
static float sanitiseAngle(float a)
{
  if (std::isfinite(a) == false)
    return 0.0f;

  return std::clamp(a, cIMUAngleMin, cIMUAngleMax);
}

// ----------------------------------------------------------------------------------------
void Receiver::onAppMsg(uint8_t msgType, const void* data, size_t len, const PeerInfo&)
{
  // This callback runs on core 0 (WIFI)
  //
  // The transport has already confirmed the sender is our paired peer, but it knows nothing
  // about this protocol's frames - so the type, the length and the payload version are all
  // checked here.
  if (msgType != eTransmitter)
    return;

  TransmitterData td;
  if (copyFrameTo(data, len, td) == false)
  {
    static unsigned long lastInvalidLengMs = 0;
    if (logDue(lastInvalidLengMs) == true)
    {
      Serial << __PRETTY_FUNCTION__ << " -> invalid len for TransmitterData: " << len
            << ", expecting: " << sizeof(TransmitterData) << endl;
    }
    return;
  }

  if (td.protocolVersion != cAppProtocolVersion)
  {
    static unsigned long lastVersionLogMs = 0;
    if (logDue(lastVersionLogMs) == true)
    {
      Serial << __PRETTY_FUNCTION__ << " -> app protocol version mismatch: received "
            << int(td.protocolVersion) << ", expected " << int(cAppProtocolVersion)
            << " - the two ends were built from different revisions of RCProtocol.h" << endl;
    }
    return;
  }

  //
  // Values are validated on the way in: this is the boundary between wire data and application
  // state, and the last point at which a bad value can be stopped cheaply.

  xSemaphoreTake(myMutex, portMAX_DELAY);

  myData.joyL |= (td.joyLBtn != 0);
  myData.joyR |= (td.joyRBtn != 0);

  myData.LX = clampAxis(td.joyLX); myData.LY = clampAxis(td.joyLY); myData.LZ = clampAxis(td.joyLZ);
  myData.RX = clampAxis(td.joyRX); myData.RY = clampAxis(td.joyRY); myData.RZ = clampAxis(td.joyRZ);

  myData.switch1 = (td.switch1 != 0); myData.switch2 = (td.switch2 != 0);
  myData.switch3 = (td.switch3 != 0); myData.switch4 = (td.switch4 != 0);

  myData.angleX = sanitiseAngle(td.angleX);
  myData.angleY = sanitiseAngle(td.angleY);

  myNewData = true;
  myLastControlDataMs = millis();

  xSemaphoreGive(myMutex);
}
