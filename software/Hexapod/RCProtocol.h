#pragma once

// ----------------------------------------------------------------------------------------
// RCProtocol - the wire format of the remote-control link
//
// The transmitter owns this file. It describes what the transmitter can send (joysticks,
// switches, buttons, its IMU) and what it can display (switch and button labels, and a few
// label/value pairs), so its shape follows the transmitter's hardware rather than any one
// robot's needs. The same transmitter drives several robots - see EDeviceType.
//
// THIS IS A COPY. The master lives in the Transmitter repository; edit it there and copy the
// result here. Arduino cannot include across sketch folders, which is why it is duplicated
// rather than shared. cAppProtocolVersion below is the guard: both ends check it on every
// frame, so a copy that has fallen behind shows up as a logged mismatch at pairing rather
// than as silently misread data.
//
// The transport that carries these payloads is generic and knows nothing about them; it
// lives in ESPNowUtilities.h and is installed as a library.
// ----------------------------------------------------------------------------------------

#include <ESPNowUtilities.h>

// Bump whenever the layout or the meaning of anything in this file changes, and reflash both
// ends together. Independent of the transport's cLinkProtocolVersion, so a change to the
// pairing protocol and a change to these payloads do not invalidate each other.
constexpr uint8_t cAppProtocolVersion = 1;

// Message types for this protocol. Numbered from the first value the transport reserves for
// applications; everything below that is its own pairing traffic.
enum EAppMsgType : uint8_t
{
  eTransmitter = ESPNowConnection::cAppMsgTypeFirst, // 0xA3
  eTelemetry                                         // 0xA4
};


// ----------------------------------------------------------------------------------------
// Devices that can appear on this link. Carried in the pairing frames, where the transport
// treats it as an opaque uint8_t and never interprets it.
// ----------------------------------------------------------------------------------------

enum EDeviceType : uint8_t
{
  eUndefined = 0, // must stay 0: the transport defaults the pairing frames' device field to it
  eTransmitter1Joy, // 1 joystick configuration
  eTransmitter2Joy, // 2 joysticks configuration
  eHexapod,
  eBalancingCube
};


// ----------------------------------------------------------------------------------------
// Payloads and their value domains
// ----------------------------------------------------------------------------------------

// Min/max values of left and right joysticks
constexpr int cJoystickMin = -1000;
constexpr int cJoystickMax =  1000;

// Min/max angles of IMU (degrees)
constexpr float cIMUAngleMin = -180.0f;
constexpr float cIMUAngleMax = 180.0f;

// Constants for the TelemetryData information elements
constexpr size_t cNumSwitches = 4;
constexpr size_t cNumButtons  = 2;
constexpr size_t cNumMessages = 4;
constexpr size_t cLabelLen    = 16;

// ---- Information of the RC transmitter, i.e. joystick values, states of the switches, ...

struct __attribute__((packed)) TransmitterData
{
  uint8_t msgType         = eTransmitter;
  uint8_t protocolVersion = cAppProtocolVersion;

  // Values for left and right joysticks. Range is [-1000, +1000]
  int16_t joyLX = 0, joyLY = 0, joyLZ = 0;
  int16_t joyRX = 0, joyRY = 0, joyRZ = 0;

  // Flags are uint8_t on the wire: a bool object whose byte is neither 0 nor 1 is undefined to
  // read, and nothing constrains what arrives here. Treat any non-zero value as true.
  uint8_t joyLBtn = 0, joyRBtn = 0;              // the two joystick buttons

  // The four switches. All four are sent whatever the transmitter uses them for. At present
  // SWITCH 1 opens the transmitter's own menu, so switch1 also tells the robot that the
  // operator is in the menu and that the zeroed axes in this frame are deliberate. A robot
  // that assigns a function to switch1 will therefore see it toggle whenever the menu is opened.
  uint8_t switch1 = 0, switch2 = 0, switch3 = 0, switch4 = 0;

  // Values of the IMU in degrees. Range [-180, 180]
  float angleX = 0.0f, angleY = 0.0f;
};

// ---- Information sent by the receiver

struct __attribute__((packed)) TelemetryData
{
  uint8_t msgType         = eTelemetry;
  uint8_t protocolVersion = cAppProtocolVersion;

  // Labels for the four switches, switchLabels[0] being SWITCH 1. While the transmitter binds
  // SWITCH 1 to its menu it shows "Menu" there and ignores whatever arrives in slot 0.
  char switchLabels[cNumSwitches][cLabelLen] = {};
  char btnLabels[cNumButtons][cLabelLen]     = {}; // Labels for the two buttons of the transmitter

  // Up to cNumMessages label/value pairs, e.g. { "height=", "13.4" }
  char messages[cNumMessages][2][cLabelLen] = {};

  // Additional information e.g. position of the servos or vectors, indicating the position of the legs
};


// The wire format is frozen deliberately. If either of these fires, the layout has changed and
// cAppProtocolVersion above must be bumped - and both ends reflashed.
static_assert(sizeof(TransmitterData) ==  28, "TransmitterData wire format changed");
static_assert(sizeof(TelemetryData)   == 226, "TelemetryData wire format changed");

// Guard against outgrowing what ESP-NOW can carry in one frame - TelemetryData is expected to
// gain fields. Taken from the SDK rather than hardcoded: v1 caps a payload at 250 bytes
// (ESP_NOW_MAX_DATA_LEN), v2 at 1470 (ESP_NOW_MAX_DATA_LEN_V2). Anything above 250 requires
// both ends to be v2-capable.
#ifdef ESP_NOW_MAX_DATA_LEN_V2
static_assert(sizeof(TelemetryData) <= ESP_NOW_MAX_DATA_LEN_V2, "TelemetryData exceeds the ESP-NOW v2 payload limit");
#else
static_assert(sizeof(TelemetryData) <= ESP_NOW_MAX_DATA_LEN,    "TelemetryData exceeds the ESP-NOW payload limit");
#endif
