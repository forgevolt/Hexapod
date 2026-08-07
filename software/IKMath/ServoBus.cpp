#include "ServoBus.h"
#include "MathUtilities.h"
#include <Streaming.h>

using namespace ControlTableItem;

// ---- ServoBus --------------------------------------------------------------------------

#define DXL_SERIAL Serial1 
const float cDXLProtocolVersion = 2.0;
const unsigned long cBaud       = 3000000; 

// Goal PWM (item 100) is expressed in units of PWM Limit (item 36), which begin() pins to
// cPWMLimitRaw so this conversion is deterministic rather than depending on whatever the servo
// happens to hold. 885 is the XC430 maximum and its factory default.
const int32_t cPWMLimitRaw = 885;
const int16_t cMaxPWMRaw   = 885; // 100 %
const int16_t cSafePWMRaw  = 442; //  50 %

// Low-voltage backstop written to each servo's MIN_VOLTAGE_LIMIT, in the control table's
// unit of 0.1 V. 99 = 9.9 V = 3.3 V/cell on a 3S LiPo.
const int32_t cMinVoltageLimit = 99;

// Address and length control table item "Goal Position"
const uint16_t cGoalPositionAddr = 116;
const uint16_t cGoalPositionLen  = 4;

// Address and length control table item "Present Position"
const uint16_t cPresentPositionAddr = 132;
const uint16_t cPresentPositionLen  = 4;

// Address and length control table item "Torque Enable"
const uint16_t cTorqueEnableAddr = 64;
const uint16_t cTorqueEnableLen  = 1;

// Address and length control table item "Goal PWM"
const uint16_t cGoalPWMAddr = 100;
const uint16_t cGoalPWMLen  = 2;

// ----------------------------------------------------------------------------------------
ServoBus::ServoBus(int dirPin, int rxPin, int txPin)
: myDXL(DXL_SERIAL, dirPin),
  myRXPin(rxPin), myTXPin(txPin),
  myAllServosAreConfigured(false)
{}

// ----------------------------------------------------------------------------------------
bool ServoBus::begin()
{
  for (int i=0; i<cNumServos; i++)
  {
    if (myServos[i].isConfigured == false)
    {
      Serial << __PRETTY_FUNCTION__ << " -> not all servos are configured, found idx " << i << endl;
      return false;
    }
  }

  // Order matters here and is not a mistake. Dynamixel2Arduino::begin() calls begin(baud) on
  // the underlying HardwareSerial itself, which brings the port up on Serial1's *default*
  // pins. Re-opening it afterwards with explicit pin arguments is what actually binds the bus
  // to cDXL_RX/cDXL_TX. 
  myDXL.begin(cBaud);
  DXL_SERIAL.begin(cBaud, SERIAL_8N1, myRXPin, myTXPin);
  myDXL.setPortProtocolVersion(cDXLProtocolVersion);
  myDXL.torqueOff(DXL_BROADCAST_ID);

  // Check if all servos are responding to a ping request
  for (int i=0; i<cNumServos; i++)
  {
    if (myDXL.ping(myServos[i].id) == false)
    {
      Serial << "ERROR: " << __PRETTY_FUNCTION__ 
                          << " -> ping failed for servo " << myServos[i].id 
                          << " error code " << myDXL.getLastLibErrCode()
                          << endl;
      return false;
    }
  }

  // Configure various servo parameters. Every write is checked
  for (int i=0; i<cNumServos; i++)
  {
    myDXL.torqueOff(myServos[i].id); // required before changing operating mode

    bool ok = true;

    // ---- EEPROM items (control table address < 64). These persist across power cycles, so
    // they are only written when they differ - see writeEEPROMItemIfChanged().

    // OPERATING_MODE is EEPROM too, but go through the library's setOperatingMode() rather
    // than a raw register write, since that also validates the mode against the servo model.
    const int32_t mode = myDXL.readControlTableItem(OPERATING_MODE, myServos[i].id);
    if (myDXL.getLastLibErrCode() != DXL_LIB_OK || mode != OP_POSITION)
      ok &= myDXL.setOperatingMode(myServos[i].id, OP_POSITION);

    ok &= writeEEPROMItemIfChanged(RETURN_DELAY_TIME,  myServos[i].id, 10); // 10 * 2 us = 20 us, for faster syncReads
    ok &= writeEEPROMItemIfChanged(MIN_POSITION_LIMIT, myServos[i].id, myServos[i].minPos);
    ok &= writeEEPROMItemIfChanged(MAX_POSITION_LIMIT, myServos[i].id, myServos[i].maxPos);
    ok &= writeEEPROMItemIfChanged(MIN_VOLTAGE_LIMIT, myServos[i].id, cMinVoltageLimit);

    // Pin the PWM ceiling so the raw Goal PWM values used elsewhere mean a known percentage.
    ok &= writeEEPROMItemIfChanged(PWM_LIMIT, myServos[i].id, cPWMLimitRaw);

    // ---- RAM items: reset to defaults on every power-up, so these must be written each boot.
    ok &= myDXL.writeControlTableItem(PROFILE_ACCELERATION, myServos[i].id, 0); // infinite acceleration time(‘0 [msec]’)
    ok &= myDXL.writeControlTableItem(PROFILE_VELOCITY, myServos[i].id, 0);     // ‘0’ represents an infinite velocity (max speed)

    ok &= myDXL.writeControlTableItem(POSITION_P_GAIN, myServos[i].id, 400); // default 700, tested with 400
    ok &= myDXL.writeControlTableItem(POSITION_I_GAIN, myServos[i].id, 0);   // default 0
    ok &= myDXL.writeControlTableItem(POSITION_D_GAIN, myServos[i].id, 0);   // default 0

    ok &= myDXL.writeControlTableItem(GOAL_PWM, myServos[i].id, cSafePWMRaw);

    if (ok == false)
    {
      Serial << "ERROR: " << __PRETTY_FUNCTION__ 
             << " -> failed to configure servo " << myServos[i].id << endl;
      return false;
    }

    myDXL.ledOff(myServos[i].id);

    // Deliberately no torqueOn() here - see the class-level comment in ServoBus.h for
    // why. Torque is enabled later by the caller, only after it has pushed a goal
    // position matching the servo's actual current position.
  }

  // Fill the members of structure to syncWrite using internal packet buffer
  mySyncWriteInfos.packet.p_buf        = nullptr;
  mySyncWriteInfos.packet.is_completed = false;
  mySyncWriteInfos.addr                = cGoalPositionAddr;
  mySyncWriteInfos.addr_length         = cGoalPositionLen;
  mySyncWriteInfos.p_xels              = mySyncWriteXels;
  mySyncWriteInfos.xel_count           = cNumServos;

  for (int i = 0; i < cNumServos; i++)
  {
    mySyncWriteXels[i].id     = myServos[i].id;
    mySyncWriteXels[i].p_data = (uint8_t*)&mySyncWriteData[i].goal_position;
  } 

  // Fill the members of structure to fastSyncRead using internal user packet buffer
  mySyncReadInfos.packet.p_buf        = nullptr;
  mySyncReadInfos.packet.is_completed = false;
  mySyncReadInfos.addr                = cPresentPositionAddr;
  mySyncReadInfos.addr_length         = cPresentPositionLen;
  mySyncReadInfos.p_xels              = mySyncReadXels;
  mySyncReadInfos.xel_count           = cNumServos;  

  for (int i = 0; i < cNumServos; i++)
  {
    mySyncReadXels[i].id = myServos[i].id;
    mySyncReadXels[i].p_recv_buf = (uint8_t*)&mySyncReadData[i];
  }

  mySyncReadInfos.is_info_changed = true;

  // Fill the members of structure to syncRead the Torque Enable item, used by isAllTorqueOn()
  myTorqueReadInfos.packet.p_buf        = nullptr;
  myTorqueReadInfos.packet.is_completed = false;
  myTorqueReadInfos.addr                = cTorqueEnableAddr;
  myTorqueReadInfos.addr_length         = cTorqueEnableLen;
  myTorqueReadInfos.p_xels              = myTorqueReadXels;
  myTorqueReadInfos.xel_count           = cNumServos;

  for (int i = 0; i < cNumServos; i++)
  {
    myTorqueReadXels[i].id = myServos[i].id;
    myTorqueReadXels[i].p_recv_buf = (uint8_t*)&myTorqueReadData[i];
  }

  myTorqueReadInfos.is_info_changed = true;

  // Fill the members of structure to syncWrite the Goal PWM item, used by setTorque()
  myPWMWriteInfos.packet.p_buf        = nullptr;
  myPWMWriteInfos.packet.is_completed = false;
  myPWMWriteInfos.addr                = cGoalPWMAddr;
  myPWMWriteInfos.addr_length         = cGoalPWMLen;
  myPWMWriteInfos.p_xels              = myPWMWriteXels;
  myPWMWriteInfos.xel_count           = cNumServos;

  for (int i = 0; i < cNumServos; i++)
  {
    myPWMWriteXels[i].id     = myServos[i].id;
    myPWMWriteXels[i].p_data = (uint8_t*)&myPWMWriteData[i].goal_pwm;
  }

  myAllServosAreConfigured = true;

  return true;
}

// ----------------------------------------------------------------------------------------
void ServoBus::configureServo(int index, uint8_t id, int32_t minPos, int32_t maxPos) 
{
  if (isValidIndex(index) == false)
    return;

  // Catch copy-paste mistakes where two indices end up with the same physical servo ID -
  // syncWrite/syncRead would otherwise silently send two different goal positions to (or
  // read the same position twice from) one physical servo, which is a confusing thing to
  // debug on a live robot.
  for (int i = 0; i < cNumServos; i++)
  {
    if (i != index && myServos[i].isConfigured == true && myServos[i].id == id)
    {
      Serial << __PRETTY_FUNCTION__ << " -> id " << id << " already used by index " << i 
             << " -> ignoring configuration for index " << index << endl;
      return;
    }
  }

  myServos[index].isConfigured = true;
  myServos[index].id           = id;
  myServos[index].minPos       = minPos;
  myServos[index].maxPos       = maxPos;
}

// ----------------------------------------------------------------------------------------
void ServoBus::setTorque(int16_t goalPWMRaw)
{
  if (myAllServosAreConfigured == false)
    return;

  for (int i=0; i<cNumServos; i++)
    myPWMWriteData[i].goal_pwm = goalPWMRaw;

  myPWMWriteInfos.is_info_changed = true;

  if (myDXL.syncWrite(&myPWMWriteInfos) == false)
  {
    Serial << __PRETTY_FUNCTION__ << " -> Goal PWM syncWrite failed, lib error code: "
           << myDXL.getLastLibErrCode() << endl;
  }

  // Broadcast write: one packet, no status replies - the same pattern setTorqueOff() uses.
  myDXL.torqueOn(DXL_BROADCAST_ID);
}

// ----------------------------------------------------------------------------------------
void ServoBus::setMaxTorque()
{
  setTorque(cMaxPWMRaw);
}

// ----------------------------------------------------------------------------------------
void ServoBus::setSafeTorque()
{
  setTorque(cSafePWMRaw);
}

// ----------------------------------------------------------------------------------------
void ServoBus::setTorqueOff()
{
  if (myAllServosAreConfigured == false)
    return;

  myDXL.torqueOff(DXL_BROADCAST_ID);
}

// ----------------------------------------------------------------------------------------
bool ServoBus::isAllTorqueOn()
{
  if (myAllServosAreConfigured == false)
    return false;

  // is_info_changed must be set on every call: this descriptor shares the library's internal
  // packet buffer with the present-position read, so a packet left there by that descriptor
  // would otherwise be re-sent instead of this one.
  myTorqueReadInfos.is_info_changed = true;

  const uint8_t recvCnt = myDXL.syncRead(&myTorqueReadInfos);

  // A servo that does not answer cannot be confirmed as torqued on, and "off" is the safe
  // answer: the caller then re-reads present positions and re-seeds the goals instead of
  // assuming the leg is holding where it thinks it is.
  if (recvCnt != cNumServos)
    return false;

  for (int i=0; i<cNumServos; i++)
  {
    // In Dynamixel Protocol, 1 = Enabled, 0 = Disabled
    if (myTorqueReadData[i].torque_enable != 1)
      return false;
  }

  return true;
}

// ----------------------------------------------------------------------------------------
void ServoBus::setGoalPosition(int index, int32_t goalPos)
{
  if (myAllServosAreConfigured == false)
    return;

  if (isValidIndex(index) == false)
    return;

  // Clamp rather than reject. Discarding the command left this joint at its previous goal while
  // the other two joints of the leg moved on - a kinematically inconsistent pose, held at
  // whatever PWM setTorque() last applied.
  const int32_t clamped = std::clamp(goalPos, myServos[index].minPos, myServos[index].maxPos);

  if (clamped != goalPos)
  {
    ++myClampedGoals;

    const int32_t overshoot = goalPos - clamped;
    if (abs(overshoot) > abs(myWorstGoalOvershoot))
    {
      myWorstGoalOvershoot = overshoot;
      myWorstGoalServo     = index;
    }
  }

  // A deadband here (skipping goals within a few ticks of the current one) was measured and
  // showed no benefit.
  myServos[index].goalPos = clamped;
}

// ----------------------------------------------------------------------------------------
ServoBus::Diagnostics ServoBus::fetchDiagnostics()
{
  Diagnostics d;

  d.clampedGoals   = myClampedGoals.exchange(0);
  d.syncWriteFails = mySyncWriteFails.exchange(0);

  d.worstGoalOvershoot = myWorstGoalOvershoot;
  d.worstGoalServo     = myWorstGoalServo;

  myWorstGoalOvershoot = 0;
  myWorstGoalServo     = -1;

  return d;
}

// ----------------------------------------------------------------------------------------
void ServoBus::setGoalPositionRad(int index, float rad)
{
  setGoalPosition(index, radToTick(rad));
}

// ----------------------------------------------------------------------------------------
uint8_t ServoBus::id(int index) const
{
  if (isValidIndex(index) == false)
    return 0;

  return myServos[index].id;
}


// ----------------------------------------------------------------------------------------
int32_t ServoBus::presentPosition(int index) const
{
  if (isValidIndex(index) == false)
    return 0;

  return myServos[index].presentPos;
}

// ----------------------------------------------------------------------------------------
int32_t ServoBus::currentGoal(int index) const
{
  if (isValidIndex(index) == false)
    return 0;

  return myServos[index].goalPos;
}

// ----------------------------------------------------------------------------------------
bool ServoBus::syncWrite()
{
  if (myAllServosAreConfigured == false)
    return false;

  for (int i=0; i<cNumServos; i++)
  {
    mySyncWriteData[i].goal_position = myServos[i].goalPos;
  }

  mySyncWriteInfos.is_info_changed = true;

  if (myDXL.syncWrite(&mySyncWriteInfos) == false)
  {
    // Counted, not logged: called once per control tick, so a failing bus would otherwise
    // produce 200 blocking log lines per second.
    ++mySyncWriteFails;
    return false;
  }
  
  return true;
}

// ----------------------------------------------------------------------------------------
bool ServoBus::syncReadPresentPosition()
{
  if (myAllServosAreConfigured == false)
    return false;

  // Rebuild the instruction packet on every call. This descriptor shares the library's internal
  // packet buffer with the Torque Enable read in isAllTorqueOn(), so the packet left in that
  // buffer by the other descriptor must not be reused.
  mySyncReadInfos.is_info_changed = true;

  uint8_t recv_cnt = myDXL.syncRead(&mySyncReadInfos);

  if (recv_cnt == 0)
  {
    Serial << __PRETTY_FUNCTION__ << " -> dxl.syncRead failed, lib error code: " << myDXL.getLastLibErrCode() << endl;
    return false;
  }
  else if (recv_cnt < cNumServos)
  {
    Serial << __PRETTY_FUNCTION__ << " -> dxl.syncRead failed, recv_cnt < cNumServos: " << recv_cnt << endl;
    return false;
  }

  for (int i=0; i<cNumServos; i++)
  {
    myServos[i].presentPos = mySyncReadData[i].present_position;
  }

  return true;
}

// ----------------------------------------------------------------------------------------
bool ServoBus::isValidIndex(int index) const
{
  if (index < 0 || index >= cNumServos)
  {
    Serial << __PRETTY_FUNCTION__ << " -> index out of range: " << index << endl;
    return false;
  }

  return true;
}

// ----------------------------------------------------------------------------------------
bool ServoBus::writeEEPROMItemIfChanged(uint8_t item, uint8_t id, int32_t value)
{
  // Only skip the write if the read definitely succeeded: readControlTableItem() returns 0
  // on failure, which would otherwise look like "already 0" for a value of 0.
  const int32_t current = myDXL.readControlTableItem(item, id);
  if (myDXL.getLastLibErrCode() == DXL_LIB_OK && current == value)
    return true;

  return myDXL.writeControlTableItem(item, id, value);
}

