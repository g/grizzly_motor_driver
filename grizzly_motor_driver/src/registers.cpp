
#include "grizzly_motor_driver/registers.h"

#define ADD_REG(id, name, initial, min, max, scale) registers_[id]=std::make_shared<Register>(name, initial, min, max, scale)

namespace grizzly_motor_driver
{

Registers::Registers() :
  io_scan_time_millis_(20),
  system_voltage_(48),
  max_speed_forward_(100),
  max_speed_reverse_(100),
  accel_time_millis_(1000),
  decel_time_millis_(250)
{
  ADD_REG(Registry::StartUpErrors, "START_UP_ERRORS", 0, 0, 65536, 1);
  ADD_REG(Registry::RunTimeErrors, "RUMTIME_ERRORS", 0, 0, 65536, 1);
  //ADD_REG(Registry::CkCityAddress, "CK_CITY_ADDRESS", 5, 1, 127, 1);
  ADD_REG(Registry::PreChargeVoltage, "PRE_CHARGE_VOLTAGE", 32, 48, 32, 1);
  ADD_REG(Registry::SystemVoltage, "SYSTEM_VOLTAGE", 48, 24, 48, 1);
  ADD_REG(Registry::AbsoluteCurrent, "ABSOLUTE_CURRENT", 300, 40, 450, 1);
  ADD_REG(Registry::PeakCurrent, "PEAK_VOLTAGE", 300, 40, 300, 1);
  ADD_REG(Registry::RatedCurrent, "RATED_CURRENT", 110, 20, 200, 1);
  ADD_REG(Registry::RegenCurrent, "REGEN_CURRENT", 150, 20, 300, 1);
  ADD_REG(Registry::CoastCurrent, "COAST_CURRENT", 75, 20, 300, 1);
  ADD_REG(Registry::MaxCurrentAllowTime, "MAX_CURRENT_ALLOW_TIME", 30, 0, 260, 1.024);
  ADD_REG(Registry::MaxCurrentResetTime, "MAX_CURRENT_RESET_TIME", 60, 0, 260, 1.024);
  ADD_REG(Registry::MaxSpeedForward, "MAX_SPEED_FORWARD", max_speed_forward_, 0, 100, 0.8);
  ADD_REG(Registry::MaxSpeedReverse, "MAX_SPEED_REVERSE", max_speed_reverse_, 0, 100, 0.8);
  ADD_REG(Registry::AccelForward, "ACCEL_FORWARD", 80, 10, 1000, 1);
  ADD_REG(Registry::DecelForward, "DECEL_FORWARD", 320, 10, 1000, 1);
  ADD_REG(Registry::AccelReverse, "ACCEL_REVERSE", 80, 10, 1000, 1);
  ADD_REG(Registry::DecelReverse, "DECEL_REVERSE", 320, 10, 1000, 1);
  ADD_REG(Registry::SroDebounceTime, "SRO_DEBOUCE_TIME", 250, 2000, 32, io_scan_time_millis_);
  ADD_REG(Registry::AnalogOutFunction, "ANALOG_OUT_FUNCTION", 0, 0, 5, 1);
  ADD_REG(Registry::AnalogOutVoltage, "ANALOG_OUT_VOLTAGE", 12, 21, 48, 1);
  ADD_REG(Registry::NeutralToStopTime, "NEUTRAL_TO_STOP_TIME", 60, 0.002, 60, io_scan_time_millis_ / 1000.0);
  ADD_REG(Registry::IOScanTime, "I_O_SCAN_TIME", io_scan_time_millis_, 2, 40, 1);
  ADD_REG(Registry::SafeSeqEnable, "SAFE_SEQ_ENABLE", 0, 0, 1, 1);
  ADD_REG(Registry::IrComp, "IR_COMP", 10, 0, 100, 1);
  ADD_REG(Registry::MotorResistance, "MOTOR_RESISTANCE", 10.24, 10, 255, 1.024);
  ADD_REG(Registry::IOActive, "I_O_ACTIVE", 0, 0, 255, 1);
  ADD_REG(Registry::HeartBeatEnabled, "HEART_BEAT_ENABLE", 1, 0, 1, 1);
  ADD_REG(Registry::HeartBeatTimer, "HEART_BEAT_TIMER", 100, 2, 16000, 1);
  ADD_REG(Registry::StatusUpdateEnable, "STATUS_UPDATE_ENABLE", 1, 0, 1, 1);
  ADD_REG(Registry::StatusUpdateTimer, "STATUS_UPDATE_TIMER", 5, 1, 16000, 1);
  ADD_REG(Registry::HeadingAckEnable, "HEADING_ACK_ENABLE", 1, 0, 1, 1);
  ADD_REG(Registry::HeadingNetSelect, "HEADING_NET_SELECT", 170, 0, 170, 1);
  ADD_REG(Registry::IsoEnable, "ISO_ENABLE", 4, 0, 7, 1);
  ADD_REG(Registry::Pot1BrokenWireDetect, "POT_1_BROKEN_WIRE_DETECT", 0, 0, 1, 1);

  for (const auto &it : registers_)
  {
    writeable_ids_.push_back(it.first);
  }

  ADD_REG(Registry::Heading, "HEADING", 0, -100, 100, 0.025);
  ADD_REG(Registry::SroSw, "SRO_SW", 0, 0, 1, 1);
  ADD_REG(Registry::TargetPwm, "TARGET_PWM", 0, -100, 100, 0.025);
  ADD_REG(Registry::ActualCurrent, "ACTUAL_CURRENT", 0, -450, 450, 1);
  ADD_REG(Registry::BatVoltage, "BAT_VOLTAGE", 0, 255, 0, 1);
  ADD_REG(Registry::Temperature, "TEMPERATURE", 0, 126, 32, 1);
  ADD_REG(Registry::MotVoltage, "MOT_VOLTAGE", 0, 255, -255, 0.1);
  ADD_REG(Registry::CurrentSensorZero, "CURRENT_SENSOR_ZERO", 0, 0, 511, 1);
  ADD_REG(Registry::BaseCharged, "BASE_CHARGED", 0, 0, 1, 1);
  ADD_REG(Registry::ControllerEnabled, "CONTROLLER_ENABLED", 0, 0, 1, 1);
  ADD_REG(Registry::StatusUpdate, "STATUS_UPDATE", 0, 0, 1, 1);
  ADD_REG(Registry::EnableKeySw, "ENABLE_KEY_SW", 0, 0, 1, 1);

  for (const auto &it : registers_)
  {
    ids_.push_back(it.first);
  }

}

std::shared_ptr<Register> Registers::getRegister(const uint16_t id)
{
  if (registers_.count(id))
  {
    return registers_[id];
  }
  std::cout << "Invalid ID " << id << " requested." << std::endl;
  return std::shared_ptr<Register>();
}

uint16_t Registers::getNumberOfWriteableIds() const
{
  return writeable_ids_.size();
}

std::vector<uint16_t> Registers::getIds() const
{
  return ids_;
}

uint16_t Registers::getId(uint16_t i) const
{
  return ids_[i];
}

uint16_t Registers::getWriteableId(uint16_t i) const
{
  return writeable_ids_[i];
}

}  // namespace grizzly_motor_driver
