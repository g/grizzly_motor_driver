
#ifndef GRIZZLY_MOTOR_DRIVER_REGISTERS_H
#define GRIZZLY_MOTOR_DRIVER_REGISTERS_H

#include <map>
#include <memory>
#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

#include "grizzly_motor_driver/register.h"

namespace grizzly_motor_driver
{

class Registers
{
public:
  Registers();

  std::shared_ptr<Register> getRegister(const uint16_t id);
  uint16_t getNumberOfWriteableIds() const;
  std::vector<uint16_t> getIds() const;
  uint16_t getId(uint16_t i) const;
  uint16_t getWriteableId(uint16_t i) const;
private:
  std::map<uint16_t, std::shared_ptr<Register> > registers_;
  uint32_t total_registers_;
  std::vector<uint16_t> ids_;
  std::vector<uint16_t> writeable_ids_;

  float io_scan_time_millis_;
  float system_voltage_;
  float max_speed_forward_;
  float max_speed_reverse_;
  float accel_time_millis_;
  float decel_time_millis_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_REGISTERS_H
