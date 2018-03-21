
#ifndef GRIZZLY_MOTOR_DRIVER_DRIVER_H
#define GRIZZLY_MOTOR_DRIVER_DRIVER_H

#include <math.h>
#include <memory>
#include <stdint.h>
#include <string>

#include "grizzly_motor_driver/registers.h"
#include "grizzly_motor_driver/interface.h"

namespace grizzly_motor_driver
{

class Driver
{
public:
  Driver(Interface &interface, const uint8_t can_id, const std::string &name);

  void configure();
  void run();

  void readFrame(const Frame &frame);

  void readRegister(uint16_t id);
  void writeRegister(uint16_t id, float value);

private:
  Interface& interface_;
  const uint8_t can_id_;
  const std::string name_;

  uint8_t state_;
  bool configured_;
  uint16_t configuration_state_;
  uint16_t total_configuration_states_;

  std::shared_ptr<Registers> registers_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_DRIVER_H