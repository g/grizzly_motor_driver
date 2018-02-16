
#ifndef GRIZZLY_MOTOR_DRIVER_DRIVER_H
#define GRIZZLY_MOTOR_DRIVER_DRIVER_H

#include <stdint.h>
#include <string>

#include "grizzly_motor_driver/interface.h"

namespace grizzly_motor_driver
{

class Driver
{
public:
  Driver(const Interface &interface, const uint8_t can_id, const std::string &name);

  void run();

private:
  const Interface& interface_;
  const uint8_t can_id_;
  const std::string name_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_DRIVER_H
