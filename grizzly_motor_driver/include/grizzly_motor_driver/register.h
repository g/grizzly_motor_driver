
#ifndef GRIZZLY_MOTOR_DRIVER_REGISTER_H
#define GRIZZLY_MOTOR_DRIVER_REGISTER_H

#include <stdint.h>

namespace grizzly_motor_driver
{

class Register
{
public:
  void setId(uint16_t id);
  uint16_t getId() const;
protected:
  uint16_t id_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_REGISTER_H
