
#ifndef GRIZZLY_MOTOR_DRIVER_ERROR_REGISTER_H
#define GRIZZLY_MOTOR_DRIVER_ERROR_REGISTER_H

#include <stdint.h>

#include "grizzly_motor_driver/register.h"

namespace grizzly_motor_driver
{

class ErrorRegister : public Register
{
public:
  ErrorRegister(const std::string &name){}

private:
  std::string name_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_ERROR_REGISTER_H
