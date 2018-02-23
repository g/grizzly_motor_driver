
#ifndef GRIZZLY_MOTOR_DRIVER_DATA_REGISTER_H
#define GRIZZLY_MOTOR_DRIVER_DATA_REGISTER_H

#include <stdint.h>

#include "grizzly_motor_driver/register.h"

namespace grizzly_motor_driver
{

class DataRegister: public Register
{
public:
  DataRegister(const std::string &name, float min, float max, float value, float scale){}

private:
  float min_;
  float max_;
  float value_;
  float scale_;
  float data_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_DATA_REGISTER_H
