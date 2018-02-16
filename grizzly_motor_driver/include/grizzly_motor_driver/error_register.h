
#ifndef GRIZZLY_MOTOR_DRIVER_ERROR_REGISTER_H
#define GRIZZLY_MOTOR_DRIVER_ERROR_REGISTER_H

#include <stdint.h>

namespace grizzly_motor_driver
{

class ErrorRegister: public Register
{
public:
  ErrorRegister(uint16_t id, float min, float min_override = 0.0, float max, float max_override = 0.0,
    float initial, float scale_);

private:
  float min_;
  float min_override_;
  float max_;
  float max_override_;
  float default_;
  float scale_;
  float data_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_DATA_REGISTER_H
