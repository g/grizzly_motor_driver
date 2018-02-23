
#ifndef GRIZZLY_MOTOR_DRIVER_REGISTERS_H
#define GRIZZLY_MOTOR_DRIVER_REGISTERS_H

#include <map>
#include <memory>
#include <stdint.h>
#include <vector>

#include "grizzly_motor_driver/data_register.h"
#include "grizzly_motor_driver/error_register.h"
#include "grizzly_motor_driver/register.h"
#include "grizzly_motor_driver/types.h"

namespace grizzly_motor_driver
{

class Registers
{
public:
  Registers();
  ~Registers();

private:
  std::map<uint16_t, std::shared_ptr<Register> > registers_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_REGISTERS_H
