
#include "grizzly_motor_driver/registers.h"

#define ADD_ERR_REG(id, name) registers_[id] = std::make_shared<ErrorRegister>(name);
#define ADD_DATA_REG(id, name, min, max, initial, scale) registers_[id]=std::make_shared<DataRegister>(name, min, max, initial, scale)

namespace grizzly_motor_driver
{

Registers::Registers()
{
  ADD_DATA_REG(CK_CITY_ADDRESS, "CK_CITY_ADDRESS", 1, 127, 5, 1);
}

}  // namespace grizzly_motor_driver
