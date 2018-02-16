
#ifndef GRIZZLY_MOTOR_DRIVER_FRAME_H
#define GRIZZLY_MOTOR_DRIVER_FRAME_H

#include <stdint.h>

namespace grizzly_motor_driver
{

struct Frame
{
  uint32_t id;
  uint8_t len;

  union data_t
  {
    struct
    {
      uint8_t destination_id;
      uint16_t destination_reg;
      uint8_t action;
      uint32_t value;
    } formed;
    uint8_t raw[8];
  } data;

  explicit Frame(uint32_t id = 0) : id(id), len(0)
  {
  }

  uint32_t getCanId() const
  {
    return ((id & 0x07F8) | (id & 0x07));
  }
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_FRAME_H
