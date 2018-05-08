
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
    struct __attribute__ ((packed))
    {
      uint8_t dest_id;
      uint16_t dest_reg;
      uint8_t action;
      int32_t value;
    };
    uint8_t raw[8];
  } data;

  explicit Frame(uint32_t id = 0) : id(id), len(8)
  {
  }

  uint32_t getCanId() const
  {
    // TODO(tbaltovski): FIX THIS!
    return (((id & 0xFFFFF7F8) | (id & 0x07)) >> 18);// & 0x00FF;
  }

};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_FRAME_H
