/**
Software License Agreement (BSD)
\authors   Tony Baltovski <tbaltovski@clearpathrobotics.com>
\copyright Copyright (c) 2018, Clearpath Robotics, Inc., All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef GRIZZLY_MOTOR_DRIVER_REGISTERS_H
#define GRIZZLY_MOTOR_DRIVER_REGISTERS_H

#include <map>
#include <memory>
#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

#include "grizzly_motor_driver/register.h"

namespace grizzly_motor_driver
{
class Registers
{
public:
  Registers(uint8_t can_id);

  std::shared_ptr<Register> getRegister(const uint16_t id);
  uint16_t getNumberOfWriteableIds() const;
  std::vector<uint16_t> getIds() const;
  uint16_t getId(uint16_t i) const;
  uint16_t getWriteableId(uint16_t i) const;

private:
  std::map<uint16_t, std::shared_ptr<Register> > registers_;
  uint32_t total_registers_;
  std::vector<uint16_t> ids_;
  std::vector<uint16_t> writeable_ids_;

  float io_scan_time_millis_;
  float system_voltage_;
  float max_speed_forward_;
  float max_speed_reverse_;
  float accel_time_millis_;
  float decel_time_millis_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_REGISTERS_H
