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

#include <limits>
#include <string>

#include "grizzly_motor_driver/register.h"

namespace grizzly_motor_driver
{

  Register::Register(const std::string &name, float initial, float min, float max, float scale) :
    name_(name),
    initial_(initial),
    min_(min),
    max_(max),
    scale_(scale),
    received_(false)
{
  raw_initial_ = static_cast<int32_t>(initial_ / scale_);
  data_ = std::numeric_limits<float>::quiet_NaN();
}

void Register::setReceived()
{
  received_ = true;
}

void Register::clearReceived()
{
  received_ = false;
}
bool Register::wasReceived() const
{
  return received_;
}

void Register::setData(float data)
{
  data_ = data * scale_;
  raw_data_ = static_cast<int32_t>(data_ / scale_);
}

float Register::sendData() const
{
  return data_ / scale_;
}

float Register::getData() const
{
  return data_;
}

float Register::sendInitial() const
{
  return initial_ / scale_;
}
void Register::setRawData(int32_t data)
{
  raw_data_ = data;
  data_ = data * scale_;
}

int32_t Register::getRawData() const
{
  return raw_data_;
}

float Register::getInitial() const
{
  return initial_;
}

int32_t Register::getRawInitial() const
{
  return raw_initial_;
}

float Register::getScale() const
{
  return scale_;
}

}  // namespace grizzly_motor_driver
