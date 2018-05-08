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

}  // namespace grizzly_motor_driver
