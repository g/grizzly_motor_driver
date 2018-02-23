
#include <ros/ros.h>

#include "grizzly_motor_driver/driver.h"

namespace grizzly_motor_driver
{

Driver::Driver(Interface &interface, const uint8_t can_id, const std::string &name) :
  interface_(interface),
  can_id_(can_id),
  name_(name),
  state_(START_UP)
{
}

void Driver::run()
{
  switch (state_)
  {
    case: START_UP_READ:
      readRegister(START_UP_ERRORS);
      state_ = START_UP_VERIFY;
      break;
    case: START_UP_VERIFY:
      
  }
}

void Driver::readFrame(const Frame &frame)
{
  if (frame.getCanId() != can_id_)
  {
    ROS_INFO("Frame is not for me.");
    return;
  }

  if (frame.len == 0)
  {
    ROS_INFO("Frame has no data.");
    return;
  }
  ROS_INFO("CAN ID 0x%08x", frame.id);
  ROS_INFO("DEVICE ID 0x%08x", frame.getCanId());
  ROS_INFO("Dest ID 0x%02x", frame.data.dest_id);
  ROS_INFO("Dest Reg 0x%04x", frame.data.dest_reg);
  ROS_INFO("Action 0x%02x", frame.data.action);
  ROS_INFO("Value 0x%08x", frame.data.value);
}

void Driver::readRegister(uint16_t id)
{
  Frame tx_frame;
  tx_frame.id = 0x00000060;
  tx_frame.len = 8;
  tx_frame.data.dest_id = can_id_;
  tx_frame.data.dest_reg = id;
  tx_frame.data.action = can_id_ + READ;
  tx_frame.data.value = 0;
  interface_.queue(tx_frame);
}

void Driver::writeRegister(uint16_t id, float value)
{
  Frame tx_frame;
  tx_frame.id = 0x00000060;
  tx_frame.len = 8;
  tx_frame.data.dest_id = can_id_;
  tx_frame.data.dest_reg = id;
  tx_frame.data.action = can_id_ + WRITE;
  tx_frame.data.value = value;
  interface_.queue(tx_frame);
}

}  // namespace grizzly_motor_driver
