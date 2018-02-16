
#include <string>
#include <ros/ros.h>

#include "grizzly_motor_driver/driver.h"
#include "grizzly_motor_driver/interface.h"

namespace grizzly_motor_driver
{

Driver::Driver(const Interface &interface, const uint8_t can_id, const std::string &name) :
  interface_(interface),
  can_id_(can_id),
  name_(name)
{
}

void Driver::run()
{

}

}  // namespace grizzly_motor_driver
