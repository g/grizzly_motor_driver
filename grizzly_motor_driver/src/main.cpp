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

#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "boost/assign.hpp"

#include "grizzly_motor_driver/driver.h"
#include "grizzly_motor_driver/frame.h"
#include "grizzly_motor_driver/interface.h"
#include "grizzly_motor_driver/node.h"

class TestInterface
{
public:
  TestInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, grizzly_motor_driver::Interface& interface)
    : nh_(nh), pnh_(pnh), interface_(interface), freq_(25), status_divisor_(3), active_(false)
  {
    ros::V_string joint_names =
        boost::assign::list_of("front_left_wheel")("front_right_wheel")("rear_left_wheel")("rear_right_wheel");
    std::vector<uint8_t> joint_canids = boost::assign::list_of(5)(4)(2)(3);
    std::vector<float> joint_directions = boost::assign::list_of(-1)(1)(-1)(1);
    drivers_.push_back(
        std::shared_ptr<grizzly_motor_driver::Driver>(new grizzly_motor_driver::Driver(interface_, 3, "test1")));
    drivers_.push_back(
        std::shared_ptr<grizzly_motor_driver::Driver>(new grizzly_motor_driver::Driver(interface_, 4, "test2")));
    drivers_.push_back(
        std::shared_ptr<grizzly_motor_driver::Driver>(new grizzly_motor_driver::Driver(interface_, 5, "test3")));
    drivers_.push_back(
        std::shared_ptr<grizzly_motor_driver::Driver>(new grizzly_motor_driver::Driver(interface_, 6, "test4")));

    node_.reset(new grizzly_motor_driver::Node(nh_, drivers_));
    for (const auto& driver : drivers_)
    {
      driver->setGearRatio(25.0);
    }
    velocity_sub = pnh.subscribe("test_speed", 1, &TestInterface::velocityCB, this);
  }

  bool connectIfNotConnected()
  {
    if (!interface_.isConnected())
    {
      if (!interface_.connect())
      {
        ROS_ERROR("Error connecting to motor driver interface. Retrying in 1 second.");
        return false;
      }
      else
      {
        ROS_INFO("Connection to motor driver interface successful.");
      }
    }
    return true;
  }

  void run()
  {
    ros::Rate rate(freq_);

    while (ros::ok())
    {
      if (!connectIfNotConnected())
      {
        ros::Duration(1.0).sleep();
        continue;
      }

      for (auto& driver : drivers_)
      {
        driver->run();
        if (status_divisor_ == driver->getId() && isActive())
        {
          ROS_INFO("Req. %d", status_divisor_);
          driver->requestStatus();
          driver->requestFeedback();
        }
        if (isActive())
          driver->commandSpeed();
      }
      status_divisor_++;
      if (status_divisor_ > 6)  // first address is 3
        status_divisor_ = 3;

      interface_.sendQueued();
      ros::spinOnce();
      interface_.sendQueued();

      grizzly_motor_driver::Frame rx_frame;
      while (interface_.receive(&rx_frame))
      {
        for (auto& driver : drivers_)
        {
          driver->readFrame(rx_frame);
        }
      }
      rate.sleep();
    }
  }

  void velocityCB(const std_msgs::Float64ConstPtr& msg)
  {
    for (std::shared_ptr<grizzly_motor_driver::Driver>& driver : drivers_)
    {
      driver->setSpeed(msg->data);
    }
  }

  bool isActive()
  {
    for (const auto& driver : drivers_)
    {
      if (!driver->isRunning())
      {
        active_ = false;
        return false;
      }
    }
    active_ = true;
    return true;
  }

private:
  ros::NodeHandle nh_, pnh_;
  grizzly_motor_driver::Interface& interface_;
  std::vector<std::shared_ptr<grizzly_motor_driver::Driver>> drivers_;
  std::shared_ptr<grizzly_motor_driver::Node> node_;
  double freq_;
  uint8_t status_divisor_;
  bool active_;
  // temp sub for velocity testing
  ros::Subscriber velocity_sub;
};

int main(int argc, char* argv[])
{
  std::string node_name = "node";
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle nh, pnh("~");

  std::string can_device;
  if (!pnh.getParam("can_device", can_device))
  {
    ROS_FATAL("%s: No CAN device given.", node_name.c_str());
    ros::shutdown();
    return -1;
  }

  grizzly_motor_driver::Interface interface(can_device);

  TestInterface n(nh, pnh, interface);
  n.run();
}
