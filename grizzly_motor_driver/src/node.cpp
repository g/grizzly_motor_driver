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

#include <vector>

#include <ros/ros.h>

#include "grizzly_motor_driver/node.h"

namespace grizzly_motor_driver
{
Node::Node(ros::NodeHandle& nh, std::vector<std::shared_ptr<grizzly_motor_driver::Driver>> drivers)
  : nh_(nh), drivers_(drivers)
{
  feedback_pub_ = nh_.advertise<grizzly_motor_msgs::MultiFeedback>("feedback", 1);
  status_pub_ = nh_.advertise<grizzly_motor_msgs::MultiStatus>("status", 1);

  feedback_msg_.feedbacks.resize(drivers_.size());
  status_msg_.statuses.resize(drivers_.size());

  feedback_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / 25), &Node::feedbackTimerCb, this);
  status_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / 1), &Node::statusTimerCb, this);
}

void Node::publishFeedback()
{
  uint8_t index = 0;
  for (const auto& driver : drivers_)
  {
    grizzly_motor_msgs::Feedback* feedback = &feedback_msg_.feedbacks[index];
    feedback->device_number = driver->getId();
    feedback->device_name = driver->getName();
    feedback->heading = driver->getHeading();
    feedback->travel = driver->getMeasuredTravel();
    feedback->velocity = driver->getMeasuredVelocity();
    index++;
  }
  feedback_msg_.header.stamp = ros::Time::now();
  feedback_pub_.publish(feedback_msg_);
}

void Node::publishStatus()
{
  uint8_t index = 0;
  for (const auto& driver : drivers_)
  {
    grizzly_motor_msgs::Status* status = &status_msg_.statuses[index];
    status->device_number = driver->getId();
    status->device_name = driver->getName();
    status->temperature_driver = driver->getDriverTemp();
    status->temperature_motor = driver->getMotorTemp();
    status->voltage_input = driver->getInputVoltage();
    status->voltage_output = driver->getOutputVoltage();
    status->current = driver->getOutputCurrent();
    status->error_runtime = driver->getRuntimeErrors();
    status->error_startup = driver->getStartupErrors();
    index++;
  }
  status_msg_.header.stamp = ros::Time::now();
  status_pub_.publish(status_msg_);
}

void Node::statusTimerCb(const ros::TimerEvent&)
{
  publishStatus();
}

void Node::feedbackTimerCb(const ros::TimerEvent&)
{
  publishFeedback();
}

}  // namespace grizzly_motor_driver
