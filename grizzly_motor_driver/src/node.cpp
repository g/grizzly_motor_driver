#include <vector>

#include <ros/ros.h>

#include "grizzly_motor_driver/node.h"

namespace grizzly_motor_driver
{

Node::Node(ros::NodeHandle& nh, std::vector<std::shared_ptr<grizzly_motor_driver::Driver>>  drivers) :
  nh_(nh),
  drivers_(drivers)
{
  feedback_pub_ = nh_.advertise<grizzly_motor_msgs::MultiFeedback>("feedback", 1);
  status_pub_ = nh_.advertise<grizzly_motor_msgs::MultiStatus>("status", 1);

  feedback_msg_.feedbacks.resize(drivers_.size());
  status_msg_.statuses.resize(drivers_.size());

  feedback_pub_timer_ = nh_.createTimer(ros::Duration(1.0/25), &Node::feedbackTimerCb, this);
  status_pub_timer_ = nh_.createTimer(ros::Duration(1.0/1), &Node::statusTimerCb, this);
}

void Node::publishFeedback()
{
  uint8_t index = 0;
  for (const auto &driver : drivers_)
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
  for (const auto &driver : drivers_)
  {
    grizzly_motor_msgs::Status* status = &status_msg_.statuses[index];
    status->device_number = driver->getId();
    status->device_name = driver->getName();
    status->temperature_driver = driver->getDriverTemp();
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
