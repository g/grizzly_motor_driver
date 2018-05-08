
#ifndef GRIZZLY_MOTOR_DRIVER_NODE_H
#define GRIZZLY_MOTOR_DRIVER_NODE_H

#include <stdint.h>
#include <string>
#include <ros/ros.h>

#include "grizzly_motor_driver/driver.h"
#include "grizzly_motor_msgs/MultiStatus.h"
#include "grizzly_motor_msgs/Status.h"
#include "grizzly_motor_msgs/MultiFeedback.h"
#include "grizzly_motor_msgs/Feedback.h"

namespace grizzly_motor_driver
{
class Node
{
public:
  Node(ros::NodeHandle& nh, std::vector<std::shared_ptr<grizzly_motor_driver::Driver>> drivers);

  void publishFeedback();
  void publishStatus();
  void feedbackTimerCb(const ros::TimerEvent&);
  void statusTimerCb(const ros::TimerEvent&);

private:
  ros::NodeHandle nh_;
  std::vector<std::shared_ptr<grizzly_motor_driver::Driver>> drivers_;

  grizzly_motor_msgs::MultiStatus status_msg_;
  grizzly_motor_msgs::MultiFeedback feedback_msg_;

  ros::Publisher status_pub_;
  ros::Publisher feedback_pub_;

  ros::Timer status_pub_timer_;
  ros::Timer feedback_pub_timer_;

  bool active_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_NODE_H
