#include <string>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "grizzly_motor_driver/driver.h"
#include "grizzly_motor_driver/frame.h"
#include "grizzly_motor_driver/interface.h"
#include "grizzly_motor_driver/node.h"
#include "grizzly_motor_driver/diagnostic_updater.h"

class TestInterface
{
public:
  TestInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, grizzly_motor_driver::Interface& interface)
    : nh_(nh), pnh_(pnh), interface_(interface), freq_(200), status_divisor_(3), active_(false)
  {
    drivers_.push_back(
       std::shared_ptr<grizzly_motor_driver::Driver>(new grizzly_motor_driver::Driver(interface_, 3, "test1")));
    drivers_.push_back(
      std::shared_ptr<grizzly_motor_driver::Driver>(new grizzly_motor_driver::Driver(interface_, 4, "test2")));
    drivers_.push_back(
       std::shared_ptr<grizzly_motor_driver::Driver>(new grizzly_motor_driver::Driver(interface_, 5, "test3")));
    drivers_.push_back(
      std::shared_ptr<grizzly_motor_driver::Driver>(new grizzly_motor_driver::Driver(interface_, 6, "test4")));


    node_.reset(new grizzly_motor_driver::Node(nh_, drivers_));
    velocitySub = pnh.subscribe("test_speed", 1, &TestInterface::velocityCB, this);
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

      if (isActive())
      {
        for (std::shared_ptr<grizzly_motor_driver::Driver>& driver : drivers_)
        {
          driver->run();
          if (status_divisor_ == driver->getId())
          {
            ROS_INFO("Req. %d", status_divisor_);
            driver->requestStatus();
            //driver->requestFeedback();
          }
        }
        status_divisor_++;
        if (status_divisor_ > 6) //first address is 3
          status_divisor_ = 3;
      }

      interface_.sendQueued();
      ros::spinOnce();
      interface_.sendQueued();

      grizzly_motor_driver::Frame rx_frame;
      while (interface_.receive(&rx_frame))
      {
        for (std::shared_ptr<grizzly_motor_driver::Driver>& driver : drivers_)
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
    for (std::shared_ptr<grizzly_motor_driver::Driver>& driver : drivers_)
    {
      if (!driver->isConfigured())
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
  ros::Subscriber velocitySub;
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

  grizzly_motor_driver::GrizzlyMotorDriverDiagnosticUpdater grizzly_motor_driver_diagnostic_updater;
  TestInterface n(nh, pnh, interface);
  n.run();
}
