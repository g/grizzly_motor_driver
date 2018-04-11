
#include <string>
#include <vector>

#include "ros/ros.h"

#include "grizzly_motor_driver/driver.h"
#include "grizzly_motor_driver/frame.h"
#include "grizzly_motor_driver/interface.h"
#include "grizzly_motor_driver/node.h"
#include "grizzly_motor_driver/diagnostic_updater.h"

class TestInterface
{
public:
  TestInterface(ros::NodeHandle& nh,
       ros::NodeHandle& pnh,
       grizzly_motor_driver::Interface& interface) :
    nh_(nh),
    pnh_(pnh),
    interface_(interface),
    freq_(25)
  {
    drivers_.push_back(grizzly_motor_driver::Driver(interface_, 5, "test"));

    node_.reset(new grizzly_motor_driver::Node(nh_, drivers_));
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

      for (grizzly_motor_driver::Driver& driver : drivers_)
      {
        driver.run();
        if (driver.isConfigured())
        {
          ROS_INFO("Req.");
          driver.requestStatus();
          driver.requestFeedback();
        }
      }

      interface_.sendQueued();
      ros::spinOnce();
      interface_.sendQueued();

      grizzly_motor_driver::Frame rx_frame;
      while (interface_.receive(&rx_frame))
      {
        for (grizzly_motor_driver::Driver& driver : drivers_)
        {
          driver.readFrame(rx_frame);
        }
      }
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_, pnh_;
  grizzly_motor_driver::Interface& interface_;
  std::vector<grizzly_motor_driver::Driver> drivers_;
  std::shared_ptr<grizzly_motor_driver::Node> node_;
  double freq_;
};


int main(int argc, char *argv[])
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
