
#include <string>
#include <vector>

#include "boost/foreach.hpp"
#include "boost/scoped_ptr.hpp"
#include "boost/shared_ptr.hpp"

#include "ros/ros.h"

#include "grizzly_motor_driver/driver.h"
#include "grizzly_motor_driver/frame.h"
#include "grizzly_motor_driver/interface.h"

class Node
{
public:
  Node(ros::NodeHandle& nh,
       ros::NodeHandle& pnh,
       grizzly_motor_driver::Interface& interface) :
    nh_(nh),
    pnh_(pnh),
    interface_(interface),
    freq_(10)
  {
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
      interface_.sendQueued();
      ros::spinOnce();
      interface_.sendQueued();

      grizzly_motor_driver::Frame recv_frame;
      while (interface_.receive(&recv_frame))
      {
        ROS_INFO("Reading");
      }
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_, pnh_;
  grizzly_motor_driver::Interface& interface_;
  std::vector<grizzly_motor_driver::Driver> drivers_;

  int freq_;
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
    return -1;
  }

  grizzly_motor_driver::Interface interface(can_device);

  Node n(nh, pnh, interface);
  n.run();
}
