
#ifndef GRIZZLY_MOTOR_DRIVER_INTERFACE_H
#define GRIZZLY_MOTOR_DRIVER_INTERFACE_H

#include <fcntl.h>
#include <poll.h>
#include <string>
#include <stdio.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "grizzly_motor_driver/frame.h"

namespace grizzly_motor_driver
{
class Interface
{
public:
  Interface(const std::string &can_device);
  ~Interface();

  bool connect();
  bool disconnect();
  bool isConnected() const;
  bool connectIfNotConnected();

  bool receive(Frame *frame);
  void queue(const Frame &frame);
  void queue(const can_frame &send_frame);
  bool send(const can_frame *send_frame);
  bool sendQueued();

  void printCanFrame(const can_frame &frame);
  can_frame toCanFrame(const Frame &frame);
  Frame froCanFrame(const can_frame &frame);

private:
  std::string can_device_;
  int socket_;
  bool connected_;

  std::vector<can_frame> queue_inbound_;
  std::vector<can_frame> queue_outbound_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_INTERFACE_H
