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
#ifndef GRIZZLY_MOTOR_DRIVER_INTERFACE_H
#define GRIZZLY_MOTOR_DRIVER_INTERFACE_H

#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>
#include <unistd.h>
#include <mutex>

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
  void sendQueued();

  void printCanFrame(const can_frame &frame);
  can_frame toCanFrame(const Frame &frame);
  Frame fromCanFrame(const can_frame &frame);

private:
  std::string can_device_;
  int socket_;
  bool connected_;

  std::mutex mutex_inbound_;
  std::mutex mutex_outbound_;

  std::vector<can_frame> queue_inbound_;
  std::vector<can_frame> queue_outbound_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_INTERFACE_H
