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
#ifndef GRIZZLY_MOTOR_DRIVER_DRIVER_H
#define GRIZZLY_MOTOR_DRIVER_DRIVER_H

#include <math.h>
#include <memory>
#include <stdint.h>
#include <string>
#include <atomic>

#include "grizzly_motor_driver/registers.h"
#include "grizzly_motor_driver/interface.h"

namespace grizzly_motor_driver
{
class Driver
{
public:
  Driver(Interface& interface, const uint8_t can_id, const std::string& name);

  void configure();
  void run();
  void setGearRatio(double ratio);
  void setSpeed(float cmd);
  void requestFeedback();
  void requestStatus();

  void readFrame(const Frame& frame);

  void requestRegister(uint16_t id);
  void writeRegister(uint16_t id);
  void writeRegister(uint16_t id, float value);

  bool isRunning() const;
  bool isFault() const;
  bool isStopping() const;
  std::string getName() const;
  uint8_t getId() const;
  float getHeading() const;
  float getMeasuredVelocity() const;
  float getMeasuredTravel() const;
  uint16_t getRuntimeErrors() const;
  uint16_t getStartupErrors() const;
  float getMotorTemp() const;
  float getDriverTemp() const;
  float getInputVoltage() const;
  float getOutputVoltage() const;
  float getOutputCurrent() const;
  float getSpeed() const;
  bool commandSpeed();
  void resetState();
  void setStopping();
  void isConnected();


private:
  Interface& interface_;
  const uint8_t can_id_;
  const std::string name_;

  uint8_t state_;
  bool configured_;
  uint16_t configuration_state_;
  uint8_t lost_messages_;
  double gear_ratio_;
  std::atomic<float> speed_;

  std::shared_ptr<Registers> registers_;
};

}  // namespace grizzly_motor_driver

#endif  // GRIZZLY_MOTOR_DRIVER_DRIVER_H
