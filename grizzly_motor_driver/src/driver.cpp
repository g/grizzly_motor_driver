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

#include <ros/ros.h>

#include "grizzly_motor_driver/driver.h"

namespace grizzly_motor_driver
{
namespace States
{
enum State
{
  Start = 0,
  Configure,
  CheckStartUpErrors,
  VerifyStartUpErrors,
  CheckRunTimeErrors,
  VerifyRunTimeErrors,
  CheckHeading,
  VerifyHeading,
  CheckSro,
  Stopped,
  PreRunning,
  CheckEnable,
  VerifyEnable,
  Running,
  Fault,
  Stopping,
  NumberOfStates
};
}  // namespace States
typedef States::State State;

Driver::Driver(Interface& interface, const uint8_t can_id, const std::string& name)
  : interface_(interface)
  , can_id_(can_id)
  , name_(name)
  , state_(State::Start)
  , configured_(false)
  , registers_(std::shared_ptr<Registers>(new Registers(can_id)))
  , speed_(0)
{
  configuration_state_ = 0;
}

void Driver::configure()
{
  ROS_INFO_THROTTLE(1, "Driver %s (%i): Configuring", name_.c_str(), can_id_);
  uint16_t reg = registers_->getWriteableId(configuration_state_);

  if (configuration_state_ >= registers_->getNumberOfWriteableIds())
  {
    ROS_INFO("Driver %s (%i) was configured.", name_.c_str(), can_id_);
    configured_ = true;
    return;
  }

  if (registers_->getRegister(reg)->wasReceived())
  {
    if (registers_->getRegister(reg)->getRawData() == registers_->getRegister(reg)->getRawInitial())
    {
      ROS_DEBUG("Driver %s (%i): Match, got %d and wanted %d", name_.c_str(), can_id_,
          registers_->getRegister(reg)->getRawData(), registers_->getRegister(reg)->getRawInitial());
      configuration_state_++;
    }
    else
    {
      ROS_WARN("Driver %s (%i), trying to configure register %d failed, got %d and wanted %d. Retrying.", name_.c_str(),
          can_id_, reg, registers_->getRegister(reg)->getRawData(), registers_->getRegister(reg)->getRawInitial());
      writeRegister(reg, registers_->getRegister(reg)->sendInitial());
    }
    registers_->getRegister(reg)->clearReceived();
  }
  else
  {
    requestRegister(reg);
  }
}

void Driver::run()
{
  switch (state_)
  {
    case State::Start:
      writeRegister(Registry::EnableKeySw, 0);
      state_ = State::Configure;
      break;
    case State::Configure:
      if (configured_)
      {
        state_ = State::CheckStartUpErrors;
      }
      else
      {
        configure();
      }
      break;
    case State::CheckStartUpErrors:
      ROS_DEBUG("Driver %s (%i): State::CheckStartUpErrors", name_.c_str(), can_id_);
      requestRegister(Registry::StartUpErrors);
      state_ = State::VerifyStartUpErrors;
      break;
    case State::VerifyStartUpErrors:
      ROS_DEBUG("Driver %s (%i): State::VerifyStartUpErrors", name_.c_str(), can_id_);
      if (registers_->getRegister(Registry::StartUpErrors)->wasReceived())
      {
        if (registers_->getRegister(Registry::StartUpErrors)->getRawData() == 0)
        {
          state_ = State::CheckRunTimeErrors;
        }
        else
        {
          state_ = State::Fault;
          ROS_ERROR("Driver %s (%i): Start Up Error(s) %d", name_.c_str(), can_id_,
                    registers_->getRegister(Registry::StartUpErrors)->getRawData());
        }
        registers_->getRegister(Registry::StartUpErrors)->clearReceived();
      }
      else
      {
        state_ = State::CheckStartUpErrors;
      }
      break;
    case State::CheckRunTimeErrors:
      ROS_DEBUG("Driver %s (%i): State::CheckRunTimeErrors", name_.c_str(), can_id_);
      requestRegister(Registry::RunTimeErrors);
      state_ = State::VerifyRunTimeErrors;
      break;
    case State::VerifyRunTimeErrors:
      ROS_DEBUG("Driver %s (%i): State::VerifyRunTimeErrors", name_.c_str(), can_id_);
      if (registers_->getRegister(Registry::RunTimeErrors)->wasReceived())
      {
        if (registers_->getRegister(Registry::RunTimeErrors)->getRawData() == 0)
        {
          state_ = State::CheckHeading;
        }
        else
        {
          state_ = State::Fault;
          ROS_ERROR("Driver %s (%i): Run Time Error(s) %d", name_.c_str(), can_id_,
              registers_->getRegister(Registry::RunTimeErrors)->getRawData());
        }
        registers_->getRegister(Registry::RunTimeErrors)->clearReceived();
      }
      else
      {
        state_ = State::CheckRunTimeErrors;
      }
      break;
    case State::CheckHeading:
      ROS_DEBUG("Driver %s (%i): State::CheckHeading", name_.c_str(), can_id_);
      requestRegister(Registry::Heading);
      state_ = State::VerifyHeading;
      break;
    case State::VerifyHeading:
      ROS_DEBUG("Driver %s (%i): State::VerifyHeading", name_.c_str(), can_id_);
      if (registers_->getRegister(Registry::Heading)->wasReceived())
      {
        if (registers_->getRegister(Registry::Heading)->getRawData() == 0)
        {
          state_ = State::CheckSro;
        }
        else
        {
          writeRegister(Registry::Heading, 0);
        }
        registers_->getRegister(Registry::Heading)->clearReceived();
      }
      else
      {
        state_ = State::CheckHeading;
      }
      break;
    case State::CheckSro:
      ROS_WARN_THROTTLE(1, "Driver %s (%i): Check SRO", name_.c_str(), can_id_);
      requestRegister(Registry::SroSw);
      state_ = State::Stopped;
      break;
    case State::Stopped:
      ROS_WARN_THROTTLE(1, "Driver %s (%i): Stopped", name_.c_str(), can_id_);
      if (registers_->getRegister(Registry::SroSw)->wasReceived())
      {
        if (registers_->getRegister(Registry::SroSw)->getRawData() != 0)
        {
          state_ = State::PreRunning;
        }
        else
        {
          state_ = State::CheckSro;
        }
        registers_->getRegister(Registry::SroSw)->clearReceived();
      }
      else
      {
        state_ = State::CheckSro;
      }
      break;
    case State::PreRunning:
      ROS_DEBUG("Driver %s (%i): State::PreRunning", name_.c_str(), can_id_);
      writeRegister(Registry::EnableKeySw, 1);
      state_ = State::CheckEnable;
      break;
    case State::CheckEnable:
      ROS_DEBUG("Driver %s (%i): State::CheckEnable", name_.c_str(), can_id_);
      requestRegister(Registry::ControllerEnabled);
      state_ = State::VerifyEnable;
      break;
    case State::VerifyEnable:
      ROS_DEBUG("Driver %s (%i): VerifyEnable", name_.c_str(), can_id_);
      if (registers_->getRegister(Registry::ControllerEnabled)->wasReceived())
      {
        if (registers_->getRegister(Registry::ControllerEnabled)->getRawData() == 0)
        {
          state_ = State::PreRunning;
        }
        else
        {
          state_ = State::Running;
          ROS_INFO("Driver %s (%i): Entering running state.", name_.c_str(), can_id_);
        }
        registers_->getRegister(Registry::ControllerEnabled)->clearReceived();
      }
      else
      {
        state_ = State::CheckEnable;
      }
      break;
    case State::Running:
      ROS_DEBUG("Driver %s (%i): State::Running", name_.c_str(), can_id_);
      break;
    case State::Fault:
      ROS_ERROR_THROTTLE(1, "Driver %s (%i): Fault", name_.c_str(), can_id_);
      break;
    case State::Stopping:
      ROS_WARN_THROTTLE(1, "Driver %s (%i): Stopping", name_.c_str(), can_id_);
      break;
  }
}

bool Driver::commandSpeed()
{
  if (state_ == State::Running)
  {
    writeRegister(Registry::TargetVelocity, speed_ * gear_ratio_);
    return (true);
  }
  return (false);
}

void Driver::resetState()
{
  state_ = State::Start;
}

void Driver::setStopping()
{
  state_ = State::Stopping;
}

void Driver::setGearRatio(double ratio)
{
  gear_ratio_ = ratio;
}

void Driver::setSpeed(float cmd)
{
  speed_ = cmd;
}

void Driver::requestFeedback()
{
  requestRegister(Registry::SroSw);
  static uint8_t count = 0;
  if (count == 4)
    isConnected();
  count++;
  if (count > 4)
    count = 0;
}

void Driver::requestStatus()
{
  static uint8_t status_item = 0;
  switch (status_item)
  {
    case 0:
      requestRegister(Registry::RunTimeErrors);
      break;
    case 1:
      requestRegister(Registry::Temperature);
      break;
    case 2:
      requestRegister(Registry::BatVoltage);
      break;
    case 3:
      requestRegister(Registry::MotVoltage);
      break;
    case 4:
      requestRegister(Registry::ActualCurrent);
      break;
  }
  status_item++;
  if (status_item > 4)
  {
    status_item = 0;
  }

  // Check for run time errors.
  if ((registers_->getRegister(Registry::RunTimeErrors)->getRawData() != 0) && configured_)
  {
    state_ = State::Fault;
    ROS_ERROR("Driver %s (%i): Run Time Error(s) %d", name_.c_str(), can_id_,
        registers_->getRegister(Registry::RunTimeErrors)->getRawData());
  }
}

void Driver::readFrame(const Frame& frame)
{
  if (frame.getCanId() != can_id_)
  {
    ROS_DEBUG("Driver %s (%i): Frame is not for me.", name_.c_str(), can_id_);
    return;
  }
  // All frams from the TPM have 8 bytes of data.
  if (frame.len < 8)
  {
    ROS_WARN("Driver %s (%i): Frame does not have enough data.", name_.c_str(), can_id_);
    return;
  }
  if (registers_->getRegister(frame.data.dest_reg))
  {
    registers_->getRegister(frame.data.dest_reg)->setReceived();
    registers_->getRegister(frame.data.dest_reg)->setRawData(frame.data.value);
  }
}

void Driver::requestRegister(uint16_t id)
{
  Frame tx_frame;
  tx_frame.id = 0x00000060;
  tx_frame.len = 8;
  tx_frame.data.dest_id = can_id_;
  tx_frame.data.dest_reg = id;
  tx_frame.data.action = can_id_ + Action::Read;
  tx_frame.data.value = 0;
  interface_.queue(tx_frame);
}

void Driver::writeRegister(uint16_t id, float value)
{
  Frame tx_frame;
  tx_frame.id = 0x00000060;
  tx_frame.len = 8;
  tx_frame.data.dest_id = can_id_;
  tx_frame.data.dest_reg = id;
  tx_frame.data.action = can_id_ + Action::Write;
  tx_frame.data.value = value;
  interface_.queue(tx_frame);
}

bool Driver::isRunning() const
{
  return (state_ == State::Running);
}

bool Driver::isFault() const
{
  return (state_ == State::Fault);
}

bool Driver::isStopping() const
{
  return (state_ == State::Stopping);
}

std::string Driver::getName() const
{
  return name_;
}
uint8_t Driver::getId() const
{
  return can_id_;
}

/**************************************
** Registry Getters
**************************************/
float Driver::getHeading() const
{
  return registers_->getRegister(Registry::Heading)->getData();
}

float Driver::getMeasuredVelocity() const
{
  return registers_->getRegister(Registry::MeasuredVelocity)->getData() / gear_ratio_;
}

float Driver::getMeasuredTravel() const
{
  return registers_->getRegister(Registry::MeasuredTravel)->getData() / gear_ratio_;
}

uint16_t Driver::getRuntimeErrors() const
{
  registers_->getRegister(Registry::RunTimeErrors)->clearReceived();
  return registers_->getRegister(Registry::RunTimeErrors)->getData();
}

uint16_t Driver::getStartupErrors() const
{
  return registers_->getRegister(Registry::StartUpErrors)->getData();
}

float Driver::getMotorTemp() const
{
  return registers_->getRegister(Registry::MotorTemp)->getData();
}

float Driver::getDriverTemp() const
{
  return registers_->getRegister(Registry::Temperature)->getData();
}

float Driver::getInputVoltage() const
{
  return registers_->getRegister(Registry::BatVoltage)->getData();
}

float Driver::getOutputVoltage() const
{
  // This data is stored as int16 but put into a int32 which causes the sign to be in correct.  It is corrected below.
  return (static_cast<float>(static_cast<int16_t>(registers_->getRegister(Registry::MotVoltage)->getRawData()))
         * registers_->getRegister(Registry::MotVoltage)->getScale());
}

float Driver::getOutputCurrent() const
{
  // This data is stored as int16 but put into a int32 which causes the sign to be in correct.  It is corrected below.
  return (static_cast<float>(static_cast<int16_t>(registers_->getRegister(Registry::ActualCurrent)->getRawData()))
         * registers_->getRegister(Registry::ActualCurrent)->getScale());
}

float Driver::getSpeed() const
{
  return speed_;
}

void Driver::isConnected()
{
  if (state_ != State::Running)
  {
    lost_messages_ = 0;
    return;
  }

  if (registers_->getRegister(Registry::RunTimeErrors)->wasReceived())
  {
    lost_messages_ = 0;
  }
  else
  {
    lost_messages_++;
  }

  if (lost_messages_ > 25)
  {
    state_ = State::Fault;
    ROS_ERROR("Driver %s (%i): Not get data from driver. %i", name_.c_str(), can_id_, lost_messages_);
    lost_messages_ = 0;
  }
}

}  // namespace grizzly_motor_driver
