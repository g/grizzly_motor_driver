
#include <ros/ros.h>

#include "grizzly_motor_driver/driver.h"

namespace grizzly_motor_driver
{

  namespace States
  {
    enum State
    {
      Configure = 0,
      CheckStartUpErrors,
      VerifyStartUpErrors,
      CheckHeading,
      VerifyHeading,
      CheckSro,
      Stopped,
      PreRunning,
      Running,
      Fault,
      NumberOfStates
    };
  }  // namespace States
  typedef States::State State;

Driver::Driver(Interface &interface, const uint8_t can_id, const std::string &name) :
  interface_(interface),
  can_id_(can_id),
  name_(name),
  state_(State::Configure),
  configured_(false),
  registers_(std::shared_ptr<Registers>(new Registers()))
{
  configuration_state_ = 0;
}

void Driver::configure()
{
  uint16_t reg = registers_->getWriteableId(configuration_state_);

  if (configuration_state_ >= registers_->getNumberOfWriteableIds())
  {
    ROS_INFO("Configured.");
    configured_ = true;
    return;
  }

  if (registers_->getRegister(reg)->wasReceived())
  {
    if (registers_->getRegister(reg)->getRawData() ==
      registers_->getRegister(reg)->getRawInitial())
    {
      ROS_DEBUG("Match, got %d and wanted %d",
        registers_->getRegister(reg)->getRawData(),
        registers_->getRegister(reg)->getRawInitial());
      configuration_state_++;
    }
    else
    {
      ROS_WARN("NOT A MATCH, got %d and wanted %d",
        registers_->getRegister(reg)->getRawData(),
        registers_->getRegister(reg)->getRawInitial());
      writeRegister(reg, registers_->getRegister(reg)->sendInitial());
    }
    registers_->getRegister(reg)->clearReceived();
  }
  else
  {
    readRegister(reg);
  }
}

void Driver::run()
{
  switch (state_)
  {
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
      ROS_INFO("Driver %i: State::CheckStartUpErrors", can_id_);
      readRegister(Registry::StartUpErrors);
      state_ = State::VerifyStartUpErrors;
      break;
    case State::VerifyStartUpErrors:
      ROS_INFO("State::VerifyStartUpErrors");
      if (registers_->getRegister(Registry::StartUpErrors)->wasReceived())
      {
        if (registers_->getRegister(Registry::StartUpErrors)->getRawData() == 0)
        {
          state_ = State::CheckHeading;
        }
        else
        {
          state_ = State::Fault;
          ROS_ERROR("Start Up Error(s) %d", registers_->getRegister(Registry::StartUpErrors)->getRawData());
        }
        registers_->getRegister(Registry::StartUpErrors)->clearReceived();
      }
      else
      {
        state_ = State::CheckStartUpErrors;
        ROS_INFO("No good");
      }
      break;
    case State::CheckHeading:
      ROS_INFO("Driver %i: State::CheckHeading", can_id_);
      readRegister(Registry::Heading);
      state_ = State::VerifyHeading;
      break;
    case State::VerifyHeading:
      ROS_INFO("VerifyHeading");
      if (registers_->getRegister(Registry::Heading)->wasReceived())
      {
        if (registers_->getRegister(Registry::Heading)->getRawData() == 0)
        {
          state_ = State::CheckSro;
          writeRegister(Registry::HeadingNetSelect, 170);
        }
        else
        {
          state_ = State::Fault;
          ROS_ERROR("Heading %d", registers_->getRegister(Registry::Heading)->getRawData());
        }
        registers_->getRegister(Registry::Heading)->clearReceived();
      }
      else
      {
        state_ = State::CheckStartUpErrors;
        ROS_INFO("No good");
      }
      break;
    case State::CheckSro:
      ROS_INFO("Driver %i: State::CheckSro", can_id_);
      readRegister(Registry::SroSw);
      state_ = State::Stopped;
      break;
    case State::Stopped:
      ROS_INFO("State::Stopped");
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
      ROS_INFO("PreRunning");
      writeRegister(Registry::HeadingNetSelect, 170);
      writeRegister(Registry::EnableKeySw, 1);
      writeRegister(Registry::Heading, 0);
      state_ = State::Running;
      break;
    case State::Running:
      ROS_INFO("Running");
      static int i = 0;
      if (i < (25*5))
      {
        writeRegister(Registry::Heading, 1200);
        i++;
      }
      else
        writeRegister(Registry::Heading, 1200);

      break;
    case State::Fault:
      ROS_ERROR("Fault");
      break;
  }
}

void Driver::readFrame(const Frame &frame)
{
  if (frame.getCanId() != can_id_)
  {
    ROS_INFO("Frame is not for me.");
    return;
  }

  if (frame.len < 8)
  {
    ROS_INFO("Frame does not have enough data.");
    return;
  }
  ROS_INFO("Got Frame for %d", frame.data.dest_reg);
  registers_->getRegister(frame.data.dest_reg)->setReceived();
  registers_->getRegister(frame.data.dest_reg)->setRawData(frame.data.value);

  // ROS_INFO("CAN ID 0x%08x", frame.id);
  // ROS_INFO("DEVICE ID 0x%08x", frame.getCanId());
  // ROS_INFO("Dest ID 0x%02x", frame.data.dest_id);
  // ROS_INFO("Dest Reg 0x%04x", frame.data.dest_reg);
  // ROS_INFO("Action 0x%02x", frame.data.action);
  // ROS_INFO("Value 0x%08x", frame.data.value);
}

void Driver::readRegister(uint16_t id)
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

}  // namespace grizzly_motor_driver
