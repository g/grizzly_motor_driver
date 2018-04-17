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
  state_(State::Start),
  configured_(false),
  registers_(std::shared_ptr<Registers>(new Registers())),
  speed_(0)
{
  configuration_state_ = 0;
}

void Driver::configure()
{
  uint16_t reg = registers_->getWriteableId(configuration_state_);

  if (configuration_state_ >= registers_->getNumberOfWriteableIds())
  {
    ROS_INFO("%s was configured.", name_.c_str());
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
      ROS_WARN("%s, trying to configure register %d failed, got %d and wanted %d. Retrying.",
        name_.c_str(), reg,
        registers_->getRegister(reg)->getRawData(),
        registers_->getRegister(reg)->getRawInitial());
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
      ROS_INFO("Driver %i: State::CheckStartUpErrors", can_id_);
      requestRegister(Registry::StartUpErrors);
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
      requestRegister(Registry::Heading);
      state_ = State::VerifyHeading;
      break;
    case State::VerifyHeading:
      ROS_INFO("VerifyHeading");
      if (registers_->getRegister(Registry::Heading)->wasReceived())
      {
        if (registers_->getRegister(Registry::Heading)->getRawData() == 0)
        {
          state_ = State::CheckSro;
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
      requestRegister(Registry::SroSw);
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
     writeRegister(Registry::EnableKeySw, 1);
     writeRegister(Registry::Heading, 0);
      state_ = State::Running;
      break;
    case State::Running:
      commandSpeed();
    //  writeRegister(501, 543);
      ROS_INFO("Running");
      break;
    case State::Fault:
      ROS_ERROR("Fault");
      break;
  }
}

bool Driver::commandSpeed()
{
  if (state_ == State::Running)
  {
    writeRegister(Registry::Heading, speed_);
    return(true);
  }
  return(false);
}

void Driver::setSpeed(double cmd)
{
  speed_ = static_cast<int32_t>(cmd);
}

void Driver::requestFeedback()
{
  requestRegister(Registry::SroSw);
}

void Driver::requestStatus()
{
  requestRegister(Registry::RunTimeErrors);
  requestRegister(Registry::StartUpErrors);
  requestRegister(Registry::Temperature);
  requestRegister(Registry::BatVoltage);
}

void Driver::readFrame(const Frame &frame)
{
  if (frame.getCanId() != can_id_)
  {
    ROS_DEBUG("%s: Frame is not for me.", name_.c_str());
    return;
  }

  // All frams from the TPM have 8 bytes of data.
  if (frame.len < 8)
  {
    ROS_INFO("%s: Frame does not have enough data.", name_.c_str());
    return;
  }
  //ROS_INFO("%s: Got Frame for %d", name_.c_str(), frame.data.dest_reg)
  if (frame.data.dest_reg == Registry::Heading)
  {
    ROS_ERROR("%s: Heading Frame: %i is: %i.", name_.c_str(), frame.data.dest_reg, frame.data.value);
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
  if(id == Registry::Heading){
    ROS_ERROR("Writing Heading with value %d", static_cast<int32_t> (value));
  }
  Frame tx_frame;
  tx_frame.id = 0x00000060;
  tx_frame.len = 8;
  tx_frame.data.dest_id = can_id_;
  tx_frame.data.dest_reg = id;
  tx_frame.data.action = can_id_ + Action::Write;
  tx_frame.data.value = value;
  interface_.queue(tx_frame);
}

bool Driver::isConfigured() const
{
  return configured_;
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

uint16_t Driver::getRuntimeErrors() const
{
  return registers_->getRegister(Registry::RunTimeErrors)->getData();
}

uint16_t Driver::getStartupErrors() const
{
  return registers_->getRegister(Registry::StartUpErrors)->getData();
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
  return registers_->getRegister(Registry::MotVoltage)->getData();
}
}  // namespace grizzly_motor_driver
