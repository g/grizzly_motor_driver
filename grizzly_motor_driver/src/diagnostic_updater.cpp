/**
Software License Agreement (BSD)
\authors   Mike Hosmar <mhosmar@clearpathrobotics.com>
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

#include "boost/foreach.hpp"
#include "diagnostic_updater/update_functions.h"
#include "grizzly_motor_driver/diagnostic_updater.h"

namespace grizzly_motor_driver
{
typedef diagnostic_msgs::DiagnosticStatus Status;

GrizzlyMotorDriverDiagnosticUpdater::GrizzlyMotorDriverDiagnosticUpdater()
{
  status_initialized_ = false;
  feedback_initialized_ = false;
  setHardwareID("none");
  status_sub_ = nh_.subscribe("status", 5, &GrizzlyMotorDriverDiagnosticUpdater::statusCallback, this);
  feedback_sub_ = nh_.subscribe("feedback", 5, &GrizzlyMotorDriverDiagnosticUpdater::feedbackCallback, this);
}

const char* GrizzlyMotorDriverDiagnosticUpdater::getFaultString(uint16_t fault)
{
  switch (fault)
  {
    case grizzly_motor_msgs::Feedback::ERROR_RUNTIME_BRAKE_OVER_CURRENT:
      return "brake over current fault";
    case grizzly_motor_msgs::Feedback::ERROR_RUNTIME_HW_SHUTDOWN:
      return "hardware shutdown fault";
    case grizzly_motor_msgs::Feedback::ERROR_RUNTIME_POT1_OPEN:
      return "pot 1 open falt";
    case grizzly_motor_msgs::Feedback::ERROR_RUNTIME_POT2_OPEN:
      return "pot 2 open falt";
    case grizzly_motor_msgs::Feedback::ERROR_RUNTIME_BOTH_DIR_SWITCHES:
      return "both direction switches enabled fault";
    case grizzly_motor_msgs::Feedback::ERROR_RUNTIME_OVER_TEMP:
      return "over temp fault";
    case grizzly_motor_msgs::Feedback::ERROR_RUNTIME_CURRENT_SENSOR_SATURATED:
      return "current sensor saturated fault";
    case grizzly_motor_msgs::Feedback::ERROR_RUNTIME_BATTERY_OVER_VOLTAGE_ON_START:
      return "battery over voltage on start fault";
    case grizzly_motor_msgs::Feedback::ERROR_RUNTIME_BATTERY_UNDER_VOLTAGE_ON_START:
      return "battery under volatge on start fault";
    case grizzly_motor_msgs::Feedback::ERROR_RUNTIME_BASE_DISCHARGED:
      return "base discharged fault";
    default:
      return "unknown fault";
  }
}

void GrizzlyMotorDriverDiagnosticUpdater::temperatureDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                                                 int driver)
{
  stat.add("Driver CAN ID", static_cast<int>(last_status_->statuses[driver].device_number));
  stat.add("Driver Role", last_status_->statuses[driver].device_name.c_str());

  stat.add("Internal driver temperature (degC)", last_status_->statuses[driver].temperature_driver);
  stat.add("Motor temperature (degC)", last_status_->statuses[driver].temperature_motor);

  if (last_status_->statuses[driver].temperature_driver < 75.0 &&
      last_status_->statuses[driver].temperature_motor < 75.0)
  {
    stat.summary(Status::OK, "Motor tempratures are OK.");
  }
  else if (last_status_->statuses[driver].temperature_driver < 90.0 &&
           last_status_->statuses[driver].temperature_motor < 90.0)
  {
    stat.summary(Status::WARN, "Motor tempratures are HIGH.");
  }
  else
  {
    stat.summary(Status::ERROR, "Motor tempratures are TOO HIGH.");
  }
}

void GrizzlyMotorDriverDiagnosticUpdater::powerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                                           int driver)
{

  if (last_status_->statuses[driver].voltage_input > 35)
  {
    stat.summary(Status::OK, "Motor power is OK.");
  }
  else
  {
    stat.summary(Status::ERROR, "Motor voltage is LOW.");
  }

  stat.add("Driver CAN ID", static_cast<int>(last_status_->statuses[driver].device_number));
  stat.add("Driver Role", last_status_->statuses[driver].device_name.c_str());

  stat.add("Voltage at motor controller input terminal (V)", last_status_->statuses[driver].voltage_input);
  stat.add("Voltage output to the motor (V)", last_status_->statuses[driver].voltage_output);
  stat.add("Current output to the motor (A)", last_status_->statuses[driver].current);
}

void GrizzlyMotorDriverDiagnosticUpdater::feedbackDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat,
                                                              int driver)
{
  if (last_feedback_->feedbacks[driver].error_runtime == 0)
  {
    stat.summary(Status::OK, "Motor driver is OK.");
  }
  else
  {
    stat.summaryf(Status::ERROR, "'%s' driver (%i) has a %s.", (last_feedback_->feedbacks[driver].device_name.c_str()),
                  last_feedback_->feedbacks[driver].device_number,
                  getFaultString(last_feedback_->feedbacks[driver].error_runtime));
  }

  stat.add("Driver CAN ID", static_cast<int>(last_feedback_->feedbacks[driver].device_number));
  stat.add("Driver Role", last_feedback_->feedbacks[driver].device_name.c_str());
}

void GrizzlyMotorDriverDiagnosticUpdater::statusCallback(const grizzly_motor_msgs::MultiStatus::ConstPtr& status_msg)
{
  last_status_ = status_msg;
  if (!status_initialized_)
  {
    for (int i = 0; i < status_msg->statuses.size(); i++)
    {
      char name[100];
      snprintf(name, sizeof(name), "Grizzly motor status on: %s with CAN ID (%d)",
               last_status_->statuses[i].device_name.c_str(), last_status_->statuses[i].device_number);
      add(name, boost::bind(&GrizzlyMotorDriverDiagnosticUpdater::temperatureDiagnostics, this, _1, i));
      add(name, boost::bind(&GrizzlyMotorDriverDiagnosticUpdater::powerDiagnostics, this, _1, i));
    }
    status_initialized_ = true;
  }
  else
  {
    update();
  }
}

void GrizzlyMotorDriverDiagnosticUpdater::feedbackCallback(
    const grizzly_motor_msgs::MultiFeedback::ConstPtr& feedback_msg)
{
  last_feedback_ = feedback_msg;
  if (!feedback_initialized_)
  {
    for (int i = 0; i < feedback_msg->feedbacks.size(); i++)
    {
      char name[100];
      snprintf(name, sizeof(name), "Grizzly motor feedback on: %s with CAN ID (%d)",
               last_feedback_->feedbacks[i].device_name.c_str(), last_feedback_->feedbacks[i].device_number);
      add(name, boost::bind(&GrizzlyMotorDriverDiagnosticUpdater::feedbackDiagnostics, this, _1, i));
    }
    feedback_initialized_ = true;
  }
  for (auto& feedback : feedback_msg->feedbacks)
  {
    if (feedback.error_runtime > 0)
    {
      update();
      break;
    }
  }
}

}  // namespace grizzly_motor_driver
