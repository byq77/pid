///////////////////////////////////////////////////////////////////////////////
//      Title     : pid.cpp
//      Project   : pid
//      Created   : 5/28/2018
//      Author    : Andy Zelenak
//
// BSD 3-Clause License
//
// Copyright (c) 2018, Los Alamos National Security, LLC
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////
// Modifications:
//      Author   : byq77
///////////////////////////////////////////////////////////////////////////////

// Perform PID calculations.

#include "pid/pid.hpp"

using std::placeholders::_1;

namespace pid_controller
{

PID::PID()
:Node(NODE_NAME)
{
  state_sub_ = this->create_subscription<std_msgs::msg::Float64>(TOPIC_FROM_PLANT, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg){
        plant_state_ = msg->data;
        new_state_or_setpt_ = true;
    });
  setpoint_sub_ = this->create_subscription<std_msgs::msg::Float64>(SETPOINT_TOPIC, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        setpoint_ = msg->data;
        new_state_or_setpt_ = true;
    });

  // Create a publisher with a custom Quality of Service profile.
  // rclcpp::QoS custom_qos_profile(rclcpp::KeepLast(7), rmw_qos_profile_sensor_data);
  control_effort_pub_ = this->create_publisher<std_msgs::msg::Float64>(TOPIC_FROM_CONTROLLER, 10);

  // Create parameter listener
  param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());

  // Get initial parameters
  getAndValidateParameters();
}

inline void PID::getAndValidateParameters()
{
  if(param_listener_->try_get_params(this->params_)) {
    const bool all_negative = (params_.Kp <= 0. && params_.Ki <= 0. && params_.Kd <= 0.);
    const bool all_positive = (params_.Kp >= 0. && params_.Ki >= 0. && params_.Kd >= 0.);
    if (all_negative || all_positive) {  // All 3 gains should have the same sign
      Kp_ = params_.Kp;
      Ki_ = params_.Ki;
      Kd_ = params_.Kd;
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 1000,
        "All three gains (Kp, Ki, Kd) should have the same sign for "
        "stability.");
    }
    if (params_.effort_limit[0] < params_.effort_limit[1]) {
      effort_lower_limit_ = params_.effort_limit[0];
      effort_upper_limit_ = params_.effort_limit[1];
    } else {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *(this->get_clock()), 1000,
      "The lower saturation limit cannot be greater than the upper "
      "saturation limit.");
    }
    if (params_.windup_limit[0] < params_.windup_limit[1]) {
      windup_lower_limit_ = params_.windup_limit[0];
      windup_upper_limit_ = params_.windup_limit[1];
    } else {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *(this->get_clock()), 1000,
        "The lower windup limit cannot be greater than the upper "
        "windup limit.");
    }
    cutoff_frequency_ = params_.cutoff_frequency;
    angle_error_ = params_.angle_error;
    pid_enabled_ = params_.pid_enabled;
  }
}

void PID::update()
{
  // Do fresh calcs if knowledge of the system has changed.
  if (new_state_or_setpt_) {
    // Get parameters from the server
    getAndValidateParameters();

    error_[2] = error_[1];
    error_[1] = error_[0];
    error_[0] = setpoint_ - plant_state_;  // Current error goes to slot 0

    // If the angle_error param is true, then address discontinuity in error
    // calc.
    // For example, this maintains an angular error between -180:180.
    if (angle_error_) {
      while (error_[0] < -1.0 * angle_wrap_ / 2.0) {
        error_[0] += angle_wrap_;
      }
      while (error_[0] > angle_wrap_ / 2.0) {
        error_[0] -= angle_wrap_;
      }

      // The proportional error will flip sign, but the integral error
      // won't and the derivative error will be poorly defined. So,
      // reset them.
      error_[2] = 0.;
      error_[1] = 0.;
      error_integral_ = 0.;
    }

    // calculate delta_t
    if (prev_time_.nanoseconds() != 0) {  // Not first time through the program
      delta_t_ = this->now() - prev_time_;
      prev_time_ = this->now();
      if (0 == delta_t_.nanoseconds()) {
        RCLCPP_ERROR(this->get_logger(),
          "delta_t is 0, skipping this loop. Possible overloaded CPU.");
        return;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "prev_time is 0, doing nothing");
      prev_time_ = this->now();
      return;
    }

    // integrate the error
    error_integral_ += error_[0] * delta_t_.nanoseconds() / 1e9;

    // Apply windup limit to limit the size of the integral term
    if (error_integral_ > windup_upper_limit_) {
      error_integral_ = windup_upper_limit_;
    }

    if (error_integral_ < windup_lower_limit_) {
      error_integral_ = windup_lower_limit_;
    }

    // My filter reference was Julius O. Smith III, Intro. to Digital Filters
    // With Audio Applications.
    if (cutoff_frequency_ != -1) {
      // Check if tan(_) is really small, could cause c = NaN
      tan_filt_ = tan((cutoff_frequency_ * 6.2832) * (delta_t_.nanoseconds() / 1e9) / 2);

      // Avoid tan(0) ==> NaN
      if ((tan_filt_ <= 0.) && (tan_filt_ > -0.01)) {
        tan_filt_ = -0.01;
      }
      if ((tan_filt_ >= 0.) && (tan_filt_ < 0.01)) {
        tan_filt_ = 0.01;
      }

      c_ = 1 / tan_filt_;
    }

    filtered_error_[2] = filtered_error_[1];
    filtered_error_[1] = filtered_error_[0];
    filtered_error_[0] = (1 / (1 + c_ * c_ + 1.414 * c_)) * (error_[2] + 2 * error_[1] + error_[0] -
      (c_ * c_ - 1.414 * c_ + 1) * filtered_error_[2] -
      (-2 * c_ * c_ + 2) * filtered_error_[1]);

    // Take derivative of error
    // First the raw, unfiltered data:
    error_deriv_[2] = error_deriv_[1];
    error_deriv_[1] = error_deriv_[0];
    error_deriv_[0] = (error_[0] - error_[1]) / delta_t_.nanoseconds() / 1e9;

    filtered_error_deriv_[2] = filtered_error_deriv_[1];
    filtered_error_deriv_[1] = filtered_error_deriv_[0];

    filtered_error_deriv_[0] =
      (1 / (1 + c_ * c_ + 1.414 * c_)) *
      (error_deriv_[2] + 2 * error_deriv_[1] + error_deriv_[0] -
      (c_ * c_ - 1.414 * c_ + 1) * filtered_error_deriv_[2] - (-2 * c_ * c_ + 2) *
      filtered_error_deriv_[1]);

    // calculate the control effort
    proportional_ = Kp_ * filtered_error_[0];
    integral_ = Ki_ * error_integral_;
    derivative_ = Kd_ * filtered_error_deriv_[0];
    control_effort_ = proportional_ + integral_ + derivative_;

    // Apply saturation limits
    if (control_effort_ > effort_upper_limit_) {
      control_effort_ = effort_upper_limit_;
    } else if (control_effort_ < effort_lower_limit_) {
      control_effort_ = effort_lower_limit_;
    }
  }

  // Publish the stabilizing control effort if the controller is enabled
  if (pid_enabled_) {
    control_msg_.data = control_effort_;
    control_effort_pub_->publish(control_msg_);
  } else {
    error_integral_ = 0.0;
  }

  new_state_or_setpt_ = false;
}

}  // namespace pid_controller
