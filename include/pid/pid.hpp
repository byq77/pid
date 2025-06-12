#ifndef PID_CONTROLLER__PID_HPP_
#define PID_CONTROLLER__PID_HPP_

#include <iostream>
#include <cmath>
#include <cstdio>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "pid/pid_controller_parameters.hpp"


namespace pid_controller
{
class PID : public rclcpp::Node
{
public:
  explicit PID();
  void update();

public:
  double rate() const
  {
    return params_.rate;
  }

public:
  static constexpr const char * TOPIC_FROM_PLANT = "state";
  static constexpr const char * SETPOINT_TOPIC = "setpoint";
  static constexpr const char * TOPIC_FROM_CONTROLLER = "control_effort";
  static constexpr const char * NODE_NAME = "pid_controller";

private:
  void getAndValidateParameters();

private:
  /////////////////////////////////////////
  // Primary PID controller input variables
  /////////////////////////////////////////
  double plant_state_;               // current state of plant
  bool new_state_or_setpt_ = false;  // Indicate that fresh calculations need to be run
  double setpoint_ = 0;              // desired state of plant

  ///////////////////////////////
  // User-configurable parameters
  ///////////////////////////////
  // PID gains
  double Kp_ = 0, Ki_ = 0, Kd_ = 0;

  // Parameters for error calc. with discontinuous input
  bool angle_error_ = false;
  double angle_wrap_ = 2.0 * 3.14159;

  // To pause the PID controller
  bool pid_enabled_ = true;

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoff_frequency_ = -1;

  //////////////////////////////////
  // Used for internal calculations:
  //////////////////////////////////

  // Primary output variable
  double control_effort_ = 0;

  rclcpp::Time prev_time_{};
  rclcpp::Duration delta_t_{std::chrono::nanoseconds::zero()};
  bool first_reconfig_ = true;

  double error_integral_ = 0;
  double proportional_ = 0;  // proportional term of output
  double integral_ = 0;      // integral term of output
  double derivative_ = 0;    // derivative term of output

  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency
  // at 1/4 of the sample rate.
  double c_ = 1.;

  // Used to check for tan(0)==>NaN in the filter calculation
  double tan_filt_ = 1.;

  // Upper and lower saturation limits
  double effort_upper_limit_ = 1000, effort_lower_limit_ = -1000;

  // Anti-windup term. Limits the absolute value of the integral term.
  double windup_upper_limit_ = 1000, windup_lower_limit_ = -1000;

  // Initialize filter data with zeros
  std::vector<double> error_ = std::vector<double>(3, 0);
  std::vector<double> filtered_error_ = std::vector<double>(3, 0);
  std::vector<double> error_deriv_ = std::vector<double>(3, 0);
  std::vector<double> filtered_error_deriv_ = std::vector<double>(3, 0);

  ///////////////////////////////////////////
  // Topic and node names and message objects
  ///////////////////////////////////////////
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_effort_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_sub_, setpoint_sub_;
  std_msgs::msg::Float64 control_msg_, state_msg_;
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};
}  // namespace pid_controller


#endif // PID_CONTROLLER__PID_HPP_
