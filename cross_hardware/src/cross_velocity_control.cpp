// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cross_hardware/cross_velocity_control.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cross_hardware
{

return_type CrossVelocityControlHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  node_ = std::make_shared<rclcpp::Node>(info.name);

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("CrossVelocityControlHardware"),
        "Joint '%s' has %d command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("CrossVelocityControlHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger("CrossVelocityControlHardware"),
        "Joint '%s' has %d state interface. 1 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("CrossVelocityControlHardware"),
        "Joint '%s' have %s state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }
  }

  exec_.add_node(node_);

  // initialize command subscriber
  fdbck_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "motors_feedback", rclcpp::SystemDefaultsQoS(), [this](
      const std::shared_ptr<std_msgs::msg::Float32MultiArray> msg) -> void {
      RCLCPP_WARN_STREAM(
      rclcpp::get_logger("CrossVelocityControlHardware"),
      "Received feedback " << msg->data[0]);
      feedback_msg_ptr_ = std::move(msg);
    });

  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("motors_command", 10);

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
CrossVelocityControlHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CrossVelocityControlHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

void CrossVelocityControlHardware::ExecutorThread()
{
  exec_.spin();
}

return_type CrossVelocityControlHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("CrossVelocityControlHardware"),
    "Starting ...please wait...");

  for (int i = 0; i <= hw_start_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("CrossVelocityControlHardware"),
      "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (uint i = 0; i < hw_states_.size(); i++) {
    if (std::isnan(hw_states_[i])) {
      hw_states_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  feedback_msg_ptr_ = std::make_shared<std_msgs::msg::Float32MultiArray>();
  cmd_msg_ptr_ = std::make_shared<std_msgs::msg::Float32MultiArray>();

  feedback_msg_ptr_->data.resize(hw_states_.size());
  cmd_msg_ptr_->data.resize(hw_states_.size());

  //Start executor thread
  th_ex_ = std::make_shared<std::thread>(std::bind(&CrossVelocityControlHardware::ExecutorThread,this));

  t0_ = node_->get_clock()->now();

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("CrossVelocityControlHardware"),
    "System Sucessfully started!");

  return return_type::OK;
}

return_type CrossVelocityControlHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("CrossVelocityControlHardware"),
    "Stopping ...please wait...");

  for (int i = 0; i <= hw_stop_sec_; i++) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("CrossVelocityControlHardware"),
      "%.1f seconds left...", hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("CrossVelocityControlHardware"),
    "System sucessfully stopped!");

  return return_type::OK;
}

hardware_interface::return_type CrossVelocityControlHardware::read()
{
  RCLCPP_INFO(
    rclcpp::get_logger("CrossVelocityControlHardware"),
    "Reading...");

  rclcpp::Time current_time = node_->get_clock()->now();
  rclcpp::Duration dt = current_time - t0_;

  for (uint i = 0; i < hw_states_.size(); i++) {
    // Cross movement
    // 0:FL 1:RL 2:FR 3:RR
    hw_states_[i] = hw_states_[i] + feedback_msg_ptr_->data[i] * dt.seconds();
    RCLCPP_INFO(
      rclcpp::get_logger("CrossVelocityControlHardware"),
      "Got state %.5f for joint %d!", hw_states_[i], i);
  }

  t0_ = node_->get_clock()->now();

  RCLCPP_INFO(
    rclcpp::get_logger("CrossVelocityControlHardware"),
    "Joints sucessfully read!");

  return return_type::OK;
}

hardware_interface::return_type cross_hardware::CrossVelocityControlHardware::write()
{
  RCLCPP_INFO(
    rclcpp::get_logger("CrossVelocityControlHardware"),
    "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++) {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("CrossVelocityControlHardware"),
      "Got command %.5f for joint %d!", hw_commands_[i], i);

    cmd_msg_ptr_->data[i] = hw_commands_[i];


  }

  cmd_pub_->publish(*cmd_msg_ptr_);

  RCLCPP_INFO(
    rclcpp::get_logger("CrossVelocityControlHardware"),
    "Joints sucessfully written!");

  return return_type::OK;
}

}  // namespace cross_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cross_hardware::CrossVelocityControlHardware,
  hardware_interface::SystemInterface
)
