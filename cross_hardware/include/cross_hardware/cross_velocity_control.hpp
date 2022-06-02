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


#ifndef CROSS_VELOCITY__CONTROL_HPP_
#define CROSS_VELOCITY__CONTROL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "cross_hardware/visibility_control.h"

using hardware_interface::return_type;

namespace cross_hardware
{
class CrossVelocityControlHardware : public
  hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(CrossVelocityControlHardware);

  CROSS_HARDWARE_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  CROSS_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CROSS_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CROSS_HARDWARE_PUBLIC
  return_type start() override;

  CROSS_HARDWARE_PUBLIC
  return_type stop() override;

  CROSS_HARDWARE_PUBLIC
  return_type read() override;

  CROSS_HARDWARE_PUBLIC
  return_type write() override;

private:

  void ExecutorThread();

  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // Delta of time between readings
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Time t0_;

  // Command publisher to arduino
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>> cmd_pub_ =
    nullptr;

  // Motors velocity feedback
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr fdbck_sub_ = nullptr;
  std::shared_ptr<std_msgs::msg::Float32MultiArray> feedback_msg_ptr_ = nullptr, cmd_msg_ptr_ = nullptr;

  //Executor
  rclcpp::executors::SingleThreadedExecutor exec_;
  // Executor thread
  std::shared_ptr<std::thread> th_ex_;


};

}  // namespace CROSS_hardware

#endif  // CROSS_HARDWARE__RRBOT_SYSTEM_POSITION_ONLY_HPP_
