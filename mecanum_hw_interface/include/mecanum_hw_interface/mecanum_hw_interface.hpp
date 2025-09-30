// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef MECANUM_HW_INTERFACE__MECANUM_HW_INTERFACE_HPP_
#define MECANUM_HW_INTERFACE__MECANUM_HW_INTERFACE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mecanum_hw_interface/mecanum_serial_port.hpp"

namespace mecanum_hw_interface
{
class MecanumHWSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // params
  std::string device_{"/dev/ttyUSB0"};
  int baud_{115200};

    // 索引映射（按名字绑定，避免顺序问题）
  int idx_fl{-1}, idx_fr{-1}, idx_rr{-1}, idx_rl{-1};

  // 状态/命令缓冲
  std::vector<double> hw_commands_;  // 角速度命令 [rad/s]
  std::vector<double> hw_states_;    // 角速度状态 [rad/s]
  std::vector<double> hw_pos_;       // 角度状态   [rad]


  std::unique_ptr<MecanumSerialPort> serial_;
};

}  // namespace mecanum_hw_interface

#endif  // MECANUM_HW_INTERFACE__MECANUM_HW_INTERFACE_HPP_
