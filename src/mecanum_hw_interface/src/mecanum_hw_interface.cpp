#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "mecanum_hw_interface/mecanum_hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

static auto s_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);


namespace mecanum_hw_interface
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_POSITION;

CallbackReturn MecanumHWSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 读取参数
  if (info_.hardware_parameters.count("device"))
    device_ = info_.hardware_parameters.at("device");
  if (info_.hardware_parameters.count("baud"))
    baud_ = std::stoi(info_.hardware_parameters.at("baud"));

  // 期望 4 个关节
  if (info_.joints.size() != 4)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mecanum_hw_interface"),
                 "Expected 4 joints, but got %zu. Define FL, FR, RR, RL.", info_.joints.size());
    return CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();
  hw_states_.assign(n, 0.0);   // rad/s
  hw_pos_.assign(n, 0.0);      // rad
  hw_commands_.assign(n, 0.0); // rad/s

  // 校验接口：command[velocity] + state[velocity, position]
  for (const auto & j : info_.joints)
  {
    bool has_cmd_vel=false, has_state_vel=false, has_state_pos=false;
    for (const auto & ci : j.command_interfaces) if (ci.name == HW_IF_VELOCITY) has_cmd_vel=true;
    for (const auto & si : j.state_interfaces) {
      if (si.name == HW_IF_VELOCITY) has_state_vel=true;
      if (si.name == HW_IF_POSITION) has_state_pos=true;
    }
    if (!has_cmd_vel || !has_state_vel || !has_state_pos)
    {
      RCLCPP_ERROR(rclcpp::get_logger("mecanum_hw_interface"),
                   "Joint '%s' must have: command[velocity] + state[velocity, position].",
                   j.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // 按名字建立索引映射
  for (size_t i=0;i<n;++i)
  {
    const auto & name = info_.joints[i].name;
    if      (name=="front_left_wheel_joint")  idx_fl = static_cast<int>(i);
    else if (name=="front_right_wheel_joint") idx_fr = static_cast<int>(i);
    else if (name=="rear_right_wheel_joint")  idx_rr = static_cast<int>(i);
    else if (name=="rear_left_wheel_joint")   idx_rl = static_cast<int>(i);
  }
  if (idx_fl<0 || idx_fr<0 || idx_rr<0 || idx_rl<0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("mecanum_hw_interface"),
                 "Wheel joint names mismatch (need front_left/right_, rear_right/left_).");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("mecanum_hw_interface"),
              "on_init ok. device=%s baud=%d", device_.c_str(), baud_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumHWSystem::on_configure(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MecanumHWSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size()*2);
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, HW_IF_VELOCITY, &hw_states_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, HW_IF_POSITION, &hw_pos_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MecanumHWSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[i].name, HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

CallbackReturn MecanumHWSystem::on_activate(const rclcpp_lifecycle::State &)
{
  std::fill(hw_states_.begin(), hw_states_.end(), 0.0);
  std::fill(hw_pos_.begin(),    hw_pos_.end(),    0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);

  serial_ = std::make_unique<MecanumSerialPort>();
  if (!serial_->open(device_, baud_))
  {
    RCLCPP_WARN(rclcpp::get_logger("mecanum_hw_interface"),
                "Serial open failed (%s,%d). Running without STM32.", device_.c_str(), baud_);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("mecanum_hw_interface"), "Serial opened.");
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumHWSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (serial_) { serial_->close(); serial_.reset(); }
  
  return CallbackReturn::SUCCESS;
}

return_type MecanumHWSystem::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // 尝试从下位机读回四轮角速度 [rad/s]
  std::array<double,4> w{};
  bool got=false;
  if (serial_ && serial_->is_open())
    got = serial_->readMeasPacket(w);

  if (got)
  {
    // ★ 这里假定下位机顺序为 [FR, FL, RL, RR]；
    hw_states_[idx_fr] = w[0];
    hw_states_[idx_fl] = w[1];
    hw_states_[idx_rl] = w[2];
    hw_states_[idx_rr] = w[3];
  }
  // else
  // {
  //   // 无硬件或读失败：用命令近似速度，便于上层可视化/调参
  //   for (size_t i=0;i<hw_states_.size();++i) hw_states_[i] = hw_commands_[i];
  // }

  // 用速度积分出角度（将来接编码器可直接写真实角度替代这一步）
  const double dt = period.seconds();
  for (size_t i=0;i<hw_pos_.size();++i) hw_pos_[i] += hw_states_[i] * dt;

  return return_type::OK;
}

return_type MecanumHWSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!(serial_ && serial_->is_open()))
    return return_type::OK;

  auto safe = [](double v){ return std::isnan(v) ? 0.0 : v; };

  // 统一按 [FR, FL, RL, RR] 封包
  std::array<double,4> w{
    safe(hw_commands_[idx_fr]),
    safe(hw_commands_[idx_fl]),
    safe(hw_commands_[idx_rl]),
    safe(hw_commands_[idx_rr])
  };

    // ☆ 每秒打印一次命令（确认 write() 在跑、值是否非零）
  // RCLCPP_INFO_THROTTLE(
  //   rclcpp::get_logger("mecanum_hw_interface"), *s_clock, 1000,
  //   "write cmd w=[fr=%.3f fl=%.3f rl=%.3f rr=%.3f]",
  //   w[0], w[1], w[2], w[3]);

  // ☆ 无条件发送（即使为 0/未变化也发，便于在 socat 看到 01 10）
  const bool ok = serial_->writeCmdPacket(w);
  if (!ok) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("mecanum_hw_interface"), *s_clock, 2000,
      "writeCmdPacket() failed (errno=%d)", errno);
  }
  return return_type::OK;
}

} // namespace mecanum_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecanum_hw_interface::MecanumHWSystem,
                       hardware_interface::SystemInterface)
