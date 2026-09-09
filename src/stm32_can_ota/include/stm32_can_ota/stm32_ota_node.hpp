#ifndef STM32_CAN_OTA__STM32_OTA_NODE_HPP_
#define STM32_CAN_OTA__STM32_OTA_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "stm32_can_ota/msg/ota_status.hpp"
#include "stm32_can_ota/ota_client.hpp"
#include "stm32_can_ota/srv/cancel_ota.hpp"
#include "stm32_can_ota/srv/start_ota.hpp"

namespace stm32_can_ota
{

// ROS 2 节点：把 OtaClient(纯 C++ 逻辑) 包装成"可被外部调用"的服务。
// 对外提供：
//   /stm32_ota/start   服务(StartOta)：请求升级
//   /stm32_ota/cancel  服务(CancelOta)：请求取消
//   /stm32_ota/status  话题(OtaStatus)：周期性广播进度/状态
class Stm32OtaNode : public rclcpp::Node
{
public:
  explicit Stm32OtaNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Stm32OtaNode() override;

private:
  using StartOta = stm32_can_ota::srv::StartOta;
  using CancelOta = stm32_can_ota::srv::CancelOta;
  using OtaStatusMsg = stm32_can_ota::msg::OtaStatus;

  OtaClient::Options loadClientOptions();  // 从参数服务器读 can_interface/超时等配置
  // /start 回调：取出 firmware_path/version/dry_run，调用 ota_client_->start()
  void handleStartOta(
    const std::shared_ptr<StartOta::Request> request,
    std::shared_ptr<StartOta::Response> response);
  // /cancel 回调：调用 ota_client_->cancel()
  void handleCancelOta(
    const std::shared_ptr<CancelOta::Request> request,
    std::shared_ptr<CancelOta::Response> response);
  void publishStatus();                    // 定时器触发：把最新状态发到 /status 话题
  void onClientStatus(const OtaClient::Status & status);  // 注册给 OtaClient 的回调：存最新状态
  OtaStatusMsg makeStatusMessage(const OtaClient::Status & status);  // Status→ROS 消息

  std::shared_ptr<OtaClient> ota_client_;  // 升级核心逻辑
  rclcpp::Service<StartOta>::SharedPtr start_service_;
  rclcpp::Service<CancelOta>::SharedPtr cancel_service_;
  rclcpp::Publisher<OtaStatusMsg>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;  // 周期性发布状态用的定时器

  std::mutex status_mutex_;                // 保护 latest_status_(跨线程读写)
  OtaClient::Status latest_status_;        // 缓存最近一次状态，供 publishStatus 取用
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__STM32_OTA_NODE_HPP_
