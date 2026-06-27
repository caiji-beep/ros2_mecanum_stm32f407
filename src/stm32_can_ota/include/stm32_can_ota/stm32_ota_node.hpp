#ifndef STM32_CAN_OTA__STM32_OTA_NODE_HPP_
#define STM32_CAN_OTA__STM32_OTA_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "stm32_can_ota/msg/ota_status.hpp"
#include "stm32_can_ota/ota_client.hpp"
#include "stm32_can_ota/srv/start_ota.hpp"

namespace stm32_can_ota
{

class Stm32OtaNode : public rclcpp::Node
{
public:
  explicit Stm32OtaNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using StartOta = stm32_can_ota::srv::StartOta;
  using OtaStatusMsg = stm32_can_ota::msg::OtaStatus;

  OtaClient::Options loadClientOptions();
  void handleStartOta(
    const std::shared_ptr<StartOta::Request> request,
    std::shared_ptr<StartOta::Response> response);
  void publishStatus();
  void onClientStatus(const OtaClient::Status & status);
  OtaStatusMsg makeStatusMessage(const OtaClient::Status & status);

  std::shared_ptr<OtaClient> ota_client_;
  rclcpp::Service<StartOta>::SharedPtr start_service_;
  rclcpp::Publisher<OtaStatusMsg>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::mutex status_mutex_;
  OtaClient::Status latest_status_;
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__STM32_OTA_NODE_HPP_
