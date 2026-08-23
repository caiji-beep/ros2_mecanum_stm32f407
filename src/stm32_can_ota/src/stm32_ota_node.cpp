#include "stm32_can_ota/stm32_ota_node.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

namespace stm32_can_ota
{

namespace
{

uint32_t checkedU32Parameter(
  rclcpp::Node & node,
  const std::string & name,
  int64_t minimum,
  int64_t maximum)
{
  const auto value = node.get_parameter(name).as_int();
  if (value < minimum || value > maximum) {
    throw std::runtime_error("parameter out of range: " + name);
  }
  return static_cast<uint32_t>(value);
}

}  // namespace

Stm32OtaNode::Stm32OtaNode(const rclcpp::NodeOptions & options)
: Node("stm32_ota_node", options)
{
  declare_parameter<std::string>("can_interface", "can0");
  declare_parameter<int64_t>("node_id", 1);
  declare_parameter<int64_t>("request_base_id", 0x600);
  declare_parameter<int64_t>("response_base_id", 0x580);
  declare_parameter<bool>("extended_id", false);
  declare_parameter<std::string>("default_firmware_path", "");
  declare_parameter<int64_t>("block_size", 256);
  declare_parameter<int64_t>("ack_timeout_ms", 1000);
  declare_parameter<int64_t>("capability_timeout_ms", 1000);
  declare_parameter<int64_t>("receive_timeout_ms", 100);
  declare_parameter<int64_t>("status_publish_period_ms", 500);

  ota_client_ = std::make_shared<OtaClient>(loadClientOptions());
  ota_client_->setStatusCallback(
    [this](const OtaClient::Status & status) {
      onClientStatus(status);
    });
  latest_status_ = ota_client_->status();

  status_pub_ = create_publisher<OtaStatusMsg>("/stm32_ota/status", 10);
  start_service_ = create_service<StartOta>(
    "/stm32_ota/start",
    [this](
      const std::shared_ptr<StartOta::Request> request,
      std::shared_ptr<StartOta::Response> response) {
      handleStartOta(request, response);
    });

  const auto publish_period_ms =
    checkedU32Parameter(*this, "status_publish_period_ms", 10, 60000);
  status_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_period_ms),
    [this]() {
      publishStatus();
    });

  RCLCPP_INFO(
    get_logger(),
    "STM32 CAN OTA skeleton started. service=/stm32_ota/start status=/stm32_ota/status");
}

OtaClient::Options Stm32OtaNode::loadClientOptions()
{
  OtaClient::Options options;
  options.can_interface = get_parameter("can_interface").as_string();

  const bool extended_id = get_parameter("extended_id").as_bool();
  const auto id_max = extended_id ? 0x1fffffff : 0x7ff;
  const auto node_id = checkedU32Parameter(*this, "node_id", 0, id_max);
  const auto request_base_id = checkedU32Parameter(*this, "request_base_id", 0, id_max);
  const auto response_base_id = checkedU32Parameter(*this, "response_base_id", 0, id_max);

  options.request_id = request_base_id + node_id;
  options.response_id = response_base_id + node_id;
  options.extended_id = extended_id;
  options.block_size = checkedU32Parameter(*this, "block_size", 1, 65536);
  options.ack_timeout = std::chrono::milliseconds(
    checkedU32Parameter(*this, "ack_timeout_ms", 1, 60000));
  options.capability_timeout = std::chrono::milliseconds(
    checkedU32Parameter(*this, "capability_timeout_ms", 1, 60000));
  options.receive_timeout = std::chrono::milliseconds(
    checkedU32Parameter(*this, "receive_timeout_ms", 1, 60000));

  if (options.request_id > static_cast<uint32_t>(id_max) ||
    options.response_id > static_cast<uint32_t>(id_max))
  {
    throw std::runtime_error("computed request_id/response_id exceeds CAN ID range");
  }

  RCLCPP_INFO(
    get_logger(),
    "OTA config: if=%s request_id=0x%X response_id=0x%X block_size=%u extended_id=%s",
    options.can_interface.c_str(),
    options.request_id,
    options.response_id,
    options.block_size,
    options.extended_id ? "true" : "false");

  return options;
}

void Stm32OtaNode::handleStartOta(
  const std::shared_ptr<StartOta::Request> request,
  std::shared_ptr<StartOta::Response> response)
{
  auto firmware_path = request->firmware_path;
  if (firmware_path.empty()) {
    firmware_path = get_parameter("default_firmware_path").as_string();
  }

  RCLCPP_INFO(
    get_logger(),
    "Start OTA requested: firmware_path='%s', version=%u, dry_run=%s",
    firmware_path.c_str(),
    request->firmware_version,
    request->dry_run ? "true" : "false");

  const bool accepted = ota_client_->start(
    firmware_path,
    request->firmware_version,
    request->dry_run);

  const auto status = ota_client_->status();
  response->accepted = accepted;
  response->state = static_cast<uint8_t>(status.state);
  response->error_code = static_cast<uint8_t>(status.error);
  response->message = status.message;
  response->resolved_firmware_path = status.firmware_path;
  response->image_size = status.image_size;
  response->image_crc32 = status.image_crc32;
  publishStatus();
}

void Stm32OtaNode::publishStatus()
{
  OtaClient::Status status;
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status = latest_status_;
  }

  status_pub_->publish(makeStatusMessage(status));
}

void Stm32OtaNode::onClientStatus(const OtaClient::Status & status)
{
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    latest_status_ = status;
  }

  RCLCPP_INFO(
    get_logger(),
    "OTA status: state=%s error=%s progress=%.2f message='%s'",
    toString(status.state).c_str(),
    toString(status.error).c_str(),
    status.progress,
    status.message.c_str());
}

Stm32OtaNode::OtaStatusMsg Stm32OtaNode::makeStatusMessage(const OtaClient::Status & status)
{
  OtaStatusMsg msg;
  msg.stamp = now();
  msg.state = static_cast<uint8_t>(status.state);
  msg.error_code = static_cast<uint8_t>(status.error);
  msg.progress = status.progress;
  msg.firmware_path = status.firmware_path;
  msg.image_size = status.image_size;
  msg.image_crc32 = status.image_crc32;
  msg.target_slot = status.capability.target_slot;
  msg.required_image = status.capability.required_image;
  msg.max_fw_size = status.capability.max_fw_size;
  msg.block_size = status.capability.block_size;
  msg.message = status.message;
  return msg;
}

}  // namespace stm32_can_ota
