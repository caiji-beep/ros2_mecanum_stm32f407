#ifndef STM32_CAN_OTA__OTA_CLIENT_HPP_
#define STM32_CAN_OTA__OTA_CLIENT_HPP_

#include <chrono>
#include <functional>
#include <mutex>
#include <string>

#include "stm32_can_ota/can_transport.hpp"
#include "stm32_can_ota/firmware_package.hpp"
#include "stm32_can_ota/ota_error.hpp"
#include "stm32_can_ota/ota_protocol.hpp"
#include "stm32_can_ota/ota_state.hpp"

namespace stm32_can_ota
{

class OtaClient
{
public:
  struct Options
  {
    std::string can_interface{"can0"};
    uint32_t request_id{0x601};
    uint32_t response_id{0x581};
    bool extended_id{false};
    uint32_t block_size{256};
    std::chrono::milliseconds ack_timeout{1000};
    std::chrono::milliseconds capability_timeout{1000};
    std::chrono::milliseconds receive_timeout{100};
  };

  struct Status
  {
    OtaState state{OtaState::idle};
    OtaError error{OtaError::none};
    float progress{0.0F};
    std::string message;
    std::string firmware_path;
    uint32_t image_size{0};
    uint32_t image_crc32{0};
    BootloaderCapability capability;
  };

  using StatusCallback = std::function<void(const Status &)>;

  explicit OtaClient(Options options);

  void setStatusCallback(StatusCallback callback);
  const Options & options() const;
  Status status() const;

  bool start(const std::string & firmware_path, uint32_t firmware_version, bool dry_run);
  void cancel();

private:
  void updateStatus(
    OtaState state,
    OtaError error,
    float progress,
    const std::string & message);

  Options options_;
  mutable std::mutex mutex_;
  Status status_;
  StatusCallback status_callback_;
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__OTA_CLIENT_HPP_
