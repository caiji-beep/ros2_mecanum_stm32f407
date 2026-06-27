#ifndef STM32_CAN_OTA__OTA_PROTOCOL_HPP_
#define STM32_CAN_OTA__OTA_PROTOCOL_HPP_

#include <cstdint>
#include <string>

#include "stm32_can_ota/can_frame.hpp"

namespace stm32_can_ota
{

enum class OtaCommand : uint8_t
{
  sync = 0x01,
  get_capability = 0x02,
  start = 0x10,
  block_begin = 0x20,
  data = 0x21,
  end = 0x30,
  ack = 0x79,
  nack = 0x1f,
};

struct BootloaderCapability
{
  uint8_t protocol_version{1};
  uint8_t target_slot{0};
  std::string required_image{"app_A.bin"};
  uint32_t max_fw_size{0};
  uint32_t block_size{256};
};

class OtaProtocol
{
public:
  struct Options
  {
    uint32_t request_id{0x601};
    uint32_t response_id{0x581};
    bool extended_id{false};
  };

  explicit OtaProtocol(Options options);

  CanFrame buildSync() const;
  CanFrame buildGetCapability() const;
  CanFrame buildStart(uint32_t image_size, uint32_t image_crc32, uint32_t firmware_version) const;
  CanFrame buildBlockBegin(uint32_t block_index, uint32_t block_size) const;
  CanFrame buildData(uint16_t sequence, const uint8_t * data, uint8_t length) const;
  CanFrame buildEnd() const;

  bool isAck(const CanFrame & frame, OtaCommand expected_command) const;
  bool isNack(const CanFrame & frame, OtaCommand expected_command) const;
  bool parseCapability(const CanFrame & frame, BootloaderCapability & capability) const;

  const Options & options() const;

private:
  CanFrame makeCommandFrame(OtaCommand command) const;

  Options options_;
};

std::string toString(OtaCommand command);

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__OTA_PROTOCOL_HPP_
