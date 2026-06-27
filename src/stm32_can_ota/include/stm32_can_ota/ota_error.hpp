#ifndef STM32_CAN_OTA__OTA_ERROR_HPP_
#define STM32_CAN_OTA__OTA_ERROR_HPP_

#include <cstdint>
#include <string>

namespace stm32_can_ota
{

enum class OtaError : uint8_t
{
  none = 0,
  invalid_argument = 1,
  transport_open_failed = 2,
  transport_io_error = 3,
  timeout = 4,
  protocol_error = 5,
  capability_rejected = 6,
  firmware_open_failed = 7,
  firmware_too_large = 8,
  not_implemented = 9,
  unknown = 255,
};

std::string toString(OtaError error);

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__OTA_ERROR_HPP_
