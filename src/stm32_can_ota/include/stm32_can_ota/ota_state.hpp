#ifndef STM32_CAN_OTA__OTA_STATE_HPP_
#define STM32_CAN_OTA__OTA_STATE_HPP_

#include <cstdint>
#include <string>

namespace stm32_can_ota
{

enum class OtaState : uint8_t
{
  idle = 0,
  syncing = 1,
  query_capability = 2,
  ready = 3,
  starting = 4,
  transferring = 5,
  verifying = 6,
  completed = 7,
  failed = 8,
  cancelled = 9,
};

std::string toString(OtaState state);

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__OTA_STATE_HPP_
