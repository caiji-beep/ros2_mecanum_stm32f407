#ifndef STM32_CAN_OTA__CAN_FRAME_HPP_
#define STM32_CAN_OTA__CAN_FRAME_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace stm32_can_ota
{

struct CanFrame
{
  uint32_t id{0};
  bool extended{false};
  bool remote{false};
  uint8_t dlc{0};
  std::array<uint8_t, 8> data{};

  std::string toString() const;
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__CAN_FRAME_HPP_
