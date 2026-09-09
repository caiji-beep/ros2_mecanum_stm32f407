#ifndef STM32_CAN_OTA__CAN_FRAME_HPP_
#define STM32_CAN_OTA__CAN_FRAME_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace stm32_can_ota
{

// 一个标准 CAN 2.0 数据帧的抽象表示（不含时间等元数据）。
// CAN 帧最多 8 字节数据，因此 data 是固定长度数组。
struct CanFrame
{
  uint32_t id{0};        // CAN 标识符(ID)：标准帧 11 位(0~0x7FF)，扩展帧 29 位
  bool extended{false};  // true = 扩展帧(29位ID)；false = 标准帧(11位ID)
  bool remote{false};    // true = 远程帧(RTR，用于请求数据，不含数据负载)；false = 数据帧
  uint8_t dlc{0};        // Data Length Code：数据字节数，范围 0~8
  std::array<uint8_t, 8> data{};  // 数据负载，最多 8 字节

  std::string toString() const;  // 调试用：把帧打印成 "id=0x601 std dlc=8 data=[...]"
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__CAN_FRAME_HPP_
