#ifndef STM32_CAN_OTA__CRC16_HPP_
#define STM32_CAN_OTA__CRC16_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stm32_can_ota
{

// CRC-16/CCITT-FALSE 校验算法(多项式 0x1021，初始值 0xFFFF，不取反、不翻转)。
// 用于每个"数据块"的完整性校验(Bootloader 收到一个块后用同样的算法核对)。
// 两个重载：一个吃裸指针+长度，一个吃 vector，方便调用处任选。
uint16_t crc16CcittFalse(const uint8_t * data, size_t length, uint16_t initial = 0xffff);
uint16_t crc16CcittFalse(const std::vector<uint8_t> & data, uint16_t initial = 0xffff);

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__CRC16_HPP_
