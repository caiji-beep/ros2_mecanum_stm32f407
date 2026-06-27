#ifndef STM32_CAN_OTA__CRC16_HPP_
#define STM32_CAN_OTA__CRC16_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stm32_can_ota
{

uint16_t crc16CcittFalse(const uint8_t * data, size_t length, uint16_t initial = 0xffff);
uint16_t crc16CcittFalse(const std::vector<uint8_t> & data, uint16_t initial = 0xffff);

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__CRC16_HPP_
