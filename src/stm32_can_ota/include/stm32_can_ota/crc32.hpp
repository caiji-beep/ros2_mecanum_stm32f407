#ifndef STM32_CAN_OTA__CRC32_HPP_
#define STM32_CAN_OTA__CRC32_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stm32_can_ota
{

uint32_t crc32Ieee(const uint8_t * data, size_t length, uint32_t initial = 0xffffffff);
uint32_t crc32Ieee(const std::vector<uint8_t> & data, uint32_t initial = 0xffffffff);

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__CRC32_HPP_
