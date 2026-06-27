#include "stm32_can_ota/crc32.hpp"

namespace stm32_can_ota
{

uint32_t crc32Ieee(const uint8_t * data, size_t length, uint32_t initial)
{
  uint32_t crc = initial;

  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint32_t>(data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 1U) != 0U) {
        crc = (crc >> 1U) ^ 0xedb88320U;
      } else {
        crc >>= 1U;
      }
    }
  }

  return ~crc;
}

uint32_t crc32Ieee(const std::vector<uint8_t> & data, uint32_t initial)
{
  return crc32Ieee(data.data(), data.size(), initial);
}

}  // namespace stm32_can_ota
