#include "stm32_can_ota/crc16.hpp"

namespace stm32_can_ota
{

uint16_t crc16CcittFalse(const uint8_t * data, size_t length, uint16_t initial)
{
  uint16_t crc = initial;

  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
      } else {
        crc = static_cast<uint16_t>(crc << 1U);
      }
    }
  }

  return crc;
}

uint16_t crc16CcittFalse(const std::vector<uint8_t> & data, uint16_t initial)
{
  return crc16CcittFalse(data.data(), data.size(), initial);
}

}  // namespace stm32_can_ota
