#ifndef STM32_CAN_OTA__FIRMWARE_IMAGE_HPP_
#define STM32_CAN_OTA__FIRMWARE_IMAGE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace stm32_can_ota
{

class FirmwareImage
{
public:
  bool load(const std::string & path, std::string * error = nullptr);
  void reset();

  bool empty() const;
  const std::string & path() const;
  const std::vector<uint8_t> & bytes() const;
  size_t size() const;
  uint32_t crc32() const;

  std::vector<uint8_t> readBlock(size_t offset, size_t length) const;

private:
  std::string path_;
  std::vector<uint8_t> bytes_;
  uint32_t crc32_{0};
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__FIRMWARE_IMAGE_HPP_
