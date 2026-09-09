#ifndef STM32_CAN_OTA__FIRMWARE_IMAGE_HPP_
#define STM32_CAN_OTA__FIRMWARE_IMAGE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace stm32_can_ota
{

// 把固件二进制文件(.bin)读进内存，并提供"切块"和"整包 CRC32"计算。
// 它是升级过程中的"数据源"：ota_client 从这里按块取数据发给 STM32。
class FirmwareImage
{
public:
  // 从磁盘读取固件文件到 bytes_，并顺便算好整包 crc32_。失败返回 false，error 写入原因。
  bool load(const std::string & path, std::string * error = nullptr);
  void reset();  // 清空已加载内容，回到空状态

  bool empty() const;                  // 是否还没加载任何数据
  const std::string & path() const;    // 返回加载时用的文件路径
  const std::vector<uint8_t> & bytes() const;  // 返回全部固件字节(只读)
  size_t size() const;                 // 固件总字节数
  uint32_t crc32() const;              // 整包 CRC32(STM32 端校验整包时用)

  // 从 offset 处取 length 字节作为一个"块"的副本，越界时只返回可用的部分。
  // 升级时 ota_client 反复调用它按 block_size 切分固件。
  std::vector<uint8_t> readBlock(size_t offset, size_t length) const;

private:
  std::string path_;
  std::vector<uint8_t> bytes_;
  uint32_t crc32_{0};
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__FIRMWARE_IMAGE_HPP_
