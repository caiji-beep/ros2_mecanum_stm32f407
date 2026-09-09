#include "stm32_can_ota/firmware_image.hpp"

#include <algorithm>
#include <fstream>

#include "stm32_can_ota/crc32.hpp"

namespace stm32_can_ota
{

bool FirmwareImage::load(const std::string & path, std::string * error)
{
  reset();

  // 以二进制 + 定位到文件尾(ate)打开，便于用 tellg() 拿到文件大小。
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    if (error != nullptr) {
      *error = "failed to open firmware image: " + path;
    }
    return false;
  }

  const auto size = file.tellg();
  if (size <= 0) {
    if (error != nullptr) {
      *error = "firmware image is empty: " + path;
    }
    return false;
  }

  // 按文件大小分配缓冲区，回到文件头，一次性读入内存。
  bytes_.resize(static_cast<size_t>(size));
  file.seekg(0, std::ios::beg);

  if (!file.read(
      reinterpret_cast<char *>(bytes_.data()),
      static_cast<std::streamsize>(bytes_.size())))
  {
    reset();
    if (error != nullptr) {
      *error = "failed to read firmware image: " + path;
    }
    return false;
  }

  // 读成功后记录路径，并算整包 CRC32(升级结束 Bootloader 用同样算法核对整包)。
  path_ = path;
  crc32_ = crc32Ieee(bytes_);
  return true;
}

void FirmwareImage::reset()
{
  path_.clear();
  bytes_.clear();
  crc32_ = 0;
}

bool FirmwareImage::empty() const
{
  return bytes_.empty();
}

const std::string & FirmwareImage::path() const
{
  return path_;
}

const std::vector<uint8_t> & FirmwareImage::bytes() const
{
  return bytes_;
}

size_t FirmwareImage::size() const
{
  return bytes_.size();
}

uint32_t FirmwareImage::crc32() const
{
  return crc32_;
}

// 从 offset 处取 length 字节作为一个"块"的副本。
// 这是升级切块的关键：ota_client 会循环调用它按 block_size 把固件切成多块。
// 若 offset 越界返回空；若剩余不足 length 则只返回到文件末尾的部分(最后一块)。
std::vector<uint8_t> FirmwareImage::readBlock(size_t offset, size_t length) const
{
  if (offset >= bytes_.size()) {
    return {};
  }

  const auto end = std::min(bytes_.size(), offset + length);
  return std::vector<uint8_t>(
    bytes_.begin() + static_cast<std::ptrdiff_t>(offset),
    bytes_.begin() + static_cast<std::ptrdiff_t>(end));
}

}  // namespace stm32_can_ota
