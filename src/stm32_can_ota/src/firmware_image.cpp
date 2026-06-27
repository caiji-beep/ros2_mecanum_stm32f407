#include "stm32_can_ota/firmware_image.hpp"

#include <algorithm>
#include <fstream>

#include "stm32_can_ota/crc32.hpp"

namespace stm32_can_ota
{

bool FirmwareImage::load(const std::string & path, std::string * error)
{
  reset();

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

  bytes_.resize(static_cast<size_t>(size));
  file.seekg(0, std::ios::beg);

  if (!file.read(reinterpret_cast<char *>(bytes_.data()), static_cast<std::streamsize>(bytes_.size()))) {
    reset();
    if (error != nullptr) {
      *error = "failed to read firmware image: " + path;
    }
    return false;
  }

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

std::vector<uint8_t> FirmwareImage::readBlock(size_t offset, size_t length) const
{
  if (offset >= bytes_.size()) {
    return {};
  }

  const auto end = std::min(bytes_.size(), offset + length);
  return std::vector<uint8_t>(bytes_.begin() + static_cast<std::ptrdiff_t>(offset),
    bytes_.begin() + static_cast<std::ptrdiff_t>(end));
}

}  // namespace stm32_can_ota
