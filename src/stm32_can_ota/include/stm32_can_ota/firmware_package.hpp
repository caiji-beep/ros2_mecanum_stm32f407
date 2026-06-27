#ifndef STM32_CAN_OTA__FIRMWARE_PACKAGE_HPP_
#define STM32_CAN_OTA__FIRMWARE_PACKAGE_HPP_

#include <cstdint>
#include <string>

#include "stm32_can_ota/firmware_image.hpp"

namespace stm32_can_ota
{

struct FirmwarePackageManifest
{
  std::string package_path;
  std::string image_path;
  std::string required_image{"app_A.bin"};
  uint32_t firmware_version{0};
};

class FirmwarePackage
{
public:
  bool loadSingleImage(const std::string & image_path, std::string * error = nullptr);
  bool loadManifest(const std::string & manifest_path, std::string * error = nullptr);

  const FirmwareImage & image() const;
  const FirmwarePackageManifest & manifest() const;
  bool hasImage() const;

private:
  FirmwareImage image_;
  FirmwarePackageManifest manifest_;
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__FIRMWARE_PACKAGE_HPP_
