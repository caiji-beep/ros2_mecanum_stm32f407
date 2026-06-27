#include "stm32_can_ota/firmware_package.hpp"

namespace stm32_can_ota
{

bool FirmwarePackage::loadSingleImage(const std::string & image_path, std::string * error)
{
  FirmwarePackageManifest manifest;
  manifest.package_path = image_path;
  manifest.image_path = image_path;
  manifest.required_image = "app_A.bin";

  FirmwareImage image;
  if (!image.load(image_path, error)) {
    return false;
  }

  manifest_ = manifest;
  image_ = image;
  return true;
}

bool FirmwarePackage::loadManifest(const std::string & manifest_path, std::string * error)
{
  if (error != nullptr) {
    *error = "manifest loading is reserved for a later version: " + manifest_path;
  }
  return false;
}

const FirmwareImage & FirmwarePackage::image() const
{
  return image_;
}

const FirmwarePackageManifest & FirmwarePackage::manifest() const
{
  return manifest_;
}

bool FirmwarePackage::hasImage() const
{
  return !image_.empty();
}

}  // namespace stm32_can_ota
