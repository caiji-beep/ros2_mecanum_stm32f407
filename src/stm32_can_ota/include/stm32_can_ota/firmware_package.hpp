#ifndef STM32_CAN_OTA__FIRMWARE_PACKAGE_HPP_
#define STM32_CAN_OTA__FIRMWARE_PACKAGE_HPP_

#include <cstdint>
#include <string>

#include "stm32_can_ota/firmware_image.hpp"

namespace stm32_can_ota
{

// 固件"清单"：描述这个固件包该刷到哪个镜像槽(app_A.bin/app_B.bin)、版本号是多少。
// 这是为了支持 A/B 双分区升级(两个固件镜像槽)，告诉 Bootloader 该写哪一边。
struct FirmwarePackageManifest
{
  std::string package_path;                 // 清单文件本身路径
  std::string image_path;                   // 实际 .bin 固件路径
  std::string required_image{"app_A.bin"};  // 期望烧写的镜像槽名(默认 app_A)
  uint32_t firmware_version{0};             // 固件版本号
};

// 固件包： = 一个 FirmwareImage(实际数据) + 一个 FirmwarePackageManifest(元信息)。
// 提供两种加载方式：直接给 .bin；或给一个 yaml/json 清单再去读 .bin。
class FirmwarePackage
{
public:
  // 方式一：直接加载单个 .bin 文件(清单用默认值：app_A、版本 0)
  bool loadSingleImage(const std::string & image_path, std::string * error = nullptr);
  // 方式二：从清单文件加载(清单里指明 image_path / required_image / firmware_version)
  bool loadManifest(const std::string & manifest_path, std::string * error = nullptr);

  const FirmwareImage & image() const;              // 取固件数据
  const FirmwarePackageManifest & manifest() const; // 取清单元信息
  bool hasImage() const;                            // 是否已成功加载固件

private:
  FirmwareImage image_;
  FirmwarePackageManifest manifest_;
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__FIRMWARE_PACKAGE_HPP_
