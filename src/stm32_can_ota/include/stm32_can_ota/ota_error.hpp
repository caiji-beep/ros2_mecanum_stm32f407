#ifndef STM32_CAN_OTA__OTA_ERROR_HPP_
#define STM32_CAN_OTA__OTA_ERROR_HPP_

#include <cstdint>
#include <string>

namespace stm32_can_ota
{

// OTA 失败时的错误码枚举(成功时为 none)。
// 数值与 msg/OtaStatus.msg 中的 ERROR_* 常量一一对应。
enum class OtaError : uint8_t
{
  none = 0,                  // 无错误(成功)
  invalid_argument = 1,      // 参数非法(如固件路径为空)
  transport_open_failed = 2, // 打不开 CAN 接口(如 can0 不存在或未 up)
  transport_io_error = 3,    // CAN 收发时底层 I/O 出错
  timeout = 4,               // 等待 Bootloader 应答超时
  protocol_error = 5,        // 协议不对(收到 NACK 或非法回复帧)
  capability_rejected = 6,   // Bootloader 返回的能力不被支持(版本/块大小不合法)
  firmware_open_failed = 7,  // 读不到固件文件
  firmware_too_large = 8,    // 固件超过 Bootloader 允许的最大尺寸/块数
  unknown = 255,             // 未知错误
};

std::string toString(OtaError error);

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__OTA_ERROR_HPP_
