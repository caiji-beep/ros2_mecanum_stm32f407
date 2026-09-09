#ifndef STM32_CAN_OTA__OTA_STATE_HPP_
#define STM32_CAN_OTA__OTA_STATE_HPP_

#include <cstdint>
#include <string>

namespace stm32_can_ota
{

// OTA 升级全过程的状态机枚举，反映当前进展到哪一步。
// 数值与 msg/OtaStatus.msg 中的 STATE_* 常量一一对应，便于在 ROS 消息里编码传输。
enum class OtaState : uint8_t
{
  idle = 0,             // 空闲，未开始
  syncing = 1,          // 正在与 STM32 Bootloader 做 SYNC 握手同步
  query_capability = 2, // 正在询问 Bootloader 的能力(块大小/最大固件等)
  ready = 3,            // 已就绪(或 dry_run 完成)，准备/已经开始传输
  starting = 4,         // 正在发送固件元数据(START/START_CRC/START_VERSION)
  transferring = 5,     // 正在逐块传输固件数据
  verifying = 6,        // 所有块发完，正在发送 END 让 Bootloader 校验整包
  completed = 7,        // 升级成功完成
  failed = 8,           // 升级失败(见 OtaError)
  cancelled = 9,        // 被用户取消
};

std::string toString(OtaState state);

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__OTA_STATE_HPP_
