#ifndef STM32_CAN_OTA__OTA_PROTOCOL_HPP_
#define STM32_CAN_OTA__OTA_PROTOCOL_HPP_

#include <cstdint>
#include <string>

#include "stm32_can_ota/can_frame.hpp"

namespace stm32_can_ota
{

// OTA 命令号(=CAN 帧第 0 字节的"功能码")。
// 主机发 0x01~0x31 的"请求"，STM32 回 0x79(ACK 成功) 或 0x1f(NACK 失败)。
enum class OtaCommand : uint8_t
{
  sync = 0x01,           // 同步握手：确认 Bootloader 在线
  get_capability = 0x02, // 询问 Bootloader 能力(分两页)
  start = 0x10,          // 开始：告知固件总大小
  start_crc = 0x11,      // 告知整包 CRC32
  start_version = 0x12,  // 告知固件版本号
  block_begin = 0x20,    // 标志一个数据块开始
  data = 0x21,           // 数据帧(实际固件内容，每帧最多 5 字节)
  block_end = 0x22,      // 本块结束 + 块 CRC16
  end = 0x30,            // 全部发完，请求整包校验
  abort = 0x31,          // 中止升级
  ack = 0x79,            // 应答：成功(STM32 发给主机)
  nack = 0x1f,           // 应答：失败(STM32 发给主机)
};

// Bootloader 自报的能力，来自 GET_CAPABILITY 的回复。
struct BootloaderCapability
{
  uint8_t protocol_version{1};      // 协议版本(需与主机匹配)
  uint8_t target_slot{0};           // 建议烧写的镜像槽
  std::string required_image{"app_A.bin"};  // 期望的镜像文件名(A/B 双分区)
  uint32_t max_fw_size{0};          // 允许的最大固件字节数
  uint32_t block_size{256};         // 单个数据块大小(主机据此切块)
};

// 把 STM32 回的一帧解析成"语义化"的应答结构。
struct OtaReply
{
  bool valid{false};          // 这帧能否被解析为合法 OTA 应答
  bool positive{false};       // 是否为 ACK(true) / NACK(false)
  OtaCommand command{OtaCommand::sync};  // 这条应答对应的是哪个命令
  uint8_t status{0};          // 状态码(NACK 时表示失败原因)
  uint16_t session_id{0};     // 会话 ID(用于校验是否同一次升级)
  uint16_t detail{0};         // 附加信息(如能力字段)
  uint8_t next_sequence{0};   // 期望的下一个数据帧序号(丢包重传用)
};

// 协议编解码器：负责把"高级命令"打包成 CAN 帧，并把回复帧解析成 OtaReply。
// 它只做"帧格式"转换，不关心流程时序(时序在 ota_client)。
class OtaProtocol
{
public:
  struct Options
  {
    uint32_t request_id{0x601};   // 主机 → STM32 的 CAN ID(标准帧)
    uint32_t response_id{0x581};  // STM32 → 主机的 CAN ID
    bool extended_id{false};      // 是否用 29 位扩展帧
  };

  explicit OtaProtocol(Options options);

  // —— 以下 buildXxx：构造"请求帧"(主机发出) ——
  CanFrame buildSync() const;
  CanFrame buildGetCapability(uint8_t page) const;  // page=0/1：能力分两页回传
  CanFrame buildStart(uint16_t session_id, uint32_t image_size) const;
  CanFrame buildStartCrc(uint16_t session_id, uint32_t image_crc32) const;
  CanFrame buildStartVersion(uint16_t session_id, uint32_t firmware_version) const;
  CanFrame buildBlockBegin(
    uint16_t session_id, uint16_t block_index, uint16_t block_size) const;
  CanFrame buildData(uint8_t sequence, const uint8_t * data, uint8_t length) const;
  CanFrame buildBlockEnd(
    uint16_t session_id, uint16_t block_index, uint16_t block_crc16) const;
  CanFrame buildEnd(uint16_t session_id, uint32_t image_crc32) const;
  CanFrame buildAbort(uint16_t session_id) const;

  // —— 以下 parse/isXxx：解析"回复帧"(STM32 发回) ——
  OtaReply parseReply(const CanFrame & frame) const;          // 通用解析成 OtaReply
  bool isAck(                                            // 判断是否为期望命令的 ACK
    const CanFrame & frame, OtaCommand expected_command, uint16_t expected_session) const;
  bool isNack(                                            // 判断是否为期望命令的 NACK
    const CanFrame & frame, OtaCommand expected_command, uint16_t expected_session) const;
  bool parseCapability(                                  // 解析 GET_CAPABILITY 回复
    const CanFrame & frame, uint8_t expected_page, BootloaderCapability & capability) const;

  const Options & options() const;

private:
  CanFrame makeCommandFrame(OtaCommand command) const;  // 内部：造一个只填了命令号的空帧

  Options options_;
};

std::string toString(OtaCommand command);

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__OTA_PROTOCOL_HPP_
