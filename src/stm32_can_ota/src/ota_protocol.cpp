#include "stm32_can_ota/ota_protocol.hpp"

#include <algorithm>
#include <cstddef>
#include <sstream>

namespace stm32_can_ota
{

// 匿名命名空间：仅本文件使用的"小工具"函数。
namespace
{

// 把 16 位整数按"小端"写入 data[offset..offset+1]。
// 小端 = 低字节在前(STM32 普遍用这种字节序)。
void putU16(std::array<uint8_t, 8> & data, size_t offset, uint16_t value)
{
  data.at(offset) = static_cast<uint8_t>(value & 0xffU);
  data.at(offset + 1U) = static_cast<uint8_t>((value >> 8U) & 0xffU);
}

// 把 32 位整数按小端写入 data[offset..offset+3]。
void putU32(std::array<uint8_t, 8> & data, size_t offset, uint32_t value)
{
  data.at(offset) = static_cast<uint8_t>(value & 0xffU);
  data.at(offset + 1U) = static_cast<uint8_t>((value >> 8U) & 0xffU);
  data.at(offset + 2U) = static_cast<uint8_t>((value >> 16U) & 0xffU);
  data.at(offset + 3U) = static_cast<uint8_t>((value >> 24U) & 0xffU);
}

// 小端读出 32 位整数。
uint32_t getU32(const std::array<uint8_t, 8> & data, size_t offset)
{
  return static_cast<uint32_t>(data.at(offset)) |
         (static_cast<uint32_t>(data.at(offset + 1U)) << 8U) |
         (static_cast<uint32_t>(data.at(offset + 2U)) << 16U) |
         (static_cast<uint32_t>(data.at(offset + 3U)) << 24U);
}

// 小端读出 16 位整数。
uint16_t getU16(const std::array<uint8_t, 8> & data, size_t offset)
{
  return static_cast<uint16_t>(data.at(offset)) |
         static_cast<uint16_t>(static_cast<uint16_t>(data.at(offset + 1U)) << 8U);
}

}  // namespace

OtaProtocol::OtaProtocol(Options options)
: options_(options)
{
}

CanFrame OtaProtocol::buildSync() const
{
  return makeCommandFrame(OtaCommand::sync);
}

CanFrame OtaProtocol::buildGetCapability(uint8_t page) const
{
  auto frame = makeCommandFrame(OtaCommand::get_capability);
  frame.data[1] = page;
  return frame;
}

CanFrame OtaProtocol::buildStart(uint16_t session_id, uint32_t image_size) const
{
  auto frame = makeCommandFrame(OtaCommand::start);
  putU16(frame.data, 1, session_id);
  putU32(frame.data, 3, image_size);
  return frame;
}

CanFrame OtaProtocol::buildStartCrc(uint16_t session_id, uint32_t image_crc32) const
{
  auto frame = makeCommandFrame(OtaCommand::start_crc);
  putU16(frame.data, 1, session_id);
  putU32(frame.data, 3, image_crc32);
  return frame;
}

CanFrame OtaProtocol::buildStartVersion(uint16_t session_id, uint32_t firmware_version) const
{
  auto frame = makeCommandFrame(OtaCommand::start_version);
  putU16(frame.data, 1, session_id);
  putU32(frame.data, 3, firmware_version);
  return frame;
}

CanFrame OtaProtocol::buildBlockBegin(
  uint16_t session_id, uint16_t block_index, uint16_t block_size) const
{
  auto frame = makeCommandFrame(OtaCommand::block_begin);
  putU16(frame.data, 1, session_id);
  putU16(frame.data, 3, block_index);
  putU16(frame.data, 5, block_size);
  return frame;
}

CanFrame OtaProtocol::buildData(uint8_t sequence, const uint8_t * data, uint8_t length) const
{
  auto frame = makeCommandFrame(OtaCommand::data);
  // 数据帧布局(8字节限制)：
  //   [0]=命令0x21  [1]=本帧序号  [2]=本帧数据长度  [3..7]=最多5字节固件数据
  // 因为一帧最多8字节扣掉3字节头，真正能装的数据只有 5 字节，
  // 所以一个 256 字节的块需要 ceil(256/5)=52 帧 DATA 才能发完。
  const auto copy_length = std::min<uint8_t>(length, 5U);
  frame.data[1] = sequence;
  frame.data[2] = copy_length;
  std::copy_n(data, copy_length, frame.data.begin() + 3);
  frame.dlc = static_cast<uint8_t>(3U + copy_length);
  return frame;
}

CanFrame OtaProtocol::buildBlockEnd(
  uint16_t session_id, uint16_t block_index, uint16_t block_crc16) const
{
  auto frame = makeCommandFrame(OtaCommand::block_end);
  putU16(frame.data, 1, session_id);
  putU16(frame.data, 3, block_index);
  putU16(frame.data, 5, block_crc16);
  return frame;
}

CanFrame OtaProtocol::buildEnd(uint16_t session_id, uint32_t image_crc32) const
{
  auto frame = makeCommandFrame(OtaCommand::end);
  putU16(frame.data, 1, session_id);
  putU32(frame.data, 3, image_crc32);
  return frame;
}

CanFrame OtaProtocol::buildAbort(uint16_t session_id) const
{
  auto frame = makeCommandFrame(OtaCommand::abort);
  putU16(frame.data, 1, session_id);
  return frame;
}

OtaReply OtaProtocol::parseReply(const CanFrame & frame) const
{
  OtaReply reply;
  // 回复帧布局(STM32→主机, ID=0x581)：
  //   [0]=0x79(ACK)/0x1f(NACK)  [1]=对应的命令号  [2]=状态码(0=OK)
  //   [3..4]=session_id  [5..6]=detail  [7]=期望的下一数据帧序号
  // 先过滤：不是预期 ID/帧类型，或长度不足 8，直接判为无效。
  if (frame.id != options_.response_id || frame.extended != options_.extended_id || frame.dlc < 8) {
    return reply;
  }

  const auto marker = frame.data[0];
  if (marker != static_cast<uint8_t>(OtaCommand::ack) &&
    marker != static_cast<uint8_t>(OtaCommand::nack))
  {
    return reply;
  }

  reply.valid = true;
  // positive(成功) = 是 ACK 且状态码为 0
  reply.positive = marker == static_cast<uint8_t>(OtaCommand::ack) && frame.data[2] == 0;
  reply.command = static_cast<OtaCommand>(frame.data[1]);  // 这条应答对应的是哪个命令
  reply.status = frame.data[2];                            // 状态码(NACK 时表示原因)
  reply.session_id = getU16(frame.data, 3);                // 会话 ID
  reply.detail = getU16(frame.data, 5);                    // 附加信息
  reply.next_sequence = frame.data[7];                     // 期望的下个数据帧序号(重传用)
  return reply;
}

bool OtaProtocol::isAck(
  const CanFrame & frame, OtaCommand expected_command, uint16_t expected_session) const
{
  const auto reply = parseReply(frame);
  return reply.valid && reply.positive && reply.command == expected_command &&
         reply.session_id == expected_session;
}

bool OtaProtocol::isNack(
  const CanFrame & frame, OtaCommand expected_command, uint16_t expected_session) const
{
  const auto reply = parseReply(frame);
  return reply.valid && !reply.positive && reply.command == expected_command &&
         reply.session_id == expected_session;
}

bool OtaProtocol::parseCapability(
  const CanFrame & frame, uint8_t expected_page, BootloaderCapability & capability) const
{
  if (frame.id != options_.response_id || frame.extended != options_.extended_id ||
    frame.dlc < 8 || frame.data[0] != static_cast<uint8_t>(OtaCommand::ack) ||
    frame.data[1] != static_cast<uint8_t>(OtaCommand::get_capability) ||
    frame.data[2] != 0 || frame.data[3] != expected_page)
  {
    return false;
  }

  if (expected_page == 0) {
    capability.protocol_version = frame.data[4];
    capability.target_slot = frame.data[5];
    capability.block_size = getU16(frame.data, 6);
  } else if (expected_page == 1) {
    capability.max_fw_size = getU32(frame.data, 4);
  } else {
    return false;
  }
  return true;
}

const OtaProtocol::Options & OtaProtocol::options() const
{
  return options_;
}

CanFrame OtaProtocol::makeCommandFrame(OtaCommand command) const
{
  CanFrame frame;
  frame.id = options_.request_id;
  frame.extended = options_.extended_id;
  frame.dlc = 8;
  frame.data.fill(0);
  frame.data[0] = static_cast<uint8_t>(command);
  return frame;
}

std::string toString(OtaCommand command)
{
  switch (command) {
    case OtaCommand::sync:
      return "SYNC";
    case OtaCommand::get_capability:
      return "GET_CAPABILITY";
    case OtaCommand::start:
      return "START";
    case OtaCommand::start_crc:
      return "START_CRC";
    case OtaCommand::start_version:
      return "START_VERSION";
    case OtaCommand::block_begin:
      return "BLOCK_BEGIN";
    case OtaCommand::data:
      return "DATA";
    case OtaCommand::block_end:
      return "BLOCK_END";
    case OtaCommand::end:
      return "END";
    case OtaCommand::abort:
      return "ABORT";
    case OtaCommand::ack:
      return "ACK";
    case OtaCommand::nack:
      return "NACK";
  }

  std::ostringstream oss;
  oss << "UNKNOWN_COMMAND_" << static_cast<int>(command);
  return oss.str();
}

}  // namespace stm32_can_ota
