#include "stm32_can_ota/ota_protocol.hpp"

#include <algorithm>
#include <cstddef>
#include <sstream>

namespace stm32_can_ota
{

namespace
{

void putU16(std::array<uint8_t, 8> & data, size_t offset, uint16_t value)
{
  data.at(offset) = static_cast<uint8_t>(value & 0xffU);
  data.at(offset + 1U) = static_cast<uint8_t>((value >> 8U) & 0xffU);
}

void putU32(std::array<uint8_t, 8> & data, size_t offset, uint32_t value)
{
  data.at(offset) = static_cast<uint8_t>(value & 0xffU);
  data.at(offset + 1U) = static_cast<uint8_t>((value >> 8U) & 0xffU);
  data.at(offset + 2U) = static_cast<uint8_t>((value >> 16U) & 0xffU);
  data.at(offset + 3U) = static_cast<uint8_t>((value >> 24U) & 0xffU);
}

uint32_t getU32(const std::array<uint8_t, 8> & data, size_t offset)
{
  return static_cast<uint32_t>(data.at(offset)) |
         (static_cast<uint32_t>(data.at(offset + 1U)) << 8U) |
         (static_cast<uint32_t>(data.at(offset + 2U)) << 16U) |
         (static_cast<uint32_t>(data.at(offset + 3U)) << 24U);
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

CanFrame OtaProtocol::buildGetCapability() const
{
  return makeCommandFrame(OtaCommand::get_capability);
}

CanFrame OtaProtocol::buildStart(
  uint32_t image_size,
  uint32_t image_crc32,
  uint32_t firmware_version) const
{
  auto frame = makeCommandFrame(OtaCommand::start);
  putU32(frame.data, 1, image_size);
  putU16(frame.data, 5, static_cast<uint16_t>(firmware_version & 0xffffU));
  frame.data[7] = static_cast<uint8_t>((image_crc32 >> 24U) & 0xffU);
  return frame;
}

CanFrame OtaProtocol::buildBlockBegin(uint32_t block_index, uint32_t block_size) const
{
  auto frame = makeCommandFrame(OtaCommand::block_begin);
  putU32(frame.data, 1, block_index);
  putU16(frame.data, 5, static_cast<uint16_t>(block_size & 0xffffU));
  return frame;
}

CanFrame OtaProtocol::buildData(uint16_t sequence, const uint8_t * data, uint8_t length) const
{
  auto frame = makeCommandFrame(OtaCommand::data);
  putU16(frame.data, 1, sequence);
  const auto copy_length = std::min<uint8_t>(length, 5U);
  std::copy_n(data, copy_length, frame.data.begin() + 3);
  return frame;
}

CanFrame OtaProtocol::buildEnd() const
{
  return makeCommandFrame(OtaCommand::end);
}

bool OtaProtocol::isAck(const CanFrame & frame, OtaCommand expected_command) const
{
  return frame.id == options_.response_id &&
         frame.dlc >= 2 &&
         frame.data[0] == static_cast<uint8_t>(OtaCommand::ack) &&
         frame.data[1] == static_cast<uint8_t>(expected_command);
}

bool OtaProtocol::isNack(const CanFrame & frame, OtaCommand expected_command) const
{
  return frame.id == options_.response_id &&
         frame.dlc >= 2 &&
         frame.data[0] == static_cast<uint8_t>(OtaCommand::nack) &&
         frame.data[1] == static_cast<uint8_t>(expected_command);
}

bool OtaProtocol::parseCapability(const CanFrame & frame, BootloaderCapability & capability) const
{
  if (!isAck(frame, OtaCommand::get_capability) || frame.dlc < 8) {
    return false;
  }

  capability.protocol_version = frame.data[2];
  capability.target_slot = frame.data[3];
  capability.max_fw_size = getU32(frame.data, 4);

  // The full capability layout is intentionally left open. The host should
  // keep treating target details as bootloader-owned data.
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
    case OtaCommand::block_begin:
      return "BLOCK_BEGIN";
    case OtaCommand::data:
      return "DATA";
    case OtaCommand::end:
      return "END";
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
