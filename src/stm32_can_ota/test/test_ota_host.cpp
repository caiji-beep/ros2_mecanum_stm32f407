#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <unistd.h>

#include "stm32_can_ota/crc16.hpp"
#include "stm32_can_ota/crc32.hpp"
#include "stm32_can_ota/ota_client.hpp"
#include "stm32_can_ota/ota_protocol.hpp"

namespace stm32_can_ota
{
namespace
{

uint16_t readU16(const std::array<uint8_t, 8> & data, size_t offset)
{
  return static_cast<uint16_t>(data[offset]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[offset + 1]) << 8U);
}

uint32_t readU32(const std::array<uint8_t, 8> & data, size_t offset)
{
  return static_cast<uint32_t>(data[offset]) |
         (static_cast<uint32_t>(data[offset + 1]) << 8U) |
         (static_cast<uint32_t>(data[offset + 2]) << 16U) |
         (static_cast<uint32_t>(data[offset + 3]) << 24U);
}

void writeU16(std::array<uint8_t, 8> & data, size_t offset, uint16_t value)
{
  data[offset] = static_cast<uint8_t>(value & 0xffU);
  data[offset + 1] = static_cast<uint8_t>((value >> 8U) & 0xffU);
}

void writeU32(std::array<uint8_t, 8> & data, size_t offset, uint32_t value)
{
  data[offset] = static_cast<uint8_t>(value & 0xffU);
  data[offset + 1] = static_cast<uint8_t>((value >> 8U) & 0xffU);
  data[offset + 2] = static_cast<uint8_t>((value >> 16U) & 0xffU);
  data[offset + 3] = static_cast<uint8_t>((value >> 24U) & 0xffU);
}

struct FakeState
{
  std::mutex mutex;
  std::vector<CanFrame> replies;
  std::vector<uint8_t> current_block;
  std::vector<uint8_t> received_image;
  uint16_t session_id{0};
  uint16_t block_index{0};
  uint32_t block_begin_count{0};
  uint32_t data_frame_count{0};
  bool nack_first_block_once{true};
  bool suppress_replies{false};
  bool final_crc_matched{false};
  uint32_t final_crc_received{0};
  uint32_t final_crc_calculated{0};
};

class FakeTransport final : public ICanTransport
{
public:
  explicit FakeTransport(std::shared_ptr<FakeState> state)
  : state_(std::move(state)) {}

  bool open(std::string *) override
  {
    open_ = true;
    return true;
  }

  void close() override {open_ = false;}
  bool isOpen() const override {return open_;}

  bool send(const CanFrame & frame, std::string * error) override
  {
    if (!open_) {
      if (error) {
        *error = "fake transport closed";
      }
      return false;
    }

    std::lock_guard<std::mutex> lock(state_->mutex);
    const auto command = static_cast<OtaCommand>(frame.data[0]);
    if (state_->suppress_replies && command != OtaCommand::abort) {
      return true;
    }
    switch (command) {
      case OtaCommand::sync:
        queueReply(command, 0, 0);
        break;
      case OtaCommand::get_capability:
        queueCapability(frame.data[1]);
        break;
      case OtaCommand::start:
        state_->session_id = readU16(frame.data, 1);
        queueReply(command, state_->session_id, 0);
        break;
      case OtaCommand::start_crc:
      case OtaCommand::start_version:
        queueReply(command, readU16(frame.data, 1), 0);
        break;
      case OtaCommand::block_begin:
        state_->session_id = readU16(frame.data, 1);
        state_->block_index = readU16(frame.data, 3);
        state_->current_block.clear();
        ++state_->block_begin_count;
        queueReply(command, state_->session_id, state_->block_index);
        break;
      case OtaCommand::data: {
          const auto length = std::min<uint8_t>(frame.data[2], 5U);
          state_->current_block.insert(
            state_->current_block.end(), frame.data.begin() + 3,
            frame.data.begin() + 3 + length);
          ++state_->data_frame_count;
          break;
        }
      case OtaCommand::block_end: {
          const auto session = readU16(frame.data, 1);
          const auto block = readU16(frame.data, 3);
          const auto received_crc = readU16(frame.data, 5);
          if (block == 0 && state_->nack_first_block_once) {
            state_->nack_first_block_once = false;
            queueReply(command, session, block, false, 4);
          } else if (received_crc != crc16CcittFalse(state_->current_block)) {
            queueReply(command, session, block, false, 3);
          } else {
            state_->received_image.insert(
              state_->received_image.end(), state_->current_block.begin(),
              state_->current_block.end());
            queueReply(command, session, block);
          }
          break;
        }
      case OtaCommand::end:
        state_->final_crc_received = readU32(frame.data, 3);
        state_->final_crc_calculated = crc32Ieee(state_->received_image);
        state_->final_crc_matched =
          state_->final_crc_received == state_->final_crc_calculated;
        queueReply(
          command, readU16(frame.data, 1), 0, state_->final_crc_matched,
          state_->final_crc_matched ? 0 : 5);
        break;
      case OtaCommand::abort:
        break;
      default:
        if (error) {
          *error = "unsupported fake command";
        }
        return false;
    }
    return true;
  }

  bool receive(
    CanFrame & frame, std::chrono::milliseconds timeout, std::string * error) override
  {
    return receiveById(0x581, frame, timeout, error);
  }

  bool receiveById(
    uint32_t, CanFrame & frame, std::chrono::milliseconds, std::string * error) override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->replies.empty()) {
      if (error) {
        *error = "CAN receive timeout";
      }
      return false;
    }
    frame = state_->replies.front();
    state_->replies.erase(state_->replies.begin());
    return true;
  }

private:
  void queueReply(
    OtaCommand command, uint16_t session, uint16_t detail,
    bool positive = true, uint8_t status = 0)
  {
    CanFrame response;
    response.id = 0x581;
    response.dlc = 8;
    response.data[0] = static_cast<uint8_t>(
      positive ? OtaCommand::ack : OtaCommand::nack);
    response.data[1] = static_cast<uint8_t>(command);
    response.data[2] = status;
    writeU16(response.data, 3, session);
    writeU16(response.data, 5, detail);
    state_->replies.push_back(response);
  }

  void queueCapability(uint8_t page)
  {
    CanFrame response;
    response.id = 0x581;
    response.dlc = 8;
    response.data[0] = static_cast<uint8_t>(OtaCommand::ack);
    response.data[1] = static_cast<uint8_t>(OtaCommand::get_capability);
    response.data[2] = 0;
    response.data[3] = page;
    if (page == 0) {
      response.data[4] = 1;
      response.data[5] = 1;
      writeU16(response.data, 6, 13);
    } else {
      writeU32(response.data, 4, 4096);
    }
    state_->replies.push_back(response);
  }

  std::shared_ptr<FakeState> state_;
  bool open_{false};
};

TEST(OtaProtocolTest, EncodesMetadataAndVariableLengthDataWithoutTruncation)
{
  OtaProtocol protocol({0x601, 0x581, false});

  const auto start = protocol.buildStart(0x1234, 0x89abcdefU);
  EXPECT_EQ(start.data[0], 0x10);
  EXPECT_EQ(readU16(start.data, 1), 0x1234);
  EXPECT_EQ(readU32(start.data, 3), 0x89abcdefU);

  const auto crc = protocol.buildStartCrc(0x1234, 0x76543210U);
  EXPECT_EQ(readU32(crc.data, 3), 0x76543210U);
  const auto version = protocol.buildStartVersion(0x1234, 0xfedcba98U);
  EXPECT_EQ(readU32(version.data, 3), 0xfedcba98U);

  const std::array<uint8_t, 2> tail{{0xaa, 0xbb}};
  const auto data = protocol.buildData(7, tail.data(), tail.size());
  EXPECT_EQ(data.dlc, 5);
  EXPECT_EQ(data.data[1], 7);
  EXPECT_EQ(data.data[2], 2);
  EXPECT_EQ(data.data[3], 0xaa);
  EXPECT_EQ(data.data[4], 0xbb);
}

TEST(OtaClientTest, TransfersBlocksAndRetriesWholeBlockAfterNack)
{
  const std::vector<uint8_t> firmware = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
  const auto path = std::filesystem::temp_directory_path() /
    ("stm32_ota_test_" + std::to_string(getpid()) + ".bin");
  {
    std::ofstream output(path, std::ios::binary);
    output.write(
      reinterpret_cast<const char *>(firmware.data()),
      static_cast<std::streamsize>(firmware.size()));
  }

  auto fake_state = std::make_shared<FakeState>();
  OtaClient::Options options;
  options.ack_timeout = std::chrono::milliseconds(10);
  options.capability_timeout = std::chrono::milliseconds(10);
  options.receive_timeout = std::chrono::milliseconds(1);
  options.max_retries = 2;

  std::mutex status_mutex;
  std::condition_variable status_changed;
  OtaClient::Status final_status;
  OtaClient client(
    options,
    [fake_state](const CanTransport::Options &) {
      return std::make_unique<FakeTransport>(fake_state);
    });
  client.setStatusCallback(
    [&](const OtaClient::Status & status) {
      std::lock_guard<std::mutex> lock(status_mutex);
      final_status = status;
      status_changed.notify_all();
    });

  ASSERT_TRUE(client.start(path.string(), 0x12345678U, false));
  {
    std::unique_lock<std::mutex> lock(status_mutex);
    ASSERT_TRUE(
      status_changed.wait_for(
        lock, std::chrono::seconds(2),
        [&]() {
          return final_status.state == OtaState::completed ||
          final_status.state == OtaState::failed;
        }));
  }

  EXPECT_EQ(final_status.state, OtaState::completed) << final_status.message;
  EXPECT_EQ(final_status.progress, 1.0F);
  {
    std::lock_guard<std::mutex> lock(fake_state->mutex);
    EXPECT_EQ(fake_state->received_image, firmware);
    EXPECT_TRUE(fake_state->final_crc_matched) << std::hex <<
      "received=" << fake_state->final_crc_received <<
      " calculated=" << fake_state->final_crc_calculated;
    EXPECT_EQ(fake_state->block_begin_count, 4U);
    EXPECT_EQ(fake_state->data_frame_count, 10U);
  }
  std::filesystem::remove(path);
}

TEST(OtaClientTest, RejectsConcurrentStartAndCancelsAWaitingTransfer)
{
  const auto path = std::filesystem::temp_directory_path() /
    ("stm32_ota_cancel_test_" + std::to_string(getpid()) + ".bin");
  {
    std::ofstream output(path, std::ios::binary);
    const std::array<uint8_t, 4> bytes{{1, 2, 3, 4}};
    output.write(
      reinterpret_cast<const char *>(bytes.data()),
      static_cast<std::streamsize>(bytes.size()));
  }

  auto fake_state = std::make_shared<FakeState>();
  fake_state->suppress_replies = true;
  OtaClient::Options options;
  options.ack_timeout = std::chrono::milliseconds(200);
  options.receive_timeout = std::chrono::milliseconds(5);
  options.max_retries = 2;

  std::mutex status_mutex;
  std::condition_variable status_changed;
  OtaClient::Status final_status;
  OtaClient client(
    options,
    [fake_state](const CanTransport::Options &) {
      return std::make_unique<FakeTransport>(fake_state);
    });
  client.setStatusCallback(
    [&](const OtaClient::Status & status) {
      std::lock_guard<std::mutex> lock(status_mutex);
      final_status = status;
      status_changed.notify_all();
    });

  ASSERT_TRUE(client.start(path.string(), 1, false));
  {
    std::unique_lock<std::mutex> lock(status_mutex);
    ASSERT_TRUE(
      status_changed.wait_for(
        lock, std::chrono::seconds(1),
        [&]() {return final_status.state == OtaState::syncing;}));
  }
  EXPECT_FALSE(client.start(path.string(), 2, false));
  client.cancel();
  {
    std::unique_lock<std::mutex> lock(status_mutex);
    ASSERT_TRUE(
      status_changed.wait_for(
        lock, std::chrono::seconds(1),
        [&]() {return final_status.state == OtaState::cancelled;}));
  }
  EXPECT_EQ(final_status.error, OtaError::none);
  std::filesystem::remove(path);
}

}  // namespace
}  // namespace stm32_can_ota
