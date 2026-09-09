#include "stm32_can_ota/ota_client.hpp"

#include <algorithm>
#include <atomic>
#include <limits>
#include <sstream>
#include <thread>
#include <utility>

#include "stm32_can_ota/crc16.hpp"

namespace stm32_can_ota
{

std::string toString(OtaState state)
{
  switch (state) {
    case OtaState::idle:
      return "idle";
    case OtaState::syncing:
      return "syncing";
    case OtaState::query_capability:
      return "query_capability";
    case OtaState::ready:
      return "ready";
    case OtaState::starting:
      return "starting";
    case OtaState::transferring:
      return "transferring";
    case OtaState::verifying:
      return "verifying";
    case OtaState::completed:
      return "completed";
    case OtaState::failed:
      return "failed";
    case OtaState::cancelled:
      return "cancelled";
  }

  return "unknown";
}

std::string toString(OtaError error)
{
  switch (error) {
    case OtaError::none:
      return "none";
    case OtaError::invalid_argument:
      return "invalid_argument";
    case OtaError::transport_open_failed:
      return "transport_open_failed";
    case OtaError::transport_io_error:
      return "transport_io_error";
    case OtaError::timeout:
      return "timeout";
    case OtaError::protocol_error:
      return "protocol_error";
    case OtaError::capability_rejected:
      return "capability_rejected";
    case OtaError::firmware_open_failed:
      return "firmware_open_failed";
    case OtaError::firmware_too_large:
      return "firmware_too_large";
    case OtaError::unknown:
      return "unknown";
  }

  return "unknown";
}

OtaClient::OtaClient(Options options, TransportFactory transport_factory)
: options_(std::move(options)), transport_factory_(std::move(transport_factory))
{
  if (!transport_factory_) {
    transport_factory_ = [](const CanTransport::Options & options) {
        return std::make_unique<CanTransport>(options);
      };
  }
  status_.message = "OTA client idle";
  status_.capability.block_size = options_.block_size;
}

OtaClient::~OtaClient()
{
  cancel_requested_.store(true);
  if (worker_.joinable()) {
    worker_.join();
  }
}

void OtaClient::setStatusCallback(StatusCallback callback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_callback_ = std::move(callback);
}

const OtaClient::Options & OtaClient::options() const
{
  return options_;
}

OtaClient::Status OtaClient::status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

bool OtaClient::start(const std::string & firmware_path, uint32_t firmware_version, bool dry_run)
{
  std::thread finished_worker;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      return false;
    }
    if (worker_.joinable()) {
      finished_worker = std::move(worker_);
    }
    running_ = true;
  }
  if (finished_worker.joinable()) {
    finished_worker.join();
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.firmware_path = firmware_path;
    status_.image_size = 0;
    status_.image_crc32 = 0;
    status_.capability = BootloaderCapability{};
    status_.capability.block_size = options_.block_size;
  }

  if (firmware_path.empty()) {
    updateStatus(
      OtaState::failed,
      OtaError::invalid_argument,
      0.0F,
      "firmware_path is required");
    markWorkerStopped();
    return false;
  }

  updateStatus(OtaState::syncing, OtaError::none, 0.0F, "loading firmware package");

  // 先把固件文件读进内存(FirmwarePackage 内部会算好整包 CRC32)。
  FirmwarePackage package;
  std::string error;
  if (!package.loadSingleImage(firmware_path, &error)) {
    updateStatus(OtaState::failed, OtaError::firmware_open_failed, 0.0F, error);
    markWorkerStopped();
    return false;
  }

  if (package.image().size() > std::numeric_limits<uint32_t>::max()) {
    updateStatus(
      OtaState::failed, OtaError::firmware_too_large, 0.0F,
      "firmware image exceeds the 32-bit OTA protocol size limit");
    markWorkerStopped();
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.firmware_path = package.image().path();
    status_.image_size = static_cast<uint32_t>(package.image().size());
    status_.image_crc32 = package.image().crc32();
    status_.capability.block_size = options_.block_size;
  }

  if (dry_run) {
    updateStatus(
      OtaState::ready,
      OtaError::none,
      1.0F,
      "dry run complete; firmware image loaded, no CAN frames sent");
    markWorkerStopped();
    return true;
  }

  static std::atomic<uint32_t> next_session{1};
  uint16_t session_id = 0;
  while (session_id == 0) {
    session_id = static_cast<uint16_t>(next_session.fetch_add(1) & 0xffffU);
  }

  cancel_requested_.store(false);
  updateStatus(OtaState::ready, OtaError::none, 0.0F, "firmware accepted; OTA worker starting");
  try {
    worker_ = std::thread(
      &OtaClient::runTransfer, this, std::move(package), firmware_version, session_id);
  } catch (const std::exception & exception) {
    updateStatus(
      OtaState::failed, OtaError::unknown, 0.0F,
      std::string("failed to start OTA worker: ") + exception.what());
    markWorkerStopped();
    return false;
  }
  return true;
}

void OtaClient::cancel()
{
  if (isRunning()) {
    cancel_requested_.store(true);
  }
}

bool OtaClient::isRunning() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

void OtaClient::runTransfer(
  FirmwarePackage package, uint32_t firmware_version, uint16_t session_id)
{
  const auto finishFailure = [this](OtaError error, const std::string & message) {
      updateStatus(OtaState::failed, error, 0.0F, message);
      markWorkerStopped();
    };

  CanTransport::Options transport_options;
  transport_options.interface_name = options_.can_interface;
  transport_options.receive_timeout_ms = static_cast<int>(options_.receive_timeout.count());
  auto transport = transport_factory_(transport_options);
  if (!transport) {
    finishFailure(OtaError::transport_open_failed, "CAN transport factory returned null");
    return;
  }

  OtaProtocol protocol({options_.request_id, options_.response_id, options_.extended_id});
  std::string error;
  if (!transport->open(&error)) {
    finishFailure(OtaError::transport_open_failed, error);
    return;
  }

  if (stopIfCancelled(*transport, protocol, session_id)) {
    transport->close();
    markWorkerStopped();
    return;
  }

  // === 步骤1：SYNC 同步握手 ===
  // 发 SYNC 帧，期望收到 ACK。用 exchangeWithRetry 自动重试最多 max_retries 次。
  // 注意 expected_session=0：SYNC 阶段还没有会话，所以会话号传 0。
  updateStatus(OtaState::syncing, OtaError::none, 0.0F, "synchronizing with bootloader");
  auto outcome = exchangeWithRetry(
    *transport, protocol, protocol.buildSync(), OtaCommand::sync, 0, options_.ack_timeout);
  if (outcome.result != ExchangeResult::ack) {
    if (outcome.result == ExchangeResult::cancelled) {
      (void)stopIfCancelled(*transport, protocol, session_id);
      transport->close();
      markWorkerStopped();
      return;
    }
    transport->close();
    finishFailure(
      outcome.result == ExchangeResult::timeout ? OtaError::timeout :
      (outcome.result == ExchangeResult::io_error ? OtaError::transport_io_error :
      OtaError::protocol_error),
      "SYNC failed: " + outcome.error);
    return;
  }

  // === 步骤2：查询 Bootloader 能力(GET_CAPABILITY) ===
  // 拿到 block_size / max_fw_size / protocol_version，后面切块和校验都要用。
  updateStatus(
    OtaState::query_capability, OtaError::none, 0.0F, "querying bootloader capability");
  BootloaderCapability capability;
  capability.block_size = options_.block_size;
  if (!queryCapability(*transport, protocol, capability, error)) {
    if (cancel_requested_.load()) {
      (void)stopIfCancelled(*transport, protocol, session_id);
      transport->close();
      markWorkerStopped();
      return;
    }
    transport->close();
    finishFailure(OtaError::capability_rejected, error);
    return;
  }

  // 能力合法性校验：协议版本必须=1；块大小在合理区间；固件不超 Bootloader 上限。
  if (capability.protocol_version != 1) {
    transport->close();
    finishFailure(
      OtaError::capability_rejected,
      "unsupported bootloader protocol version: " +
      std::to_string(capability.protocol_version));
    return;
  }
  if (capability.block_size == 0 || capability.block_size > 1280) {
    transport->close();
    finishFailure(
      OtaError::capability_rejected,
      "bootloader block_size must be in the range 1..1280");
    return;
  }
  if (capability.max_fw_size != 0 && package.image().size() > capability.max_fw_size) {
    transport->close();
    finishFailure(
      OtaError::firmware_too_large,
      "firmware exceeds bootloader max_fw_size=" + std::to_string(capability.max_fw_size));
    return;
  }

  // 算出总共要发多少个块：向上取整(最后一块可能不足 block_size)。
  const auto block_count =
    (package.image().size() + capability.block_size - 1U) / capability.block_size;
  if (block_count > 65536U) {
    transport->close();
    finishFailure(
      OtaError::firmware_too_large,
      "firmware needs more than 65536 addressable OTA blocks");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.capability = capability;
  }

  updateStatus(OtaState::starting, OtaError::none, 0.0F, "sending firmware metadata");
  // 把三条元数据命令放进数组，逐个发送(START/START_CRC/START_VERSION)。
  const struct MetadataFrame
  {
    CanFrame frame;
    OtaCommand command;
  } metadata[] = {
    {protocol.buildStart(session_id, static_cast<uint32_t>(package.image().size())),
      OtaCommand::start},
    {protocol.buildStartCrc(session_id, package.image().crc32()), OtaCommand::start_crc},
    {protocol.buildStartVersion(session_id, firmware_version), OtaCommand::start_version},
  };

  for (const auto & item : metadata) {
    if (stopIfCancelled(*transport, protocol, session_id)) {
      transport->close();
      markWorkerStopped();
      return;
    }
    outcome = exchangeWithRetry(
      *transport, protocol, item.frame, item.command, session_id, options_.ack_timeout);
    if (outcome.result != ExchangeResult::ack) {
      if (outcome.result == ExchangeResult::cancelled) {
        (void)stopIfCancelled(*transport, protocol, session_id);
        transport->close();
        markWorkerStopped();
        return;
      }
      transport->close();
      finishFailure(
        outcome.result == ExchangeResult::timeout ? OtaError::timeout :
        (outcome.result == ExchangeResult::io_error ? OtaError::transport_io_error :
        OtaError::protocol_error),
        toString(item.command) + " failed: " + outcome.error);
      return;
    }
  }

  // === 步骤4：逐块传输固件数据 ===
  // 把固件按 block_size 切成 block_count 块，逐块 sendBlock()。
  // 每块的 offset = 块号 × 块大小；进度 = 已发字节 / 总字节。
  for (size_t block_index = 0; block_index < block_count; ++block_index) {
    if (stopIfCancelled(*transport, protocol, session_id)) {  // 每发一块前检查是否被取消
      transport->close();
      markWorkerStopped();
      return;
    }

    const auto offset = block_index * capability.block_size;
    const auto block = package.image().readBlock(offset, capability.block_size);
    std::ostringstream message;
    message << "transferring block " << (block_index + 1U) << "/" << block_count;
    updateStatus(
      OtaState::transferring, OtaError::none,
      static_cast<float>(offset) / static_cast<float>(package.image().size()), message.str());

    if (!sendBlock(
        *transport, protocol, session_id, static_cast<uint16_t>(block_index), block, error))
    {
      if (cancel_requested_.load()) {
        (void)stopIfCancelled(*transport, protocol, session_id);
        transport->close();
        markWorkerStopped();
      } else {
        transport->close();
        finishFailure(OtaError::protocol_error, error);
      }
      return;
    }
  }

  // === 步骤5：收尾(END) ===
  // 所有块发完后，发 END(带整包 CRC32)，让 Bootloader 做整包校验。
  // 若 ACK → completed 成功；否则按超时/IO/协议错误分类报告。
  updateStatus(OtaState::verifying, OtaError::none, 1.0F, "requesting full-image verification");
  outcome = exchangeWithRetry(
    *transport, protocol, protocol.buildEnd(session_id, package.image().crc32()),
    OtaCommand::end, session_id, options_.ack_timeout);
  if (outcome.result == ExchangeResult::cancelled) {
    (void)stopIfCancelled(*transport, protocol, session_id);
    transport->close();
    markWorkerStopped();
    return;
  }
  transport->close();
  if (outcome.result != ExchangeResult::ack) {
    finishFailure(
      outcome.result == ExchangeResult::timeout ? OtaError::timeout :
      (outcome.result == ExchangeResult::io_error ? OtaError::transport_io_error :
      OtaError::protocol_error),
      "END verification failed: " + outcome.error);
    return;
  }

  updateStatus(
    OtaState::completed, OtaError::none, 1.0F,
    "OTA completed; bootloader verified the full image CRC32");
  markWorkerStopped();
}

// 一次"请求→等待应答"交互：
//   1. 先检查是否被取消；
//   2. 发出 request 帧；
//   3. 在 timeout 内反复 receiveById 等目标 ID 的回复帧(超时则重试，直到超时总时长用完)；
//   4. 解析成 OtaReply，判断是 ACK / NACK，还是别的命令(不匹配则继续等)。
OtaClient::ExchangeOutcome OtaClient::exchange(
  ICanTransport & transport,
  const OtaProtocol & protocol,
  const CanFrame & request,
  OtaCommand expected_command,
  uint16_t expected_session,
  std::chrono::milliseconds timeout)
{
  ExchangeOutcome outcome;
  if (cancel_requested_.load()) {
    outcome.result = ExchangeResult::cancelled;
    outcome.error = "cancelled";
    return outcome;
  }

  if (!transport.send(request, &outcome.error)) {
    outcome.result = ExchangeResult::io_error;
    return outcome;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (cancel_requested_.load()) {
      outcome.result = ExchangeResult::cancelled;
      outcome.error = "cancelled";
      return outcome;
    }

    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    const auto poll_timeout = std::max(
      std::chrono::milliseconds(1), std::min(options_.receive_timeout, remaining));
    CanFrame response;
    std::string receive_error;
    if (!transport.receiveById(
        protocol.options().response_id, response, poll_timeout, &receive_error))
    {
      if (receive_error.find("timeout") != std::string::npos) {
        continue;  // 单次 poll 超时，但总 deadline 还没到 → 继续等
      }
      outcome.result = ExchangeResult::io_error;
      outcome.error = receive_error;
      return outcome;
    }

    const auto reply = protocol.parseReply(response);
    if (!reply.valid || reply.command != expected_command ||
      reply.session_id != expected_session)
    {
      continue;
    }
    outcome.reply = reply;
    outcome.result = reply.positive ? ExchangeResult::ack : ExchangeResult::nack;
    if (!reply.positive) {
      outcome.error = "NACK status=" + std::to_string(reply.status) +
        " detail=" + std::to_string(reply.detail) +
        " next_sequence=" + std::to_string(reply.next_sequence);
    }
    return outcome;
  }

  outcome.result = ExchangeResult::timeout;
  outcome.error = "ACK timeout for " + toString(expected_command);
  return outcome;
}

OtaClient::ExchangeOutcome OtaClient::exchangeWithRetry(
  ICanTransport & transport,
  const OtaProtocol & protocol,
  const CanFrame & request,
  OtaCommand expected_command,
  uint16_t expected_session,
  std::chrono::milliseconds timeout)
{
  ExchangeOutcome last;
  for (uint32_t attempt = 0; attempt <= options_.max_retries; ++attempt) {
    last = exchange(
      transport, protocol, request, expected_command, expected_session, timeout);
    if (last.result == ExchangeResult::ack || last.result == ExchangeResult::cancelled) {
      return last;
    }
  }
  return last;
}

bool OtaClient::queryCapability(
  ICanTransport & transport,
  const OtaProtocol & protocol,
  BootloaderCapability & capability,
  std::string & error)
{
  for (uint8_t page = 0; page < 2; ++page) {
    bool page_received = false;
    for (uint32_t attempt = 0; attempt <= options_.max_retries && !page_received; ++attempt) {
      if (cancel_requested_.load()) {
        error = "capability query cancelled";
        return false;
      }
      const auto request = protocol.buildGetCapability(page);
      if (!transport.send(request, &error)) {
        continue;
      }

      const auto deadline = std::chrono::steady_clock::now() + options_.capability_timeout;
      while (std::chrono::steady_clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now());
        CanFrame response;
        std::string receive_error;
        if (!transport.receiveById(
            protocol.options().response_id, response,
            std::max(
              std::chrono::milliseconds(1),
              std::min(options_.receive_timeout, remaining)), &receive_error))
        {
          if (receive_error.find("timeout") != std::string::npos) {
            continue;
          }
          error = receive_error;
          break;
        }
        if (protocol.parseCapability(response, page, capability)) {
          page_received = true;
          break;
        }
      }
    }
    if (!page_received) {
      error = "GET_CAPABILITY page " + std::to_string(page) + " timed out or was rejected";
      return false;
    }
  }
  return true;
}

bool OtaClient::sendBlock(
  ICanTransport & transport,
  const OtaProtocol & protocol,
  uint16_t session_id,
  uint16_t block_index,
  const std::vector<uint8_t> & block,
  std::string & error)
{
  // 先算这一整块的 CRC16，用于块的完整性校验(Bootloader 收完后会核对)。
  const auto block_crc = crc16CcittFalse(block);
  // 整块传输最多重试 max_retries+1 次。
  for (uint32_t attempt = 0; attempt <= options_.max_retries; ++attempt) {
    // (1) 发 BLOCK_BEGIN 通知"第 block_index 块开始"，并要它回 ACK(且 detail==block_index)。
    auto outcome = exchange(
      transport, protocol,
      protocol.buildBlockBegin(
        session_id, block_index, static_cast<uint16_t>(block.size())),
      OtaCommand::block_begin, session_id, options_.ack_timeout);
    if (outcome.result == ExchangeResult::cancelled) {
      error = "transfer cancelled";
      return false;
    }
    if (outcome.result != ExchangeResult::ack || outcome.reply.detail != block_index) {
      error = "BLOCK_BEGIN failed: " + outcome.error;
      continue;  // 失败重试整块
    }

    // (2) 把块切成每 5 字节一帧，逐帧发 DATA(帧内序号 sequence 自增)。
    bool data_sent = true;
    uint8_t sequence = 0;
    for (size_t offset = 0; offset < block.size(); offset += 5U, ++sequence) {
      if (cancel_requested_.load()) {
        error = "transfer cancelled";
        return false;
      }
      const auto length = static_cast<uint8_t>(std::min<size_t>(5U, block.size() - offset));
      if (!transport.send(protocol.buildData(sequence, block.data() + offset, length), &error)) {
        data_sent = false;
        break;
      }
      // 可选帧间延时，给慢速 Bootloader 喘息时间(默认 0 不延时)。
      if (options_.inter_frame_delay.count() > 0) {
        std::this_thread::sleep_for(options_.inter_frame_delay);
      }
    }
    if (!data_sent) {
      continue;  // 发数据帧失败 → 重试整块
    }

    // (3) 发 BLOCK_END(带块 CRC16)，让 Bootloader 校验本块；ACK 即整块成功。
    outcome = exchange(
      transport, protocol, protocol.buildBlockEnd(session_id, block_index, block_crc),
      OtaCommand::block_end, session_id, options_.ack_timeout);
    if (outcome.result == ExchangeResult::ack && outcome.reply.detail == block_index) {
      return true;
    }
    error = "BLOCK_END failed for block " + std::to_string(block_index) + ": " + outcome.error;
  }
  return false;
}

bool OtaClient::stopIfCancelled(
  ICanTransport & transport, const OtaProtocol & protocol, uint16_t session_id)
{
  if (!cancel_requested_.load()) {
    return false;
  }
  std::string ignored;
  (void)transport.send(protocol.buildAbort(session_id), &ignored);
  updateStatus(OtaState::cancelled, OtaError::none, 0.0F, "OTA cancelled; ABORT sent");
  return true;
}

void OtaClient::markWorkerStopped()
{
  std::lock_guard<std::mutex> lock(mutex_);
  running_ = false;
}

void OtaClient::updateStatus(
  OtaState state,
  OtaError error,
  float progress,
  const std::string & message)
{
  StatusCallback callback;
  Status status_copy;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.state = state;
    status_.error = error;
    status_.progress = progress;
    status_.message = message;
    status_copy = status_;
    callback = status_callback_;
  }

  if (callback) {
    callback(status_copy);
  }
}

}  // namespace stm32_can_ota
