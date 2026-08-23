#include "stm32_can_ota/ota_client.hpp"

#include <utility>

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
    case OtaError::not_implemented:
      return "not_implemented";
    case OtaError::unknown:
      return "unknown";
  }

  return "unknown";
}

OtaClient::OtaClient(Options options)
: options_(std::move(options))
{
  status_.message = "OTA client idle";
  status_.capability.block_size = options_.block_size;
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
  (void)firmware_version;

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
    return false;
  }

  updateStatus(OtaState::syncing, OtaError::none, 0.0F, "loading firmware package");

  FirmwarePackage package;
  std::string error;
  if (!package.loadSingleImage(firmware_path, &error)) {
    updateStatus(OtaState::failed, OtaError::firmware_open_failed, 0.0F, error);
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
    return true;
  }

  updateStatus(
    OtaState::failed,
    OtaError::not_implemented,
    0.0F,
    "OTA transfer is a skeleton only; real SYNC/GET_CAPABILITY/START/DATA flow is not implemented yet");
  return false;
}

void OtaClient::cancel()
{
  updateStatus(OtaState::cancelled, OtaError::none, 0.0F, "OTA cancelled");
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
