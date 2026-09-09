#ifndef STM32_CAN_OTA__OTA_CLIENT_HPP_
#define STM32_CAN_OTA__OTA_CLIENT_HPP_

#include <chrono>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "stm32_can_ota/can_transport.hpp"
#include "stm32_can_ota/firmware_package.hpp"
#include "stm32_can_ota/ota_error.hpp"
#include "stm32_can_ota/ota_protocol.hpp"
#include "stm32_can_ota/ota_state.hpp"

namespace stm32_can_ota
{

class OtaClient
{
public:
  // 可调参数。
  struct Options
  {
    std::string can_interface{"can0"};              // CAN 接口名
    uint32_t request_id{0x601};                     // 主机→STM32 的 CAN ID
    uint32_t response_id{0x581};                    // STM32→主机的 CAN ID
    bool extended_id{false};                        // 是否扩展帧
    uint32_t block_size{256};                       // 块大小(实际以 Bootloader 能力为准来覆盖)
    std::chrono::milliseconds ack_timeout{1000};    // 等待单条 ACK 的超时
    std::chrono::milliseconds capability_timeout{1000};  // 等待能力回复的超时
    std::chrono::milliseconds receive_timeout{100}; // 底层收帧超时(传给 transport)
    uint32_t max_retries{3};                        // 单条命令失败重试次数
    std::chrono::microseconds inter_frame_delay{0}; // 帧间延时(某些 Bootloader 处理慢需加)
  };

  // 当前升级状态快照，对外(节点/话题)可见。
  struct Status
  {
    OtaState state{OtaState::idle};       // 当前状态机状态
    OtaError error{OtaError::none};       // 错误码
    float progress{0.0F};                 // 进度 0~1
    std::string message;                  // 人类可读说明
    std::string firmware_path;            // 正在升级的固件路径
    uint32_t image_size{0};               // 固件字节数
    uint32_t image_crc32{0};              // 固件整包 CRC32
    BootloaderCapability capability;      // 已获知的能力
  };

  // 状态变化回调：升级进行中每推进一步都会调用它(节点用它发 ROS 话题)。
  using StatusCallback = std::function<void (const Status &)>;
  // 传输工厂：返回 ICanTransport。默认造真实 SocketCAN，测试时可换成"假传输"。
  using TransportFactory =
    std::function<std::unique_ptr<ICanTransport>(const CanTransport::Options &)>;

  explicit OtaClient(Options options, TransportFactory transport_factory = {});
  ~OtaClient();

  OtaClient(const OtaClient &) = delete;
  OtaClient & operator=(const OtaClient &) = delete;

  void setStatusCallback(StatusCallback callback);
  const Options & options() const;
  Status status() const;       // 线程安全读取当前状态快照

  // 启动一次升级：加载固件 → 起后台线程跑 runTransfer。
  // dry_run=true 只同步+查能力+校验，不真正烧写(用于"试一试")。
  bool start(const std::string & firmware_path, uint32_t firmware_version, bool dry_run);
  void cancel();               // 请求取消(线程安全，置 cancel_requested_)
  bool isRunning() const;      // 是否后台线程仍在跑

private:
  // exchange 的返回结果类别。
  enum class ExchangeResult
  {
    ack,        // 收到期望的 ACK
    nack,       // 收到 NACK(命令被拒)
    timeout,    // 等待超时
    io_error,   // 收发底层出错
    cancelled,  // 过程中被取消
  };

  // exchange 的完整结果。
  struct ExchangeOutcome
  {
    ExchangeResult result{ExchangeResult::io_error};
    OtaReply reply;    // 解析出的回复(若有)
    std::string error; // 错误描述
  };

  // 后台线程主函数：完整的升级流程(同步→能力→开始→传输块→收尾)。
  void runTransfer(FirmwarePackage package, uint32_t firmware_version, uint16_t session_id);
  // 一次"请求-应答"交互：发 request 帧，再用 receiveById 等期望的回复帧并解析。
  ExchangeOutcome exchange(
    ICanTransport & transport,
    const OtaProtocol & protocol,
    const CanFrame & request,
    OtaCommand expected_command,
    uint16_t expected_session,
    std::chrono::milliseconds timeout);
  // 带重试的 exchange：失败(超时/NACK/IO)时最多重试 max_retries 次。
  ExchangeOutcome exchangeWithRetry(
    ICanTransport & transport,
    const OtaProtocol & protocol,
    const CanFrame & request,
    OtaCommand expected_command,
    uint16_t expected_session,
    std::chrono::milliseconds timeout);
  // 询问 Bootloader 能力(连续问两页，拼出 BootloaderCapability)。
  bool queryCapability(
    ICanTransport & transport,
    const OtaProtocol & protocol,
    BootloaderCapability & capability,
    std::string & error);
  // 发送一个数据块：block_begin → 多个 data → block_end(带块 CRC16)，并校验每块 ACK。
  bool sendBlock(
    ICanTransport & transport,
    const OtaProtocol & protocol,
    uint16_t session_id,
    uint16_t block_index,
    const std::vector<uint8_t> & block,
    std::string & error);
  // 检查是否已被取消；若是，向 STM32 发 ABORT 并清理。返回 true 表示已取消。
  bool stopIfCancelled(
    ICanTransport & transport, const OtaProtocol & protocol, uint16_t session_id);
  void markWorkerStopped();    // 标记后台线程结束、收尾资源
  // 更新 Status 并触发 status_callback_(线程安全)。
  void updateStatus(
    OtaState state,
    OtaError error,
    float progress,
    const std::string & message);

  Options options_;
  TransportFactory transport_factory_;
  mutable std::mutex mutex_;
  Status status_;
  StatusCallback status_callback_;
  std::thread worker_;
  std::atomic<bool> cancel_requested_{false};
  bool running_{false};
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__OTA_CLIENT_HPP_
