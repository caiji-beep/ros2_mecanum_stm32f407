#ifndef STM32_CAN_OTA__CAN_TRANSPORT_HPP_
#define STM32_CAN_OTA__CAN_TRANSPORT_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "stm32_can_ota/can_frame.hpp"

namespace stm32_can_ota
{

// CAN 收发器的抽象接口(纯虚类)。
// 用接口是为了方便做测试：可以写一个"假传输"实现注入测试，而不必真连 CAN 硬件。
class ICanTransport
{
public:
  virtual ~ICanTransport() = default;

  virtual bool open(std::string * error = nullptr) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
  // 发一帧
  virtual bool send(const CanFrame & frame, std::string * error = nullptr) = 0;
  // 盲收一帧：只要总线上来任意一帧就返回(超时内)
  virtual bool receive(
    CanFrame & frame, std::chrono::milliseconds timeout, std::string * error = nullptr) = 0;
  // 按 ID 收一帧：不断丢弃不是 can_id 的帧，直到等到目标帧或超时。
  // 这是 OTA 协议的关键——确保"我发的命令"对上"它回的应答"。
  virtual bool receiveById(
    uint32_t can_id, CanFrame & frame, std::chrono::milliseconds timeout,
    std::string * error = nullptr) = 0;
};

// 真正的 SocketCAN 实现：基于 Linux 原始 CAN 套接字。
class CanTransport final : public ICanTransport
{
public:
  struct Options
  {
    std::string interface_name{"can0"};  // 网络接口名，通常用 can0
    int receive_timeout_ms{100};         // 默认接收超时(毫秒)
  };

  explicit CanTransport(Options options);
  ~CanTransport() override;

  CanTransport(const CanTransport &) = delete;
  CanTransport & operator=(const CanTransport &) = delete;

  bool open(std::string * error = nullptr) override;
  void close() override;
  bool isOpen() const override;

  bool send(const CanFrame & frame, std::string * error = nullptr) override;
  bool receive(
    CanFrame & frame,
    std::chrono::milliseconds timeout,
    std::string * error = nullptr) override;
  bool receiveById(
    uint32_t can_id,
    CanFrame & frame,
    std::chrono::milliseconds timeout,
    std::string * error = nullptr) override;

  const Options & options() const;

private:
  Options options_;     // 配置(接口名/超时)
  int socket_fd_{-1};   // CAN 套接字文件描述符，-1 表示未打开
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__CAN_TRANSPORT_HPP_
