#ifndef STM32_CAN_OTA__CAN_TRANSPORT_HPP_
#define STM32_CAN_OTA__CAN_TRANSPORT_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "stm32_can_ota/can_frame.hpp"

namespace stm32_can_ota
{

class CanTransport
{
public:
  struct Options
  {
    std::string interface_name{"can0"};
    int receive_timeout_ms{100};
  };

  explicit CanTransport(Options options);
  ~CanTransport();

  CanTransport(const CanTransport &) = delete;
  CanTransport & operator=(const CanTransport &) = delete;

  bool open(std::string * error = nullptr);
  void close();
  bool isOpen() const;

  bool send(const CanFrame & frame, std::string * error = nullptr);
  bool receive(
    CanFrame & frame,
    std::chrono::milliseconds timeout,
    std::string * error = nullptr);
  bool receiveById(
    uint32_t can_id,
    CanFrame & frame,
    std::chrono::milliseconds timeout,
    std::string * error = nullptr);

  const Options & options() const;

private:
  Options options_;
  int socket_fd_{-1};
};

}  // namespace stm32_can_ota

#endif  // STM32_CAN_OTA__CAN_TRANSPORT_HPP_
