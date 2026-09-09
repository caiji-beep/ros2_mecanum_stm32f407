#include "stm32_can_ota/can_transport.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <utility>

namespace stm32_can_ota
{

namespace
{

// 错误写入辅助：只有调用方传了 error 指针才写，避免空指针。
void setError(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

// CAN ID 里的高位 bit 是"标志位"(扩展帧标志 CAN_EFF_FLAG / 远程帧标志 CAN_RTR_FLAG)，
// 不是 ID 本身。这里把标志位剥掉，只保留纯 ID(CAN_EFF_MASK=29位, CAN_SFF_MASK=11位)。
uint32_t stripCanFlags(uint32_t can_id)
{
  const bool extended = (can_id & CAN_EFF_FLAG) != 0U;
  return can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
}

}  // namespace

std::string CanFrame::toString() const
{
  std::ostringstream oss;
  oss << "id=0x" << std::hex << std::uppercase << id
      << (extended ? " ext" : " std")
      << (remote ? " rtr" : "")
      << " dlc=" << std::dec << static_cast<int>(dlc)
      << " data=[";

  for (uint8_t i = 0; i < dlc && i < data.size(); ++i) {
    if (i != 0) {
      oss << " ";
    }
    oss << "0x" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        << static_cast<int>(data[i]);
  }

  oss << "]";
  return oss.str();
}

CanTransport::CanTransport(Options options)
: options_(std::move(options))
{
}

CanTransport::~CanTransport()
{
  close();
}

bool CanTransport::open(std::string * error)
{
  close();

  // 1) 创建 CAN 原始套接字(PF_CAN 协议族)
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    setError(error, std::string("socket() failed: ") + std::strerror(errno));
    return false;
  }

  // 2) 设为非阻塞：避免 receive 在无数据时无限阻塞(超时交给 select)
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags >= 0) {
    (void)fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  // 3) 把接口名(如 "can0")转成内核接口索引
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, options_.interface_name.c_str(), IFNAMSIZ - 1);

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    setError(
      error,
      std::string("ioctl(SIOCGIFINDEX) failed for ") + options_.interface_name + ": " +
      std::strerror(errno));
    close();
    return false;
  }

  // 4) 把套接字绑定到该 CAN 接口
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    setError(error, std::string("bind() failed: ") + std::strerror(errno));
    close();
    return false;
  }

  return true;
}

void CanTransport::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanTransport::isOpen() const
{
  return socket_fd_ >= 0;
}

bool CanTransport::send(const CanFrame & frame, std::string * error)
{
  if (!isOpen()) {
    setError(error, "CAN transport is not open");
    return false;
  }

  struct can_frame raw_frame;
  std::memset(&raw_frame, 0, sizeof(raw_frame));
  raw_frame.can_id = frame.id | (frame.extended ? CAN_EFF_FLAG : 0U) |
    (frame.remote ? CAN_RTR_FLAG : 0U);
  raw_frame.can_dlc = static_cast<__u8>(std::min<uint8_t>(frame.dlc, 8U));
  std::copy_n(frame.data.begin(), raw_frame.can_dlc, raw_frame.data);

  const auto written = write(socket_fd_, &raw_frame, sizeof(raw_frame));
  if (written != static_cast<ssize_t>(sizeof(raw_frame))) {
    setError(error, std::string("CAN write() failed: ") + std::strerror(errno));
    return false;
  }

  return true;
}

bool CanTransport::receive(
  CanFrame & frame,
  std::chrono::milliseconds timeout,
  std::string * error)
{
  if (!isOpen()) {
    setError(error, "CAN transport is not open");
    return false;
  }

  // 用 select 实现"最多等 timeout 毫秒"：有数据可读才往下走，否则超时返回 false。
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(socket_fd_, &read_set);

  struct timeval tv;
  tv.tv_sec = static_cast<long>(timeout.count() / 1000);
  tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

  const auto ready = select(socket_fd_ + 1, &read_set, nullptr, nullptr, &tv);
  if (ready < 0) {
    if (errno == EINTR) {
      setError(error, "CAN receive interrupted");
    } else {
      setError(error, std::string("select() failed: ") + std::strerror(errno));
    }
    return false;
  }

  if (ready == 0) {
    setError(error, "CAN receive timeout");
    return false;
  }

  // 有数据：读出一帧原始 can_frame，转成我们的 CanFrame 结构。
  struct can_frame raw_frame;
  const auto bytes = read(socket_fd_, &raw_frame, sizeof(raw_frame));
  if (bytes != static_cast<ssize_t>(sizeof(raw_frame))) {
    setError(error, std::string("CAN read() failed: ") + std::strerror(errno));
    return false;
  }

  frame.id = stripCanFlags(raw_frame.can_id);  // 去掉标志位只留纯 ID
  frame.extended = (raw_frame.can_id & CAN_EFF_FLAG) != 0U;
  frame.remote = (raw_frame.can_id & CAN_RTR_FLAG) != 0U;
  frame.dlc = raw_frame.can_dlc;
  frame.data.fill(0);
  std::copy_n(raw_frame.data, std::min<uint8_t>(raw_frame.can_dlc, 8U), frame.data.begin());
  return true;
}

// 按 ID 收帧：在 timeout 内循环接收，丢弃所有"ID 不符"的帧，
// 直到收到目标 can_id 的帧或超时。这样能保证"我发的 START 对上 START 的 ACK"。
bool CanTransport::receiveById(
  uint32_t can_id,
  CanFrame & frame,
  std::chrono::milliseconds timeout,
  std::string * error)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::string last_error;

  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    CanFrame candidate;

    if (!receive(candidate, std::max(std::chrono::milliseconds(1), remaining), &last_error)) {
      setError(error, last_error);
      return false;
    }

    if (candidate.id == can_id) {
      frame = candidate;
      return true;
    }
    // 否则 ID 不符，继续循环丢弃，直到超时
  }

  setError(error, "CAN receiveById timeout");
  return false;
}

const CanTransport::Options & CanTransport::options() const
{
  return options_;
}

}  // namespace stm32_can_ota
