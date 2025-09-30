#include "mecanum_hw_interface/mecanum_serial_port.hpp"

#include <cerrno>
#include <poll.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <algorithm>

static constexpr uint8_t HDR0=0xAA, HDR1=0x55;   // 包头标识 
static constexpr uint8_t ID_CMD=0x01, ID_MEAS=0x02;  // 命令包和测量包ID
static constexpr uint8_t LEN_BODY=16; // 4 floats
static constexpr size_t PKT_HDR_LEN=4;  // 包头长度
static constexpr size_t PKT_TOTAL=PKT_HDR_LEN + LEN_BODY + 2;   // + CRC16

static constexpr size_t RXBUF_MAX = 4096;
bool MecanumSerialPort::open(const std::string & device, int baud)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) { close(); return false; }

  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD);
  // 8N1 & 关闭软/硬流控（很关键，避免被流控卡住）
  tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_cflag &= ~CRTSCTS;

  speed_t spd = B115200;
  switch (baud) {
    case 921600: spd=B921600; break;
    case 460800: spd=B460800; break;
    case 230400: spd=B230400; break;
    case 115200: default: spd=B115200; break;
  }
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);

  // 非阻塞读
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tio) != 0) { close(); return false; }
  tcflush(fd_, TCIOFLUSH);

  rxbuf_.clear();
  rxbuf_.reserve(512);
  return true;
}

void MecanumSerialPort::close()
{
  if (fd_ >= 0) { ::close(fd_); fd_=-1; }
  rxbuf_.clear();
}

bool MecanumSerialPort::writeCmdPacket(const std::array<double,4> & w)
{
  if (fd_ < 0) return false;
  uint8_t buf[PKT_TOTAL];
  buf[0]=HDR0; buf[1]=HDR1; buf[2]=ID_CMD; buf[3]=LEN_BODY;
  float f0=w[0], f1=w[1], f2=w[2], f3=w[3];
  pack_f32_le_(f0, buf+4);
  pack_f32_le_(f1, buf+8);
  pack_f32_le_(f2, buf+12);
  pack_f32_le_(f3, buf+16);
  uint16_t crc = crc16_(buf, PKT_HDR_LEN + LEN_BODY);
  buf[20]=crc & 0xFF; buf[21]=(crc>>8)&0xFF;
  return write_exact_(buf, PKT_TOTAL);
}

bool MecanumSerialPort::readMeasPacket(std::array<double,4> & w_meas)
{
  if (fd_ < 0) return false;
  fill_rx_nonblock_();
  return try_parse_one_frame_(w_meas);
}

void MecanumSerialPort::fill_rx_nonblock_()
{
  if (fd_ < 0) return;
  int avail = 0;
  if (ioctl(fd_, FIONREAD, &avail) != 0 || avail <= 0) return;

  uint8_t tmp[512];
  while (avail > 0)
  {
    const int chunk = std::min<int>(avail, static_cast<int>(sizeof(tmp)));
    ssize_t n = ::read(fd_, tmp, chunk);
    if (n > 0)
    {
      rxbuf_.insert(rxbuf_.end(), tmp, tmp + n);
      if (rxbuf_.size() > RXBUF_MAX) {
        rxbuf_.erase(rxbuf_.begin(), rxbuf_.end() - RXBUF_MAX);
      }
      avail -= static_cast<int>(n);
    }
    else
    {
      break; // EAGAIN/EINTR/无数据
    }
  }
}

bool MecanumSerialPort::try_parse_one_frame_(std::array<double,4> & w_meas)
{
  size_t i = 0;
  while (i + PKT_HDR_LEN <= rxbuf_.size())
  {
    if (rxbuf_[i] != HDR0) { ++i; continue; }
    if (i + 1 >= rxbuf_.size()) return false;
    if (rxbuf_[i+1] != HDR1) { ++i; continue; }

    if (i + PKT_HDR_LEN > rxbuf_.size()) return false;
    const uint8_t id  = rxbuf_[i+2];
    const uint8_t len = rxbuf_[i+3];

    if (id != ID_MEAS) { ++i; continue; }
    if (len != LEN_BODY) { ++i; continue; }

    if (i + PKT_TOTAL > rxbuf_.size()) return false; // 帧未收全

    const uint8_t* pkt = rxbuf_.data() + i;
    const uint16_t crc_calc = crc16_(pkt, PKT_HDR_LEN + LEN_BODY);
    const uint16_t crc_rx   = static_cast<uint16_t>(pkt[PKT_HDR_LEN + LEN_BODY]) |
                              (static_cast<uint16_t>(pkt[PKT_HDR_LEN + LEN_BODY + 1]) << 8);
    if (crc_calc != crc_rx) { ++i; continue; }

    float f0 = unpack_f32_le_(pkt + 4);
    float f1 = unpack_f32_le_(pkt + 8);
    float f2 = unpack_f32_le_(pkt + 12);
    float f3 = unpack_f32_le_(pkt + 16);
    w_meas = {static_cast<double>(f0), static_cast<double>(f1),
              static_cast<double>(f2), static_cast<double>(f3)};

    // 丢掉本帧及其之前的噪声
    rxbuf_.erase(rxbuf_.begin(), rxbuf_.begin() + (i + PKT_TOTAL));
    return true;
  }

  // 保留尾部最多 3 字节以便下次匹配 AA 55
  if (rxbuf_.size() > 3) {
    rxbuf_.erase(rxbuf_.begin(), rxbuf_.end() - 3);
  }
  return false;
}

bool MecanumSerialPort::write_exact_(const uint8_t* buf, size_t len)
{
  if (fd_ < 0) return false;
  // 22 字节基本一把写完；写不全直接放弃这帧（下个周期再发）
  ssize_t n = ::write(fd_, buf, len);
  return n == static_cast<ssize_t>(len);
}

uint16_t MecanumSerialPort::crc16_(const uint8_t* data, size_t len)
{
  uint16_t crc=0xFFFF;
  for (size_t i=0;i<len;++i) 
  {
    crc ^= data[i];
    for (int j=0;j<8;++j) crc = (crc & 1) ? ((crc>>1) ^ 0xA001) : (crc>>1);
  }
  return crc;
}

void MecanumSerialPort::pack_f32_le_(float v, uint8_t *out4) 
{ 
  std::memcpy(out4,&v,4);
}
float MecanumSerialPort::unpack_f32_le_(const uint8_t *in4) 
{ 
  float v;
  std::memcpy(&v,in4,4);
  return v; 
}
