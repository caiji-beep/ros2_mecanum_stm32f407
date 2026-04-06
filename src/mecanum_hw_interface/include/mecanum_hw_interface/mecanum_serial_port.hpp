#pragma once
#include <string>
#include <array>
#include <cstdint>
#include <vector>

class MecanumSerialPort
{
public:
  bool open(const std::string & device, int baud);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // 协议：AA 55 01 10 + 4x float(rad/s) + CRC16
  bool writeCmdPacket(const std::array<double,4> & w);

  // 协议：AA 55 02 10 + 4x float(rad/s) + CRC16（非阻塞，自动对齐找包）
  bool readMeasPacket(std::array<double,4> & w_meas);

private:
  int fd_{-1};

  // ---- 新增：接收缓冲，用于非阻塞读 + 组帧 ----
  std::vector<uint8_t> rxbuf_;          // 累积接收的数据
  static constexpr size_t RXBUF_MAX = 4096; // 防止无限增长

  // 低级 I/O & 工具
  bool write_exact_(const uint8_t* buf, size_t len);// 精确写入
  static uint16_t crc16_(const uint8_t* data, size_t len);// CRC16计算
  static void pack_f32_le_(float v, uint8_t *out4);// 浮点数打包为小端字节序
  static float unpack_f32_le_(const uint8_t *in4);// 小端字节序解析为浮点数

  // ---- 新增：从串口尽量多读入缓冲、在缓冲中尝试解析一帧 ----
  void fill_rx_nonblock_(); // 非阻塞填充接收缓冲
  bool try_parse_one_frame_(std::array<double,4> & w_meas); // 尝试解析一帧
};
