#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
STM32 假设备串口仿真器
协议：
  主机->设备（命令包）：AA 55 01 10 + 4*float(LE) + CRC16(IBM,LE)
  设备->主机（测量包）：AA 55 02 10 + 4*float(LE) + CRC16(IBM,LE)
功能：
  解析主机发来的 4 个轮速命令(rad/s)，通过一阶惯性得到“测量轮速”并回包。
  适合验证上位机 writeCmdPacket()/readMeasPacket() 是否正确。
"""

import serial
import struct
import time
import argparse
import sys
from typing import List

HDR_CMD = bytes([0xAA, 0x55, 0x01, 0x10])
HDR_MEAS = bytes([0xAA, 0x55, 0x02, 0x10])
PKT_LEN = 4 + 4*4 + 2  # 头4 + 4个float(16) + CRC2 = 22 字节

def crc16_ibm(data: bytes) -> int:
    """CRC-16/IBM (Modbus) poly=0xA001, init=0xFFFF, ref in/out."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if (crc & 1) != 0:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def read_exact(ser: serial.Serial, n: int, timeout_s: float = 1.0) -> bytes:
    deadline = time.time() + timeout_s
    buf = b""
    while len(buf) < n:
        if time.time() > deadline:
            raise TimeoutError("read_exact timeout")
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
    return buf

def find_and_read_cmd(ser: serial.Serial) -> List[float]:
    """
    从流中定位到命令包头 AA 55 01 10，然后读取完整一帧并校验。
    返回 4 个目标轮速 float（LE）。
    """
    # 简单状态机：不断读字节直到匹配到 HDR_CMD
    sync = b""
    while True:
        b1 = ser.read(1)
        if not b1:
            continue
        sync += b1
        if len(sync) > 4:
            sync = sync[-4:]
        if sync == HDR_CMD:
            # 读余下 16(float*4) + 2(CRC)
            payload = read_exact(ser, 18, timeout_s=1.0)
            frame = HDR_CMD + payload  # 总共 22 字节
            # 校验 CRC
            data_wo_crc = frame[:-2]
            crc_lo, crc_hi = frame[-2], frame[-1]
            recv_crc = crc_hi << 8 | crc_lo  # 低字节在前（LE）
            calc_crc = crc16_ibm(data_wo_crc)
            if recv_crc != calc_crc:
                # CRC 错，丢弃，继续同步
                # print(f"[WARN] CRC mismatch: recv={recv_crc:04X}, calc={calc_crc:04X}")
                sync = b""
                continue
            # 解析 4 个 float（LE）
            floats = list(struct.unpack("<ffff", frame[4:4+16]))
            return floats

def build_meas_packet(w_meas: List[float]) -> bytes:
    payload = struct.pack("<ffff", *w_meas)
    data_wo_crc = HDR_MEAS + payload
    crc = crc16_ibm(data_wo_crc)
    # Modbus/IBM 常见按小端发送 CRC：低字节在前
    pkt = data_wo_crc + struct.pack("<H", crc)
    return pkt

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="比如 /tmp/ttyV1（假设备连这一端）")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=float, default=100.0, help="设备内部更新频率Hz")
    ap.add_argument("--tau", type=float, default=0.15, help="一阶惯性时间常数(秒)")
    ap.add_argument("--noise", type=float, default=0.0, help="测量噪声幅度(均匀分布, rad/s)")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.01)
    except Exception as e:
        print(f"[ERR] 打开串口失败: {e}")
        sys.exit(1)

    print(f"[OK] 假设备已连接 {args.port} @ {args.baud} baud")
    print("[TIP] 等待上位机发送命令帧 AA 55 01 10 + 4xfloat + CRC16(IBM) ...")

    # 一阶惯性模型： w_meas_dot = (w_ref - w_meas) / tau
    w_ref = [0.0, 0.0, 0.0, 0.0]
    w_meas = [0.0, 0.0, 0.0, 0.0]
    dt = 1.0 / max(1e-3, args.rate)
    last_cmd_time = time.time()

    import random

    try:
        while True:
            # 1) 非阻塞尝试抓取命令包（若有就更新 w_ref）
            try:
                ser.timeout = 0.0  # 非阻塞
                # peek 一下缓冲区有无数据
                if ser.in_waiting:
                    ser.timeout = 0.2
                    new_ref = find_and_read_cmd(ser)
                    w_ref = new_ref
                    last_cmd_time = time.time()
                    # print(f"[CMD] w_ref = {w_ref}")
            except TimeoutError:
                pass
            except Exception:
                # 同步失败或 CRC 错误会在函数内部丢弃
                pass

            # 2) 一阶惯性更新
            for i in range(4):
                w_meas[i] += (w_ref[i] - w_meas[i]) * (dt / max(args.tau, 1e-3))

            # 3) 加一点可选噪声
            if args.noise > 0.0:
                w_meas_noisy = [wm + random.uniform(-args.noise, args.noise) for wm in w_meas]
            else:
                w_meas_noisy = w_meas

            # 4) 回测量包
            pkt = build_meas_packet(w_meas_noisy)
            ser.write(pkt)

            # 5) 简单的“看门狗”：若长期没收到命令，则缓慢回零
            if time.time() - last_cmd_time > 2.0:
                w_ref = [0.0, 0.0, 0.0, 0.0]

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n[EXIT] 假设备退出")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
