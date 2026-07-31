#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""COM13 VOFA 数据自动抓取与分析工具。"""

import argparse
import csv
import os
import struct
import sys
import time
from datetime import datetime

try:
    import serial
    from serial import SerialException
except ImportError:
    serial = None
    SerialException = Exception

JUSTFLOAT_TAIL = b"\x00\x00\x80\x7f"
DEFAULT_CHANNEL_NAMES = [
    "ch0_pos_error",
    "ch1_current_pulse",
    "ch2_target_speed",
]


def parse_args():
    parser = argparse.ArgumentParser(
        description="自动抓取 COM13 的 VOFA/串口数据，默认解析 JustFloat 3 通道。"
    )
    parser.add_argument("--port", default="COM13", help="串口号，默认 COM13")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument("--channels", type=int, default=3, help="JustFloat 通道数，默认 3")
    parser.add_argument(
        "--mode",
        choices=["justfloat", "text", "firewater", "raw"],
        default="justfloat",
        help="解析模式，默认 justfloat",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0,
        help="采集时长，单位秒；0 表示一直采集，按 Ctrl+C 停止",
    )
    parser.add_argument("--csv", default="", help="CSV 保存路径；默认自动生成")
    parser.add_argument(
        "--show-raw",
        action="store_true",
        help="显示每帧原始十六进制数据",
    )
    return parser.parse_args()


def make_csv_path(user_path):
    if user_path:
        return user_path
    base_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(base_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(log_dir, f"vofa_com13_{stamp}.csv")


def print_start_message(args, csv_path):
    print("=" * 72)
    print("COM13 VOFA 数据自动抓取工具")
    print("=" * 72)
    print(f"串口: {args.port}")
    print(f"波特率: {args.baud}")
    print(f"模式: {args.mode}")
    if args.mode == "justfloat":
        print(f"JustFloat 通道数: {args.channels}")
        print("当前固件默认含义: ch0=位置误差, ch1=当前位置脉冲, ch2=目标速度")
        print("帧格式: N 个 float 小端数据 + 帧尾 00 00 80 7F")
    print(f"CSV: {csv_path}")
    print("停止方法: 按 Ctrl+C")
    print("=" * 72)


def open_serial(args):
    if serial is None:
        print("错误: 没有安装 pyserial。")
        print("请运行: python -m pip install pyserial")
        return None
    try:
        return serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
        )
    except SerialException as exc:
        print(f"错误: 打不开串口 {args.port}: {exc}")
        print("请检查:")
        print("1. 设备管理器里的端口号是否真的是 COM13。")
        print("2. VOFA、串口助手、另一个脚本是否正在占用 COM13。")
        print("3. 单片机或 USB 转串口是否已经插好。")
        return None


def channel_names(count):
    names = []
    for index in range(count):
        if index < len(DEFAULT_CHANNEL_NAMES):
            names.append(DEFAULT_CHANNEL_NAMES[index])
        else:
            names.append(f"ch{index}")
    return names


def write_header(writer, mode, channels):
    if mode == "justfloat":
        writer.writerow(["time_s", "frame_index", *channel_names(channels), "raw_hex"])
    elif mode == "firewater":
        writer.writerow(["time_s", "line_index", "values", "raw_text"])
    elif mode == "text":
        writer.writerow(["time_s", "line_index", "text"])
    else:
        writer.writerow(["time_s", "chunk_index", "hex"])


def format_values(values):
    return " ".join(f"ch{i}={value:.3f}" for i, value in enumerate(values))


def capture_justfloat(ser, args, writer):
    payload_len = args.channels * 4
    frame_len = payload_len + len(JUSTFLOAT_TAIL)
    unpack_fmt = "<" + "f" * args.channels
    buffer = bytearray()
    start = time.time()
    last_report = start
    valid_frames = 0
    dropped_bytes = 0
    no_data_since = start

    print(f"开始解析 JustFloat：每帧 {frame_len} 字节，其中数据 {payload_len} 字节。")

    while args.duration <= 0 or time.time() - start < args.duration:
        data = ser.read(256)
        now = time.time()
        if data:
            buffer.extend(data)
            no_data_since = now
        elif now - no_data_since > 5:
            print("提示: 5 秒内没有收到数据，请检查单片机是否运行、TX/RX/GND 和 COM13。")
            no_data_since = now

        while True:
            tail_index = buffer.find(JUSTFLOAT_TAIL)
            if tail_index < 0:
                if len(buffer) > frame_len * 4:
                    dropped_bytes += len(buffer) - frame_len
                    del buffer[:-frame_len]
                break

            if tail_index < payload_len:
                dropped_bytes += tail_index + len(JUSTFLOAT_TAIL)
                del buffer[:tail_index + len(JUSTFLOAT_TAIL)]
                continue

            payload_start = tail_index - payload_len
            if payload_start > 0:
                dropped_bytes += payload_start
                del buffer[:payload_start]
                tail_index -= payload_start

            frame = bytes(buffer[:frame_len])
            payload = frame[:payload_len]
            del