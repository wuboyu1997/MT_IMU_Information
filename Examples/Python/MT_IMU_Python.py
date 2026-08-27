#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
MT_IMU_Python.py
================================================================================
Copyright (C) 2026 info@mtmems.com
License: MIT
Version: 1.0.0
Date: 2026-08-26
Author: Wu Boyu
Description: IMU data acquisition and serial communication tool
================================================================================
"""
from __future__ import annotations

import argparse
import contextlib
import io
import struct
import sys
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:
    serial = None

VALID_DATA_LENGTH_MODE0 = 0x3C
VALID_DATA_LENGTH_MODE1 = 0x20
VALID_DATA_LENGTH_MODE2 = 0x1C
VALID_DATA_LENGTH_MODE4 = 0x2A
VALID_DATA_LENGTH_MODE5 = 0x46

FRAME_HEADER = b'\x59\x53'
MAX_FRAME_LEN = 128

(RATE_1HZ, RATE_2HZ, RATE_5HZ, RATE_10HZ, RATE_20HZ,
 RATE_25HZ, RATE_50HZ, RATE_100HZ, RATE_200HZ, RATE_400HZ) = range(10)
(MODE0, MODE1, MODE2, MODE3, MODE4, MODE5) = range(6)
(SYNC_OUT, SYNC_IN, SYNC_RMC) = range(3)

RATE_NAMES = ['1Hz', '2Hz', '5Hz', '10Hz', '20Hz', '25Hz', '50Hz', '100Hz', '200Hz', '400Hz']
MODE_NAMES = ['Mode0', 'Mode1', 'Mode2', 'Mode3', 'Mode4', 'Mode5']
SYNC_NAMES = ['OUT', 'IN', 'RMC']

CONFIG_RATE = [
    (0x00, 0x01),  # RateCmd_1Hz
    (0x00, 0x02),  # RateCmd_2Hz
    (0x00, 0x03),  # RateCmd_5Hz
    (0x00, 0x04),  # RateCmd_10Hz
    (0x00, 0x05),  # RateCmd_20Hz
    (0x00, 0x06),  # RateCmd_25Hz
    (0x00, 0x07),  # RateCmd_50Hz
    (0x00, 0x08),  # RateCmd_100Hz
    (0x00, 0x09),  # RateCmd_200Hz
    (0x00, 0x0A),  # RateCmd_400Hz
]
CONFIG_MODE = [
    (0xD8, 0x00),  # ModeCmd_Mode0
    (0xC0, 0x04),  # ModeCmd_Mode1
    (0xC0, 0x00),  # ModeCmd_Mode2
    (0x00, 0x00),  # ModeCmd_Mode3
    (0xD0, 0x01),  # ModeCmd_Mode4
    (0xD0, 0x10),  # ModeCmd_Mode5
]
CONFIG_SYNC = [
    (0x00, 0x00),  # SyncCmd_OUT
    (0x00, 0x01),  # SyncCmd_IN
    (0x00, 0x02),  # SyncCmd_RMC
]


def data_check_mt(data: bytes | bytearray) -> tuple[int, int]:
    ck1 = 0
    ck2 = 0
    for b in data:
        ck1 = (ck1 + b) & 0xFF
        ck2 = (ck2 + ck1) & 0xFF
    return ck1, ck2

def mt_imu_set_rate(ser, data: tuple[int, int] | list[int]) -> None:
    cmd = [0x59, 0x53, 0x03, 0x0A, data[0], data[1]]
    ck1, ck2 = data_check_mt(cmd[2:])
    cmd += [ck1, ck2]
    ser.write(bytes(cmd))


def mt_imu_set_mode(ser, data: tuple[int, int] | list[int]) -> None:
    cmd = [0x59, 0x53, 0x04, 0x12, 0x00, data[0], data[1]]
    ck1, ck2 = data_check_mt(cmd[2:])
    cmd += [ck1, ck2]
    ser.write(bytes(cmd))


def mt_imu_set_sync(ser, data: tuple[int, int] | list[int]) -> None:
    cmd = [0x59, 0x53, 0x05, 0x0A, data[0], data[1]]
    ck1, ck2 = data_check_mt(cmd[2:])
    cmd += [ck1, ck2]
    ser.write(bytes(cmd))


def send_config(ser, rate: int, mode: int, sync: int, delay: float) -> None:
    print('[INFO] 开始配置 IMU ...', flush=True)
    time.sleep(delay)
    mt_imu_set_rate(ser, CONFIG_RATE[rate])
    print(f'[INFO] 已发送刷新率配置: {RATE_NAMES[rate]}', flush=True)
    time.sleep(delay)
    mt_imu_set_mode(ser, CONFIG_MODE[mode])
    print(f'[INFO] 已发送输出模式配置: {MODE_NAMES[mode]}', flush=True)
    time.sleep(delay)
    mt_imu_set_sync(ser, CONFIG_SYNC[sync])
    print(f'[INFO] 已发送时间同步配置: {SYNC_NAMES[sync]}', flush=True)

@dataclass
class IMUData:
    """对应 imu.h 中 imuData_t 结构体"""
    Temperature: float = 0.0
    Ax: float = 0.0
    Ay: float = 0.0
    Az: float = 0.0
    Gx: float = 0.0
    Gy: float = 0.0
    Gz: float = 0.0
    q0: float = 0.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    timeStamp: float = 0.0

imu_data = IMUData()  # 对应 C 中的全局变量 imuData

def _i32(buf: bytes, off: int) -> int:
    return struct.unpack_from('<i', buf, off)[0]


def _i16(buf: bytes, off: int) -> int:
    return struct.unpack_from('<h', buf, off)[0]

def _u64(buf: bytes, off: int) -> int:
    return struct.unpack_from('<Q', buf, off)[0]


def decode_and_prn_mode0(data_in: bytes) -> None:
    if len(data_in) < 65:
        return
    # acc
    imu_data.Ax = _i32(data_in, 7) * 1e-6
    imu_data.Ay = _i32(data_in, 11) * 1e-6
    imu_data.Az = _i32(data_in, 15) * 1e-6
    # gyro
    imu_data.Gx = _i32(data_in, 21) * 1e-6
    imu_data.Gy = _i32(data_in, 25) * 1e-6
    imu_data.Gz = _i32(data_in, 29) * 1e-6
    # eular
    imu_data.pitch = _i32(data_in, 35) * 1e-6
    imu_data.roll = _i32(data_in, 39) * 1e-6
    imu_data.yaw = _i32(data_in, 43) * 1e-6
    # quaternion
    imu_data.q0 = _i32(data_in, 49) * 1e-6
    imu_data.q1 = _i32(data_in, 53) * 1e-6
    imu_data.q2 = _i32(data_in, 57) * 1e-6
    imu_data.q3 = _i32(data_in, 61) * 1e-6
    # prn
    print('Mode0: acc+gyro+eular+quaternion', flush=True)
    print(f'Ax: {imu_data.Ax:.3f}  Ay: {imu_data.Ay:.3f}  Az: {imu_data.Az:.3f}', flush=True)
    print(f'Gx: {imu_data.Gx:.3f}  Gy: {imu_data.Gy:.3f}  Gz: {imu_data.Gz:.3f}', flush=True)
    print(f'roll: {imu_data.roll:.3f}  pitch: {imu_data.pitch:.3f}  yaw: {imu_data.yaw:.3f}', flush=True)
    print(f'q0: {imu_data.q0:.3f}  q1: {imu_data.q1:.3f}  q2: {imu_data.q2:.3f}  q3: {imu_data.q3:.3f}', flush=True)
    print(flush=True)


def decode_and_prn_mode1(data_in: bytes) -> None:
    if len(data_in) < 37:
        return
    # temperature
    imu_data.Temperature = _i16(data_in, 7) * 1e-2
    # acc
    imu_data.Ax = _i32(data_in, 11) * 1e-6
    imu_data.Ay = _i32(data_in, 15) * 1e-6
    imu_data.Az = _i32(data_in, 19) * 1e-6
    # gyro
    imu_data.Gx = _i32(data_in, 25) * 1e-6
    imu_data.Gy = _i32(data_in, 29) * 1e-6
    imu_data.Gz = _i32(data_in, 33) * 1e-6
    # prn
    print('Mode1: temp+acc+gyro', flush=True)
    print(f'Temperature: {imu_data.Temperature:.3f}', flush=True)
    print(f'Ax: {imu_data.Ax:.3f}  Ay: {imu_data.Ay:.3f}  Az: {imu_data.Az:.3f}', flush=True)
    print(f'Gx: {imu_data.Gx:.3f}  Gy: {imu_data.Gy:.3f}  Gz: {imu_data.Gz:.3f}', flush=True)
    print(flush=True)


def decode_and_prn_mode2(data_in: bytes) -> None:
    if len(data_in) < 33:
        return
    # acc
    imu_data.Ax = _i32(data_in, 7) * 1e-6
    imu_data.Ay = _i32(data_in, 11) * 1e-6
    imu_data.Az = _i32(data_in, 15) * 1e-6
    # gyro
    imu_data.Gx = _i32(data_in, 21) * 1e-6
    imu_data.Gy = _i32(data_in, 25) * 1e-6
    imu_data.Gz = _i32(data_in, 29) * 1e-6
    # prn
    print('Mode2: acc+gyro', flush=True)
    print(f'Ax: {imu_data.Ax:.3f}  Ay: {imu_data.Ay:.3f}  Az: {imu_data.Az:.3f}', flush=True)
    print(f'Gx: {imu_data.Gx:.3f}  Gy: {imu_data.Gy:.3f}  Gz: {imu_data.Gz:.3f}', flush=True)
    print(flush=True)


def decode_and_prn_mode4(data_in: bytes) -> None:
    if len(data_in) < 47:
        return
    # acc
    imu_data.Ax = _i32(data_in, 7) * 1e-6
    imu_data.Ay = _i32(data_in, 11) * 1e-6
    imu_data.Az = _i32(data_in, 15) * 1e-6
    # gyro
    imu_data.Gx = _i32(data_in, 21) * 1e-6
    imu_data.Gy = _i32(data_in, 25) * 1e-6
    imu_data.Gz = _i32(data_in, 29) * 1e-6
    # eular
    imu_data.pitch = _i32(data_in, 35) * 1e-6
    imu_data.roll = _i32(data_in, 39) * 1e-6
    imu_data.yaw = _i32(data_in, 43) * 1e-6
    # prn
    print('Mode4: acc+gyro+eular', flush=True)
    print(f'Ax: {imu_data.Ax:.3f}  Ay: {imu_data.Ay:.3f}  Az: {imu_data.Az:.3f}', flush=True)
    print(f'Gx: {imu_data.Gx:.3f}  Gy: {imu_data.Gy:.3f}  Gz: {imu_data.Gz:.3f}', flush=True)
    print(f'roll: {imu_data.roll:.3f}  pitch: {imu_data.pitch:.3f}  yaw: {imu_data.yaw:.3f}', flush=True)
    print(flush=True)


def decode_and_prn_mode5(data_in: bytes) -> None:
    if len(data_in) < 75:
        return
    # acc
    imu_data.Ax = _i32(data_in, 7) * 1e-6
    imu_data.Ay = _i32(data_in, 11) * 1e-6
    imu_data.Az = _i32(data_in, 15) * 1e-6
    # gyro
    imu_data.Gx = _i32(data_in, 21) * 1e-6
    imu_data.Gy = _i32(data_in, 25) * 1e-6
    imu_data.Gz = _i32(data_in, 29) * 1e-6
    # eular
    imu_data.pitch = _i32(data_in, 35) * 1e-6
    imu_data.roll = _i32(data_in, 39) * 1e-6
    imu_data.yaw = _i32(data_in, 43) * 1e-6
    # quaternion
    imu_data.q0 = _i32(data_in, 49) * 1e-6
    imu_data.q1 = _i32(data_in, 53) * 1e-6
    imu_data.q2 = _i32(data_in, 57) * 1e-6
    imu_data.q3 = _i32(data_in, 61) * 1e-6
    # timeStamp
    imu_data.timeStamp = _u64(data_in, 67) * 1e-6
    # prn
    print('Mode5: acc+gyro+eular+quaternion+timeStamp', flush=True)
    print(f'Ax: {imu_data.Ax:.3f}  Ay: {imu_data.Ay:.3f}  Az: {imu_data.Az:.3f}', flush=True)
    print(f'Gx: {imu_data.Gx:.3f}  Gy: {imu_data.Gy:.3f}  Gz: {imu_data.Gz:.3f}', flush=True)
    print(f'roll: {imu_data.roll:.3f}  pitch: {imu_data.pitch:.3f}  yaw: {imu_data.yaw:.3f}', flush=True)
    print(f'q0: {imu_data.q0:.3f}  q1: {imu_data.q1:.3f}  q2: {imu_data.q2:.3f}  q3: {imu_data.q3:.3f}', flush=True)
    print(f'timeStamp: {imu_data.timeStamp:.3f}', flush=True)
    print(flush=True)


def _dispatch(data_in: bytes) -> None:
    switch_on = data_in[4]
    if switch_on == VALID_DATA_LENGTH_MODE0:
        decode_and_prn_mode0(data_in)
    elif switch_on == VALID_DATA_LENGTH_MODE1:
        decode_and_prn_mode1(data_in)
    elif switch_on == VALID_DATA_LENGTH_MODE2:
        decode_and_prn_mode2(data_in)
    elif switch_on == VALID_DATA_LENGTH_MODE4:
        decode_and_prn_mode4(data_in)
    elif switch_on == VALID_DATA_LENGTH_MODE5:
        decode_and_prn_mode5(data_in)
    # default: 忽略

def _feed(rx: bytearray, frames_out: list[bytes] | None = None) -> None:
    while rx:
        idx = rx.find(FRAME_HEADER)
        if idx < 0:
            rx.clear()
            return
        if idx > 0:
            del rx[:idx]
        if len(rx) < 7:
            return
        frame_len = rx[4] + 7
        if not (7 <= frame_len <= MAX_FRAME_LEN):
            del rx[0]
            continue
        if frame_len > len(rx):
            return
        frame = bytes(rx[:frame_len])
        ck1, ck2 = data_check_mt(frame[2:-2])
        if ck1 == frame[-2] and ck2 == frame[-1]:
            del rx[:frame_len]
            if frames_out is not None:
                frames_out.append(frame)
            else:
                _dispatch(frame)
        else:
            del rx[0]


def listen(ser) -> None:
    print('[INFO] 开始监听 IMU 数据帧（Ctrl+C 退出）...', flush=True)
    rx = bytearray()
    while True:
        chunk = ser.read(1024)
        if chunk:
            rx.extend(chunk)
        _feed(rx)


def main() -> None:
    parser = argparse.ArgumentParser(
        prog='MT_IMU_Python.py',
        description='MT_IMU 数据采集/解析/打印程序（Python 3.11.6）',
    )
    parser.add_argument('port', nargs='?',
                        help='串口设备名：Windows 如 COM3，Linux 如 /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=460800,
                        help='串口波特率（默认 460800，对应 C 中 uart2_init(460800)）')
    parser.add_argument('--rate', type=int, default=RATE_100HZ,
                        help='刷新率索引 0~9（默认 7=100Hz）：' + ' '.join(f'{i}:{n}' for i, n in enumerate(RATE_NAMES)))
    parser.add_argument('--mode', type=int, default=MODE0,
                        help='输出模式索引 0~5（默认 0=Mode0）：' + ' '.join(f'{i}:{n}' for i, n in enumerate(MODE_NAMES)))
    parser.add_argument('--sync', type=int, default=SYNC_OUT,
                        help='时间同步索引 0~2（默认 0=OUT）：' + ' '.join(f'{i}:{n}' for i, n in enumerate(SYNC_NAMES)))
    parser.add_argument('--no-config', action='store_true',
                        help='跳过配置命令（配置已写入 IMU FLASH 时使用）')
    args = parser.parse_args()

    if not args.port:
        parser.print_help()
        print('\n[错误] 未指定串口，例如: python MT_IMU_Python.py COM3', file=sys.stderr)
        sys.exit(1)

    if serial is None:
        print('[错误] 未安装 pyserial，请先执行: pip install pyserial', file=sys.stderr)
        sys.exit(1)

    if not 0 <= args.rate < len(CONFIG_RATE):
        parser.error(f'--rate 超出范围 0~{len(CONFIG_RATE) - 1}')
    if not 0 <= args.mode < len(CONFIG_MODE):
        parser.error(f'--mode 超出范围 0~{len(CONFIG_MODE) - 1}')
    if not 0 <= args.sync < len(CONFIG_SYNC):
        parser.error(f'--sync 超出范围 0~{len(CONFIG_SYNC) - 1}')

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f'[错误] 无法打开串口 {args.port}: {e}', file=sys.stderr)
        sys.exit(1)

    print(f'[INFO] 串口已打开: {args.port} @ {args.baud} baud', flush=True)

    try:
        if not args.no_config:
            send_config(ser, args.rate, args.mode, args.sync, 5.0)
        listen(ser)
    except KeyboardInterrupt:
        print('\n[INFO] 用户中断，退出。', flush=True)
    finally:
        ser.close()


if __name__ == '__main__':
    main()
