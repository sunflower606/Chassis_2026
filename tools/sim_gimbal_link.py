#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_gimbal_link.py —— 用 PC 冒充云台板，向循迹板发 13 字节链路帧

用途（对应《模式2实施计划》阶段 S3）：
    在没有接云台板的情况下，验证「链路解析 → 转向方向 → 距离闭环 → 限幅/斜率」
    这一整条通路。把「通信」「方向」「安全」三个问题一次性隔离出来。

硬件接法（USB-TTL，CH340/CP2102 之类）：
    USB-TTL TX  ->  循迹板 PA10（USART1_RX）
    USB-TTL RX  <-  循迹板 PA9 （USART1_TX，可选，用来看调试行）
    USB-TTL GND --- 循迹板 GND          ← 必须共地！

协议（与 modules/vision_link.c、Gimbal_2026/modules/link_protocol.c 一致）：
    字节 0      : 0xAA        帧头
    字节 1      : 0x55
    字节 2      : 0x02        类型（链路帧）
    字节 3      : 0x08        负载长度
    字节 4~5    : int16  大端 yaw 偏角，单位 0.1 度（±1320 = ±132°）
    字节 6~7    : int16  大端 画面偏移，仅诊断（本脚本固定 0）
    字节 8~9    : uint16 大端 超声波距离 cm（0 = 无效）
    字节 10     : flags，bit0 = 检测到人
    字节 11     : 保留 0
    字节 12     : 校验 = 字节 0~11 的 XOR

用法：
    # 1) 列出串口
    python sim_gimbal_link.py --list

    # 2) 自检（不需要硬件，检查帧格式与校验）
    python sim_gimbal_link.py --selftest

    # 3) 引导式跑完 S3 的 8 个测试步骤（推荐）
    python sim_gimbal_link.py --port COM5 --sequence

    # 4) 交互模式，手动改偏角/距离/有无目标
    python sim_gimbal_link.py --port COM5

    # 5) 直接发一串固定值（脚本化/自动化用）
    python sim_gimbal_link.py --port COM5 --dev-deg 30 --dist 200 --seconds 5

交互模式命令：
    dev <度>      设置云台偏角，例如 dev 30 / dev -45 / dev 0
    dist <cm>     设置超声波距离，0 表示测不到（无效）
    person on     标记"检测到人"（flags bit0 = 1）
    person off    标记"没人"（flags bit0 = 0）
    link off      停止发送，模拟云台板断电/断线
    link on       恢复发送
    show          打印当前要发的状态
    seq           直接进入引导式测试序列
    quit          退出

依赖：pyserial（pip install pyserial）
"""

from __future__ import annotations

import argparse
import struct
import sys
import threading
import time

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None


# ---------------------------------------------------------------- 协议常量 --
FRAME_LEN = 13
HEAD_0 = 0xAA
HEAD_1 = 0x55
TYPE_LINK = 0x02
PAYLOAD_LEN = 0x08
CHECKSUM_INDEX = 12
FLAG_DETECTED = 0x01

DEFAULT_BAUD = 115200
DEFAULT_PERIOD_MS = 30


def build_frame(yaw_dev_tenth_deg: int, offset_x: int = 0,
                distance_cm: int = 0, detected: bool = True) -> bytes:
    """按协议拼一帧 13 字节数据。

    yaw_dev_tenth_deg : 云台偏角，单位 0.1 度（外部传度数请先 ×10）
    offset_x          : 画面水平偏移，仅诊断用
    distance_cm       : 超声波距离，0 表示无效
    detected          : 是否检测到人
    """
    if not -32768 <= yaw_dev_tenth_deg <= 32767:
        raise ValueError("yaw 偏角超出 int16 范围")
    if not -32768 <= offset_x <= 32767:
        raise ValueError("offset_x 超出 int16 范围")
    if not 0 <= distance_cm <= 65535:
        raise ValueError("distance_cm 超出 uint16 范围")

    body = bytearray()
    body.append(HEAD_0)
    body.append(HEAD_1)
    body.append(TYPE_LINK)
    body.append(PAYLOAD_LEN)
    body += struct.pack(">h", yaw_dev_tenth_deg)
    body += struct.pack(">h", offset_x)
    body += struct.pack(">H", distance_cm)
    body.append(FLAG_DETECTED if detected else 0x00)
    body.append(0x00)

    checksum = 0
    for byte in body:
        checksum ^= byte
    body.append(checksum)

    assert len(body) == FRAME_LEN
    return bytes(body)


def parse_frame(frame: bytes) -> dict:
    """把一帧解回字典，用于自检与打印。校验失败抛 ValueError。"""
    if len(frame) != FRAME_LEN:
        raise ValueError(f"帧长应为 {FRAME_LEN}，实际 {len(frame)}")
    if frame[0] != HEAD_0 or frame[1] != HEAD_1:
        raise ValueError("帧头不是 AA 55")
    if frame[2] != TYPE_LINK or frame[3] != PAYLOAD_LEN:
        raise ValueError("类型/长度字段不对")

    checksum = 0
    for byte in frame[:CHECKSUM_INDEX]:
        checksum ^= byte
    if checksum != frame[CHECKSUM_INDEX]:
        raise ValueError(f"校验错误：算出 {checksum:#04x}，帧里 {frame[CHECKSUM_INDEX]:#04x}")

    return {
        "yaw_dev_tenth_deg": struct.unpack(">h", frame[4:6])[0],
        "offset_x": struct.unpack(">h", frame[6:8])[0],
        "distance_cm": struct.unpack(">H", frame[8:10])[0],
        "detected": bool(frame[10] & FLAG_DETECTED),
    }


# ------------------------------------------------------------------ 发送器 --
class LinkSender:
    """后台线程按固定周期发帧；状态可随时改。"""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD,
                 period_ms: int = DEFAULT_PERIOD_MS) -> None:
        if serial is None:
            raise RuntimeError("没有安装 pyserial，先执行：pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=0)
        self.period_s = period_ms / 1000.0
        self.period_ms = period_ms

        self.yaw_dev_tenth_deg = 0
        self.offset_x = 0
        self.distance_cm = 0
        self.detected = False
        self.link_on = True
        self.sent = 0

        self._running = True
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)

    # ---- 状态设置 ----
    def set_dev_deg(self, deg: float) -> None:
        with self._lock:
            self.yaw_dev_tenth_deg = int(round(deg * 10.0))

    def set_distance_cm(self, cm: int) -> None:
        with self._lock:
            self.distance_cm = max(0, min(65535, int(cm)))

    def set_detected(self, detected: bool) -> None:
        with self._lock:
            self.detected = bool(detected)

    def set_link(self, on: bool) -> None:
        with self._lock:
            self.link_on = bool(on)

    def describe(self) -> str:
        with self._lock:
            return (f"dev={self.yaw_dev_tenth_deg / 10.0:+.1f}°(={self.yaw_dev_tenth_deg}), "
                    f"dist={self.distance_cm}cm, person={'YES' if self.detected else 'NO'}, "
                    f"link={'ON' if self.link_on else 'OFF'}, sent={self.sent}")

    # ---- 线程 ----
    def start(self) -> "LinkSender":
        self._thread.start()
        return self

    def _run(self) -> None:
        next_time = time.perf_counter()
        while self._running:
            with self._lock:
                on = self.link_on
                frame = build_frame(self.yaw_dev_tenth_deg, self.offset_x,
                                    self.distance_cm, self.detected)
            if on:
                try:
                    self.ser.write(frame)
                    self.sent += 1
                except Exception as exc:  # 串口被拔掉之类
                    print(f"\n[发送失败] {exc}")
                    self._running = False
                    break
            next_time += self.period_s
            sleep = next_time - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:  # 落后了就重新对齐，别让误差累积
                next_time = time.perf_counter()

    def close(self) -> None:
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass


# --------------------------------------------------------------- 自检模式 --
def run_selftest() -> int:
    print("=== 帧格式自检（不需要硬件）===")
    cases = [
        (0, 0, 100, True),
        (300, 0, 200, True),
        (-450, 0, 30, True),
        (0, 0, 0, False),
        (1320, 0, 400, True),
        (-1320, 0, 2, True),
    ]
    failed = 0
    for dev, off, dist, det in cases:
        frame = build_frame(dev, off, dist, det)
        hex_text = " ".join(f"{b:02X}" for b in frame)
        try:
            back = parse_frame(frame)
        except ValueError as exc:
            print(f"  [FAIL] dev={dev} dist={dist} det={det}: {exc}")
            failed += 1
            continue
        ok = (back["yaw_dev_tenth_deg"] == dev and back["distance_cm"] == dist
              and back["detected"] == det)
        print(f"  [{'PASS' if ok else 'FAIL'}] dev={dev:+5d} dist={dist:3d} "
              f"person={int(det)} -> {hex_text}")
        if not ok:
            failed += 1

    # 校验必须能被检出：故意改错一个字节
    frame = bytearray(build_frame(300, 0, 200, True))
    frame[4] ^= 0xFF
    try:
        parse_frame(bytes(frame))
        print("  [FAIL] 改坏的数据竟然通过了校验")
        failed += 1
    except ValueError:
        print("  [PASS] 篡改帧被校验拦截")

    print(f"\n结果：{'全部通过' if failed == 0 else f'{failed} 项失败'}")
    return 0 if failed == 0 else 1


# ------------------------------------------------------------ 引导式序列 --
def run_sequence(sender: LinkSender) -> None:
    """按《模式2实施计划》S3 的 8 个步骤逐条跑，每步等你确认。"""

    def step(no: int, title: str, expect: str,
             dev: float | None = None, dist: int | None = None,
             person: bool | None = None, link: bool | None = None) -> None:
        if dev is not None:
            sender.set_dev_deg(dev)
        if dist is not None:
            sender.set_distance_cm(dist)
        if person is not None:
            sender.set_detected(person)
        if link is not None:
            sender.set_link(link)

        print("\n" + "-" * 64)
        print(f"步骤 {no}：{title}")
        print(f"  当前发送：{sender.describe()}")
        print(f"  期望现象：{expect}")
        input("  观察完按回车继续（Ctrl+C 退出）...")

    print("=== S3 引导式测试（车体请先架起，轮子离地）===")
    print("提示：OLED 第 1 行显示 Mode:FOLL OK / --，第 2 行 Dev，第 4 行 L/R")

    step(1, "无人（flags=0）", "两轮不动，OLED 第 1 行应显示 Mode:FOLL --",
         dev=0, dist=0, person=False, link=True)
    step(2, "有人、偏角 0、距离=目标 100cm", "两轮都不动（steer=0, forward=0）",
         dev=0, dist=100, person=True)
    step(3, "有人、偏角 0、距离 200cm", "两轮同速前进（距离闭环生效）",
         dev=0, dist=200)
    step(4, "有人、偏角 0、距离 30cm", "两轮同速后退（太近要倒车）",
         dev=0, dist=30)
    step(5, "偏角 +30°（人偏右）、距离 100cm", "左轮快、右轮慢 -> 车向右转",
         dev=30, dist=100)
    step(6, "偏角 -30°（人偏左）、距离 100cm", "右轮快、左轮慢 -> 车向左转",
         dev=-30, dist=100)
    step(7, "偏角 +2.5°（小于 3° 死区）", "两轮不动（死区生效）",
         dev=2.5, dist=100)
    step(8, "断链（停止发送）", "500ms 内停车，OLED 第 1 行变 Mode:FOLL --",
         dev=0, dist=100, link=False)

    print("\n" + "=" * 64)
    print("S3 结束。判定：")
    print("  - 步骤 3/4 方向反了 -> 距离增益或符号问题（先别看电机接线）")
    print("  - 步骤 5/6 转向反了 -> 把 chassis_config.h 的")
    print("    CHASSIS_FOLLOW_STEER_SIGN 由 1 改成 -1（改之前先确认）")
    print("  - 步骤 8 没停 -> 链路超时判定失效")
    print("  - 全程 OLED 都是 -- -> 接线/波特率问题，先看串口调试行的 err=")


# ------------------------------------------------------------ 交互式模式 --
def run_interactive(sender: LinkSender) -> None:
    print("=== 交互模式 ===")
    print("命令：dev <度> | dist <cm> | person on|off | link on|off | show | seq | quit")
    print(f"当前：{sender.describe()}\n")

    while True:
        try:
            line = input("sim> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd in ("quit", "exit", "q"):
                return
            elif cmd == "dev" and len(parts) == 2:
                sender.set_dev_deg(float(parts[1]))
            elif cmd == "dist" and len(parts) == 2:
                sender.set_distance_cm(int(parts[1]))
            elif cmd == "person" and len(parts) == 2:
                sender.set_detected(parts[1].lower() in ("on", "1", "yes", "y"))
            elif cmd == "link" and len(parts) == 2:
                sender.set_link(parts[1].lower() in ("on", "1", "yes", "y"))
            elif cmd == "seq":
                sender.set_link(False)
                run_sequence(sender)
                sender.set_link(True)
            elif cmd == "show":
                pass
            else:
                print("看不懂这条命令，可用：dev/dist/person/link/show/seq/quit")
                continue
        except ValueError:
            print("参数不对，例如：dev 30 / dist 150 / person on")
            continue

        print(f"  -> {sender.describe()}")


# -------------------------------------------------------------------- 入口 --
def list_serial_ports() -> int:
    if list_ports is None:
        print("没有安装 pyserial，先执行：pip install pyserial")
        return 1
    ports = list(list_ports.comports())
    if not ports:
        print("没有找到串口。检查 USB-TTL 是否插好、驱动是否安装。")
        return 1
    print("可用串口：")
    for port in ports:
        print(f"  {port.device:<12} {port.description}")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="PC 冒充云台板，向循迹板发送 13 字节链路帧（S3 测试用）")
    parser.add_argument("--port", help="串口号，例如 COM5 或 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="波特率（默认 115200）")
    parser.add_argument("--period-ms", type=int, default=DEFAULT_PERIOD_MS,
                        help="发送周期，毫秒（默认 30，与云台板一致）")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    parser.add_argument("--selftest", action="store_true", help="只做帧格式自检，不需要硬件")
    parser.add_argument("--sequence", action="store_true", help="跑 S3 的 8 步引导测试")
    parser.add_argument("--dev-deg", type=float, help="固定偏角（度）")
    parser.add_argument("--dist", type=int, help="固定距离（cm），0 = 无效")
    parser.add_argument("--person", choices=["on", "off"], help="是否检测到人")
    parser.add_argument("--seconds", type=float, help="固定值模式下发送多少秒后退出")
    args = parser.parse_args(argv)

    if args.selftest:
        return run_selftest()
    if args.list:
        return list_serial_ports()
    if not args.port:
        parser.print_help()
        print("\n提示：没有 --port 时可以先用 --selftest 检查帧格式，或用 --list 看串口。")
        return 1

    sender = LinkSender(args.port, args.baud, args.period_ms)

    # 固定值模式
    if args.dev_deg is not None or args.dist is not None or args.person is not None:
        if args.dev_deg is not None:
            sender.set_dev_deg(args.dev_deg)
        if args.dist is not None:
            sender.set_distance_cm(args.dist)
        if args.person is not None:
            sender.set_detected(args.person == "on")
        else:
            sender.set_detected(True)

        sender.start()
        print(f"开始发送：{sender.describe()}")
        try:
            deadline = time.perf_counter() + (args.seconds or 10.0)
            while time.perf_counter() < deadline:
                time.sleep(0.5)
                print(f"  {sender.describe()}")
        except KeyboardInterrupt:
            print("\n中断")
        finally:
            sender.close()
        return 0

    sender.start()
    try:
        if args.sequence:
            run_sequence(sender)
        else:
            run_interactive(sender)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        sender.set_link(False)
        sender.close()
        print("已停止发送，串口已关闭。")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
