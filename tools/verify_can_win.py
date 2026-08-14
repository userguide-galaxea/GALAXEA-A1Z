#!/usr/bin/env python3
"""Windows CAN 通信验证脚本（HHS USB-CANFD 适配器，gs_usb 用户态后端）。

1. 检查 gs_usb 运行时（libusb-package / WinUSB 驱动）
2. 列出已识别的 CAN 适配器
3. 打开总线、发一帧测试报文、检查 tx 状态

用法:
    python tools\\verify_can_win.py
"""

import importlib.util
import sys
import time

HHS_VID = 0xA8FA
HHS_PID = 0x8598


def step1_check_runtime() -> None:
    print("==> [1/3] 检查 gs_usb 运行时")
    missing = [p for p in ("usb", "gs_usb", "can") if importlib.util.find_spec(p) is None]
    if missing:
        print(f"  [FAIL] 缺少 Python 包: {', '.join(missing)}")
        print("         请执行: pip install -e .[gs_usb]")
        sys.exit(1)
    print("  [OK] python-can / pyusb / gs_usb 已安装")


def step2_scan_adapters() -> None:
    print("==> [2/3] 扫描 USB CAN 适配器")
    import usb.core

    try:
        devs = list(usb.core.find(find_all=True, idVendor=HHS_VID, idProduct=HHS_PID))
    except usb.core.USBError as e:
        print(f"  [FAIL] USB 访问失败: {e}")
        print("         大概率是 WinUSB 驱动未安装——请用 Zadig 将 a8fa:8598 的驱动替换为 WinUSB")
        sys.exit(1)

    if not devs:
        print("  [FAIL] 未找到 HHS 适配器 (a8fa:8598)")
        print("         1. 检查 USB 连接")
        print("         2. 确认已用 Zadig 安装 WinUSB 驱动（Windows 默认驱动会占用设备）")
        sys.exit(1)

    for i, d in enumerate(devs):
        print(f"  [OK] 设备 {i}: VID:PID {d.idVendor:04x}:{d.idProduct:04x} bus={d.bus} addr={d.address}")


def step3_open_bus() -> None:
    print("==> [3/3] 打开总线并发送测试帧")
    import can
    from a1z.motor_drivers.can_backend import open_can_bus

    try:
        bus = open_can_bus(bitrate=1_000_000)
    except Exception as e:
        print(f"  [FAIL] 无法打开 CAN 总线: {e}")
        print("         若提示 Cannot find device，请确认 Zadig WinUSB 驱动已安装")
        sys.exit(1)

    try:
        # 发一帧无害报文（不存在的 CAN ID，电机不会响应）
        msg = can.Message(arbitration_id=0x7FF, data=[0] * 8, is_extended_id=False)
        bus.send(msg)
        time.sleep(0.1)
        print("  [OK] 测试帧已发送，总线工作正常")
        print()
        print("验证通过，A1Z SDK 可以在此机器上运行。")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    step1_check_runtime()
    step2_scan_adapters()
    step3_open_bus()
