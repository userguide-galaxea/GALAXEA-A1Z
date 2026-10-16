#!/usr/bin/env python3
"""Windows CAN 通信验证脚本（HHS USB-CANFD 适配器，gs_usb 用户态后端）。

1. 检查 gs_usb Python 运行时（python-can / PyUSB / gs_usb）
2. 列出已识别的 CAN 适配器
3. 打开总线、发一帧测试报文、检查 TX 状态

注意：该脚本只验证 USB-CAN 适配器的识别、打开和发送能力，
不验证电机在线状态、接线/终端电阻、控制回路或重力补偿。

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
        print('         请执行: python -m pip install -e ".[gs_usb]"')
        sys.exit(1)
    print("  [OK] python-can / pyusb / gs_usb 已安装")


def step2_scan_adapters() -> None:
    print("==> [2/3] 扫描 USB CAN 适配器")
    import usb.core

    try:
        devs = list(usb.core.find(find_all=True, idVendor=HHS_VID, idProduct=HHS_PID))
    except usb.core.USBError as e:
        print(f"  [FAIL] USB 访问失败: {e}")
        print("         检查 Zadig 中 a8fa:8598 是否已绑定 WinUSB，并确认设备未被其他程序占用")
        sys.exit(1)

    if not devs:
        print("  [FAIL] 未找到 HHS 适配器 (a8fa:8598)")
        print("         1. 检查适配器供电、USB 线缆和接口")
        print("         2. 在设备管理器/Zadig 中确认 VID:PID 为 a8fa:8598")
        print("         3. 确认已用 Zadig 将该设备绑定到 WinUSB")
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
        print("         检查 WinUSB 绑定、gs_usb/libusb 运行时、适配器型号及设备占用情况")
        sys.exit(1)

    try:
        # 发送一帧标准 CAN 测试报文。成功只代表主机侧发送调用完成。
        msg = can.Message(arbitration_id=0x7FF, data=[0] * 8, is_extended_id=False)
        try:
            bus.send(msg)
            time.sleep(0.1)
        except Exception as e:
            print(f"  [FAIL] 测试帧发送失败: {e}")
            print("         检查适配器状态、CAN 接口配置及是否被其他程序占用")
            sys.exit(1)

        print("  [OK] USB-CAN 适配器已打开，测试帧发送调用成功")
        print()
        print("适配器打开与发送测试通过。")
        print("未验证：CANH/CANL 接线、终端电阻、机械臂上电、电机反馈、")
        print("6 个关节在线状态、250 Hz 控制稳定性或重力补偿安全性。")
        print("如需验证电机通信，请在完成安全检查后运行: python tools\\motor_diag.py --scan")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    step1_check_runtime()
    step2_scan_adapters()
    step3_open_bus()
