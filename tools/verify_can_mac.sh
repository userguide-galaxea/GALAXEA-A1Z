#!/usr/bin/env bash
# macOS CAN 通信验证脚本（HHS USB-CANFD 适配器，gs_usb 用户态后端）
#
# 1. 检查 libusb 运行时
# 2. 列出已识别的 CAN 适配器
# 3. 打开总线、发一帧测试报文、检查 tx 状态
#
# 默认使用当前环境的 python（conda/venv 激活态），可用 PYTHON 环境变量覆盖：
#     PYTHON=/path/to/python bash tools/verify_can_mac.sh
set -euo pipefail

# 优先用 PYTHON 环境变量，其次当前 PATH 里的 python，最后 python3
PY="${PYTHON:-$(command -v python || command -v python3)}"
echo "使用 Python: $PY ($("$PY" --version 2>&1))"

echo "==> [1/3] 检查 libusb 运行时"
"$PY" - <<'EOF'
import importlib.util, sys
missing = [p for p in ("usb", "gs_usb", "can") if importlib.util.find_spec(p) is None]
if missing:
    print(f"  [FAIL] 缺少 Python 包: {', '.join(missing)}")
    print(f"         当前解释器: {sys.executable}")
    print("         请确认在正确的环境中执行: pip install -e .[gs_usb]")
    sys.exit(1)
try:
    import libusb_package
    print("  [OK] libusb-package (内置 libusb)")
except ImportError:
    # 回退检查系统 libusb
    import ctypes.util
    if ctypes.util.find_library("usb-1.0"):
        print("  [OK] 系统 libusb（brew）")
    else:
        print("  [FAIL] 未找到 libusb。请执行: pip install libusb-package  或  brew install libusb")
        sys.exit(1)
EOF

echo "==> [2/3] 扫描 USB CAN 适配器"
"$PY" - <<'EOF'
import usb.core
HHS_VID, HHS_PID = 0xA8FA, 0x8598
devs = list(usb.core.find(find_all=True, idVendor=HHS_VID, idProduct=HHS_PID))
if not devs:
    print("  [FAIL] 未找到 HHS 适配器 (a8fa:8598)。检查 USB 连接。")
    raise SystemExit(1)
for i, d in enumerate(devs):
    print(f"  [OK] 设备 {i}: VID:PID {d.idVendor:04x}:{d.idProduct:04x} bus={d.bus} addr={d.address}")
EOF

echo "==> [3/3] 打开总线并发送测试帧"
"$PY" - <<'EOF'
import logging
import sys, time
import can
from a1z.motor_drivers.can_backend import open_can_bus

logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(message)s")

try:
    bus = open_can_bus(bitrate=1_000_000)
except Exception as e:
    print(f"  [FAIL] 无法打开 CAN 总线: {e}")
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
EOF
