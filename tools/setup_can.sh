#!/usr/bin/env bash
# Linux SocketCAN 一键配置脚本（HHS USB-CANFD 适配器）
#
# 1. 加载 gs_usb 内核驱动并注册 HHS VID/PID
# 2. 配置并启动 CAN 接口（1 Mbps）
# 3. 发送一帧测试报文自检
set -euo pipefail

CHANNEL="${1:-can0}"
BITRATE="${2:-1000000}"

if [[ $EUID -ne 0 ]]; then
    echo "请用 sudo 运行: sudo bash tools/setup_can.sh [$CHANNEL] [$BITRATE]"
    exit 1
fi

echo "==> [1/3] 加载 gs_usb 驱动并注册 HHS VID/PID"
modprobe gs_usb
# 已绑定时忽略报错
sh -c 'echo "a8fa 8598" > /sys/bus/usb/drivers/gs_usb/new_id' 2>/dev/null || true

echo "==> [2/3] 配置并启动 $CHANNEL @ $BITRATE bps"
if ! ip link show "$CHANNEL" &>/dev/null; then
    echo "  [FAIL] 接口 $CHANNEL 不存在。检查 USB-CAN 适配器是否连接。"
    ip link show type can || true
    exit 1
fi
ip link set "$CHANNEL" down 2>/dev/null || true
ip link set "$CHANNEL" type can bitrate "$BITRATE"
ip link set "$CHANNEL" up
ip -details link show "$CHANNEL"

echo "==> [3/3] 发送测试帧自检"
if command -v cansend &>/dev/null; then
    # 发一帧无害报文（不存在的 CAN ID，电机不会响应）
    cansend "$CHANNEL" 7FF#0000000000000000
    echo "  [OK] 测试帧已发送"
    TX_DROPPED=$(ip -s link show "$CHANNEL" | grep -oP 'tx_dropped\s+\K\d+' || echo "0")
    if [[ "$TX_DROPPED" != "0" ]]; then
        echo "  [WARN] tx_dropped=$TX_DROPPED，可能是老内核 gs_usb 端点 bug（< 6.8.0-124）"
        echo "         详见 docs/gs_usb_fix.md 或升级内核"
    else
        echo "  [OK] tx_dropped=0"
    fi
else
    echo "  [SKIP] 未安装 can-utils，跳过 TX 自检（sudo apt install can-utils）"
fi

echo
echo "配置完成，$CHANNEL 已就绪。"
