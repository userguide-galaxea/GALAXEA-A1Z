#!/usr/bin/env python3
"""CAN 抓包工具：被动监听总线，逐帧落盘为 CSV，供 can_analyze.py 统计各 CAN ID 的
实际帧率 / 到达间隔。

背景：01-docs/01-devlogs/2026-07-20.md §5 "下一步" 第 1 条——J6（CAN id 0x06）在 SDK
侧观测到的位置反馈更新率仅 ~6Hz（正常应接近 SDK 250Hz 控制环），但不确定是"总线上
本来就没这么多帧"还是"帧都在总线上、SDK 没吃到"。本工具只做被动 candump 抓包，产出
不经过 SDK 任何环节的总线层真值，交给 can_analyze.py 做逐 ID 帧率对比和分岔判读。

用法（CLI）：
    # 与 ./test.sh --mode joint --joint 6 ... 同步跑（另开一个终端，抓包时长覆盖测试窗口）：
    python -m a1z.analysis.tools.can_capture --can can0 --duration 20 \
        --out 2026-07-20-run-67/can-capture-J6.csv

    # 抓够指定帧数后自动退出（不确定测试时长时更可控）
    python -m a1z.analysis.tools.can_capture --can can0 --n-frames 5000 --out capture.csv

    # 手动 Ctrl+C 停止（--duration / --n-frames 都不给时持续抓，直到手动中断）
    python -m a1z.analysis.tools.can_capture --can can0 --out capture.csv

    # 只关心某几个 CAN ID（十进制关节号或 0x 十六进制均可，逗号分隔；默认全抓）
    python -m a1z.analysis.tools.can_capture --can can0 --ids 6 --out capture-J6-only.csv
    python -m a1z.analysis.tools.can_capture --can can0 --ids 0x01,0x06 --out capture.csv

输出路径（--out）解析规则（见 01-docs/04-sops/07-测试数据管理-SOP.md）：
    - 绝对路径：原样使用。
    - 相对路径 + 设置了 TEST_LOG_ROOT 环境变量：落到
      $TEST_LOG_ROOT/02-a1z/01-output/<相对路径>（与 run_test.py 的 output 根一致，
      抓包 CSV 因此可以直接放进对应 run 目录，如 --out <date>-run-<id>/can-capture-J6.csv）。
    - 相对路径 + 未设置 TEST_LOG_ROOT：相对当前工作目录（保持旧行为不变）。

依赖：系统 candump（can-utils，`sudo apt install can-utils`）。本工具通过子进程调用
`candump -L <can>` 解析其 log-file 格式（`(epoch) iface id#data`）逐行落盘，不打开
python-can/socketcan 的收发 socket——纯被动监听，可以和 SDK、其它诊断工具（如
tools/motor_diag.py）同时挂在同一个 CAN 接口上互不干扰。
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import shutil
import subprocess
import time
from pathlib import Path
from typing import Dict, Optional, Set

# joint_idx(1-based CAN id) → 可读名字，与 tools/motor_diag.py 的 JOINT_CONFIG 一致
# （id 直接等于关节号：0x01..0x06=J1..J6，0x07=gripper）。
JOINT_NAMES = {1: "J1", 2: "J2", 3: "J3", 4: "J4", 5: "J5", 6: "J6", 7: "gripper"}


def resolve_output_path(path: Path) -> Path:
    """把用户给的输出路径按 TEST_LOG_ROOT 规则解析（见模块 docstring / SOP-07）。

    绝对路径原样返回；相对路径在 TEST_LOG_ROOT 设置时挂到
    $TEST_LOG_ROOT/02-a1z/01-output/ 下，否则保持相对当前工作目录（旧行为）。
    """
    path = Path(path)
    if path.is_absolute():
        return path
    root = os.environ.get("TEST_LOG_ROOT")
    if root:
        return Path(root) / "02-a1z" / "01-output" / path
    return path


_LOG_LINE_RE = re.compile(
    r"^\((?P<ts>\d+\.\d+)\)\s+(?P<iface>\S+)\s+(?P<id>[0-9A-Fa-f]+)#(?P<data>[0-9A-Fa-f]*)\s*$"
)


def _check_can_interface(channel: str) -> None:
    """接口不存在/未 UP 时直接报错退出，避免抓到一个空文件才发现接口没通。"""
    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", channel],
            capture_output=True, text=True, timeout=5,
        )
    except FileNotFoundError:
        raise SystemExit("未找到 `ip` 命令，请确认运行在 Linux 环境。")
    output = result.stdout.strip()
    if result.returncode != 0 or not output:
        raise SystemExit(f"接口 {channel} 不存在。检查 USB-CAN 适配器是否连接。")
    if "state UP" not in output and "LOWER_UP" not in output:
        raise SystemExit(
            f"接口 {channel} 存在但未启动，请先:\n"
            f"  sudo ip link set {channel} down\n"
            f"  sudo ip link set {channel} type can bitrate 1000000\n"
            f"  sudo ip link set {channel} up"
        )


def parse_ids(spec: Optional[str]) -> Optional[Set[int]]:
    """'--ids 6,0x07' → {6,7}；None（未指定）→ 不过滤，全抓。"""
    if spec is None:
        return None
    ids = set()
    for tok in spec.split(","):
        tok = tok.strip()
        if not tok:
            continue
        ids.add(int(tok, 16) if tok.lower().startswith("0x") else int(tok, 10))
    return ids


def fmt_id(can_id: int) -> str:
    name = JOINT_NAMES.get(can_id)
    return f"0x{can_id:02X}({name})" if name else f"0x{can_id:02X}"


def capture(
    channel: str,
    out_path: Path,
    *,
    duration: Optional[float] = None,
    n_frames: Optional[int] = None,
    ids: Optional[Set[int]] = None,
) -> Dict[int, int]:
    """抓包主循环。返回 {can_id: 帧数} 供调用方打印小结。"""
    if shutil.which("candump") is None:
        raise SystemExit("未找到 `candump`（can-utils）。请先 `sudo apt install can-utils`。")
    _check_can_interface(channel)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    counts: Dict[int, int] = {}
    t0: Optional[float] = None
    total = 0

    proc = subprocess.Popen(
        ["candump", "-L", channel],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        text=True, bufsize=1,
    )
    print(f"[can_capture] 抓取 {channel} → {out_path}"
          + (f"  duration={duration}s" if duration else "")
          + (f"  n_frames={n_frames}" if n_frames else "")
          + (f"  ids={sorted(ids)}" if ids else "  ids=全部")
          + "  (Ctrl+C 停止)")

    try:
        with out_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t_epoch", "t_rel_s", "can_id_hex", "dlc", "data_hex"])
            assert proc.stdout is not None
            for line in proc.stdout:
                m = _LOG_LINE_RE.match(line)
                if not m:
                    continue  # 跳过 RTR/错误帧等非常规行，不中断抓包
                can_id = int(m.group("id"), 16)
                if ids is not None and can_id not in ids:
                    continue
                t_epoch = float(m.group("ts"))
                if t0 is None:
                    t0 = t_epoch
                t_rel = t_epoch - t0
                data_hex = m.group("data")
                dlc = len(data_hex) // 2
                writer.writerow([f"{t_epoch:.6f}", f"{t_rel:.6f}",
                                  f"0x{can_id:02X}", dlc, data_hex])
                f.flush()  # 中途 Ctrl+C 也不丢已抓到的帧

                counts[can_id] = counts.get(can_id, 0) + 1
                total += 1
                if duration is not None and t_rel >= duration:
                    break
                if n_frames is not None and total >= n_frames:
                    break
    except KeyboardInterrupt:
        print("\n[can_capture] 收到中断，停止抓包。")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

    return counts


def _print_summary(counts: Dict[int, int], duration_s: float) -> None:
    print(f"\n[can_capture] 小结（实际抓取时长 {duration_s:.2f}s）:")
    if not counts:
        print("  未抓到任何帧。检查总线是否有电机在线 / 接线 / 波特率。")
        return
    for can_id in sorted(counts):
        n = counts[can_id]
        rate = n / duration_s if duration_s > 0 else float("nan")
        print(f"  {fmt_id(can_id):16s} 帧数={n:6d}  平均帧率≈{rate:6.1f} Hz")
    print("\n  详细统计 / 分岔判读请运行:")
    print("    python -m a1z.analysis.tools.can_analyze <上面的 --out CSV 路径>")


def build_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="被动 CAN 抓包（wraps candump -L），落盘 CSV")
    ap.add_argument("--can", default="can0", help="CAN 接口名 (默认: can0)")
    ap.add_argument("--out", required=True, type=Path, help="输出 CSV 路径")
    ap.add_argument("--duration", type=float, default=None, help="抓包时长（秒），默认不限")
    ap.add_argument("--n-frames", type=int, default=None,
                     help="抓够该帧数后自动停止（过滤后计数），默认不限")
    ap.add_argument("--ids", default=None,
                     help="只抓指定 CAN ID，逗号分隔，十进制或 0x 十六进制均可（默认全抓）")
    return ap.parse_args()


def main() -> None:
    args = build_args()
    ids = parse_ids(args.ids)
    out_path = resolve_output_path(args.out)
    t_start = time.time()
    counts = capture(args.can, out_path, duration=args.duration,
                      n_frames=args.n_frames, ids=ids)
    _print_summary(counts, time.time() - t_start)


if __name__ == "__main__":
    main()
