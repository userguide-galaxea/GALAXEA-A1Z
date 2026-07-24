#!/usr/bin/env bash
# calib_tau_c.sh — J1–J5 库伦摩擦 τ̂_c 标定激励批处理(SOP-09 P0-6)。
#
# 目的:给 J1–J5 补齐 IntegralConfig.from_level 定标锚点 τ̂_c,让六关节都能
# 启用误差积分前馈(τ_I,max = 1.2·τ̂_c)。J6 已标定(≈0.11 Nm),此批只跑 J1–J5。
#
# 方法(regress_tau_c.py 多速率反对称法):同一控制律下每关节至少 2 个扫描
# 速率的三角波,零速截距即 τ̂_c。此批每关节跑 3 个速率(周期 p2/p4/p8)以获得
# R² 交叉验证——与 J6 干净集(G0D-p2/p8 + G0A-J6 p4)同口径。
#
# ★ 口径纪律(务必守住,否则回归被污染)★
#   · --gap-us 250(默认 CAN 节奏),不带 --kp/--kd(默认 PD 增益);
#   · run-id 命名空间 G0E-J<k>-p<period>,回归只喂这些专用 run,不做宽 glob
#     (实测:宽 glob 混入 b0/kp 覆盖档会把 J6 从 0.112/R²0.62 污染到 0.09/R²0.06)。
#
# 前置条件(SOP-09 §1/§4.0):
#   1) 正本 → 运行面已 rsync(本脚本随 GALAXEA-A1Z 一起同步到 .old-leader);
#   2) export TEST_LOG_ROOT=/home/czd/Projects/fix-ee-response-problem/02-test-log
#      (漏设则数据静默落本地 output/,SOP-07 已知坑);
#   3) CAN up(ip -br link | grep can0)、conda 环境 teleop(test.sh 会自动激活)。
#
# 用法:
#   cd open-a1z-t.old-leader/GALAXEA-A1Z/a1z/analysis
#   ./test.sh --mode joints --dry-run     # 先核验安全窗口(无硬件)
#   bash calib_tau_c.sh                    # 跑全部 J1–J5(15 条)
#   bash calib_tau_c.sh 3                  # 只跑 J3(便于分次/重跑单关节)
#
# 安全:安全窗口/前置姿态由 safety.py 自动处理(SAFE_RANGES_DEG/PRECONDITIONS_DEG
# 六关节全覆盖;J1/J3/J4 无需前置,J2/J5 已定义)。窄窗关节(J3/J4:−40..−10°)
# amp 会被自动收窄,SNR 偏低属预期。递增/值守纪律沿用 SOP-09 §6:逐关节起、观察
# eff 不发散;异常立即 Ctrl+C/estop。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

if [[ -z "${TEST_LOG_ROOT:-}" ]]; then
  echo "!! TEST_LOG_ROOT 未设置——数据会落本地 output/(SOP-07 §2 坑)。" >&2
  echo "   export TEST_LOG_ROOT=.../02-test-log 后重跑,或确认就是要落本地。" >&2
fi

# 只跑指定关节(参数),否则 J1–J5 全跑。J6 已标定,不在此批。
JOINTS=("$@")
if [[ ${#JOINTS[@]} -eq 0 ]]; then
  JOINTS=(1 2 3 4 5)
fi

PERIODS=(2 4 8)   # 三速率;p<period> 越小 |q̇| 越大

for k in "${JOINTS[@]}"; do
  for p in "${PERIODS[@]}"; do
    echo "==== 标定激励 J${k} 三角波 period=${p}s → run G0E-J${k}-p${p} ===="
    ./test.sh --gap-us 250 --mode joint --joint "$k" --wave triangle \
      --period-triangle "$p" --run "G0E-J${k}-p${p}"
  done
done

echo
echo "全部完成。回归(在正本侧 open-a1z-t/GALAXEA-A1Z 执行,逐关节、不宽 glob):"
echo '  for k in 1 2 3 4 5; do'
echo '    python -m a1z.analysis.script.regress_tau_c \'
echo '      "$TEST_LOG_ROOT"/02-a1z/01-output/*-run-G0E-J$k-p*/unit-J$k-triangle.csv \'
echo '      --json /tmp/tau_c_J$k.json'
echo '  done'
