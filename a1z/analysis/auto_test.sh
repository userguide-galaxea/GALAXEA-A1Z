#!/usr/bin/env bash
# G0-ext 机制对灵敏度回归管线（SOP-11 §15.4.2 B1）
#
# 用法:
#   ./auto_test.sh run      # 默认全部 24 组 verify（coulomb J1–J6 + integral J1–J6）
#   ./auto_test.sh judge    # 默认全部 12 对离线判定
#   ./auto_test.sh all      # A + B 串联
#
# 关节/机制筛选（run 和 judge 均支持）:
#   --coulomb-joints 1,2,3,4,5,6     coulomb 对覆盖的关节（默认全部；'' = 跳过）
#   --integral-joints 1,2,3,4,5,6    积分对覆盖的关节（默认全部；'' = 跳过）
#
# 示例:
#   # 仅补跑积分 J1,J2,J5,J6（coulomb 全关节已采完）:
#   ./auto_test.sh --coulomb-joints '' --integral-joints 1,2,5,6 run
#   ./auto_test.sh --coulomb-joints '' --integral-joints 1,2,5,6 judge
#
# 机制对矩阵（全量）:
#   - coulomb 对: J1–J6（off = 无前馈；on = hat:1.0 tanh 全关节）
#   - 积分对:    J1–J6（K0 = 关断；K1 = 默认档开启）
#   每次 verify.sh --mode joint 产出 tri+step 两条腿（~2min/run）
#
# Requires:
#   - conda env 'teleop'（verify.sh 内部自动激活）
#   - export TEST_LOG_ROOT=<repo>/02-test-log（Part A 必需）
#   - CAN 就绪 + 臂空载 + 急停（Part A 必需）
#   - MOS < 40°C（Part A 前置条件）
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# ── helpers ──────────────────────────────────────────────────────────────

_section()  { echo -e "\n${BOLD}${CYAN}═══ $* ═══${NC}"; }
_pass()     { echo -e "  ${GREEN}PASS${NC}  $*"; }
_fail()     { echo -e "  ${RED}FAIL${NC}  $*"; }
_warn()     { echo -e "  ${YELLOW}WARN${NC}  $*"; }
_step()     { echo -e "${BOLD}[auto_test]${NC} $*"; }

# 将逗号分隔的关节列表转为 bash 数组（已过滤空串和非法值）。
# 空串 '' → 空数组（表示跳过该机制）。
_parse_joints() {
  local raw="$1"
  local -n out="$2"
  out=()
  if [[ -z "$raw" ]]; then
    return 0
  fi
  IFS=',' read -ra parts <<<"$raw"
  for p in "${parts[@]}"; do
    p="${p## }"; p="${p%% }"  # trim
    if [[ "$p" =~ ^[1-6]$ ]]; then
      out+=("$p")
    else
      _warn "忽略非法关节值: '$p'（应为 1–6）"
    fi
  done
}

usage() {
  cat <<EOF
Usage: ./auto_test.sh [options] <command>

Commands:
  run       Part A — 运行 verify（默认全量 24 组，~50min 机时，需上机）
  judge     Part B — 离线判定机制对（零上机，读取已有 run 产物）
  all       A + B 串联（run 完成后自动 judge）

关节/机制筛选（--coulomb-joints / --integral-joints 放命令前）:
  --coulomb-joints J1,J2,...     coulomb 对覆盖关节（默认 1,2,3,4,5,6；'' = 跳过）
  --integral-joints J1,J2,...    积分对覆盖关节（默认 1,2,3,4,5,6；'' = 跳过）

Pre-flight (Part A):
  export TEST_LOG_ROOT=<repo>/02-test-log
  CAN 就绪 + 臂空载 + 急停
  MOS < 40°C

Examples:
  ./auto_test.sh run                                    # 全量
  ./auto_test.sh --integral-joints 1,2,5,6 run          # 仅积分 J1,J2,J5,J6
  ./auto_test.sh --coulomb-joints '' --integral-joints 1,2,5,6 run   # 跳过 coulomb
  ./auto_test.sh --integral-joints 1,2,5,6 judge        # 仅判定积分 J1,J2,J5,J6

Output:
  Run 产物: \$TEST_LOG_ROOT/02-a1z/01-output/\$(date +%F)-run-g0ext-*/
  Judge 输出: 终端逐指标 off/on 对照 + PASS/FAIL + Overall (exit 0/1)
EOF
  exit 0
}

_preflight_check() {
  _section "前置检查"
  local ok=1

  if [[ -z "${TEST_LOG_ROOT:-}" ]]; then
    _fail "TEST_LOG_ROOT 未设置（export TEST_LOG_ROOT=<repo>/02-test-log）"
    ok=0
  else
    _pass "TEST_LOG_ROOT=$TEST_LOG_ROOT"
  fi

  if ! command -v conda &>/dev/null; then
    _fail "conda 不可用"
    ok=0
  else
    _pass "conda 可用"
  fi

  if ! python -c "import a1z" 2>/dev/null; then
    _warn "a1z SDK 未在当前环境 importable（verify.sh 会自动激活 teleop env）"
  else
    _pass "a1z SDK importable"
  fi

  echo ""
  echo -e "${YELLOW}请确认以下条件均已满足:${NC}"
  echo "  □ CAN 就绪 + 臂空载 + 急停"
  echo "  □ MOS < 40°C"
  echo "  □ E 段 PD 冻结 ✅（2026-08-03 E6 边际效应判定）"
  echo "  □ S1 库伦前馈 ✅ + S2 积分连续接口 ✅"
  echo "  □ cost_spec v13 ✅"
  echo ""

  if [[ $ok -eq 0 ]]; then
    echo -e "${RED}前置检查未通过，退出。${NC}"
    exit 1
  fi
}

# ── Part A: 运行 verify ──────────────────────────────────────────────────

# 参数: coulomb_joints_str integral_joints_str
run_part_a() {
  local coulomb_str="$1"
  local integral_str="$2"

  local COULOMB_JOINTS=()
  local INTEGRAL_JOINTS=()
  _parse_joints "$coulomb_str" COULOMB_JOINTS
  _parse_joints "$integral_str" INTEGRAL_JOINTS

  local N_COULOMB=$(( ${#COULOMB_JOINTS[@]} * 2 ))   # off + on per joint
  local N_INTEGRAL=$(( ${#INTEGRAL_JOINTS[@]} * 2 ))
  local TOTAL=$(( N_COULOMB + N_INTEGRAL ))

  if [[ $TOTAL -eq 0 ]]; then
    echo -e "${RED}ERROR: 无选中关节（--coulomb-joints 和 --integral-joints 均为空）${NC}"
    exit 1
  fi

  _preflight_check

  cd "$HERE"
  local O="$TEST_LOG_ROOT/02-a1z/01-output"
  local FAILED_RUNS=()
  local COUNT=0

  _section "Part A: 运行机制对（$TOTAL run）"
  echo "  coulomb 关节: ${COULOMB_JOINTS[*]:-(跳过)}"
  echo "  integral 关节: ${INTEGRAL_JOINTS[*]:-(跳过)}"
  echo "输出目录: $O"
  echo "开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
  echo ""

  # ── A1. coulomb 对 ──
  if [[ ${#COULOMB_JOINTS[@]} -gt 0 ]]; then
    _step "A1. coulomb 对：J${COULOMB_JOINTS[*]}（off = 无前馈；on = hat:1.0）"
    for J in "${COULOMB_JOINTS[@]}"; do
      COUNT=$((COUNT + 1))
      echo ""
      echo -e "${BOLD}── [$COUNT/$TOTAL] J${J} coulomb-off ──${NC}"
      if ! ./verify.sh --mode joint --joint "$J" --run "g0ext-J${J}-coulomb-off"; then
        _fail "J${J} coulomb-off 退出码非零"
        FAILED_RUNS+=("g0ext-J${J}-coulomb-off")
      fi

      COUNT=$((COUNT + 1))
      echo ""
      echo -e "${BOLD}── [$COUNT/$TOTAL] J${J} coulomb-on ──${NC}"
      if ! ./verify.sh --mode joint --joint "$J" --coulomb-ff hat:1.0 --run "g0ext-J${J}-coulomb-on"; then
        _fail "J${J} coulomb-on 退出码非零"
        FAILED_RUNS+=("g0ext-J${J}-coulomb-on")
      fi
    done
  else
    _step "A1. coulomb 对：跳过（--coulomb-joints 为空）"
  fi

  # ── A2. 积分对 ──
  if [[ ${#INTEGRAL_JOINTS[@]} -gt 0 ]]; then
    _step "A2. 积分对：J${INTEGRAL_JOINTS[*]}（K0 = 关断；K1 = 默认档开启）"
    for J in "${INTEGRAL_JOINTS[@]}"; do
      COUNT=$((COUNT + 1))
      echo ""
      echo -e "${BOLD}── [$COUNT/$TOTAL] J${J} int-K0 ──${NC}"
      if ! ./verify.sh --mode joint --joint "$J" --ki-level K0 --run "g0ext-J${J}-int-K0"; then
        _fail "J${J} int-K0 退出码非零"
        FAILED_RUNS+=("g0ext-J${J}-int-K0")
      fi

      COUNT=$((COUNT + 1))
      echo ""
      echo -e "${BOLD}── [$COUNT/$TOTAL] J${J} int-K1 ──${NC}"
      if ! ./verify.sh --mode joint --joint "$J" --ki-level K1 --integral-joints "$J" --run "g0ext-J${J}-int-K1"; then
        _fail "J${J} int-K1 退出码非零"
        FAILED_RUNS+=("g0ext-J${J}-int-K1")
      fi
    done
  else
    _step "A2. 积分对：跳过（--integral-joints 为空）"
  fi

  echo ""
  _section "Part A 完成"
  echo "结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "完成: $((TOTAL - ${#FAILED_RUNS[@]})) / $TOTAL"

  if [[ ${#FAILED_RUNS[@]} -gt 0 ]]; then
    echo ""
    echo -e "${RED}失败 run:${NC}"
    for r in "${FAILED_RUNS[@]}"; do
      echo "  - $r"
    done
    echo ""
    echo -e "${RED}Part A 有失败 run，请修复后重跑受影响 run，不继续 Part B。${NC}"
    return 1
  fi

  _pass "全部 $TOTAL run 完成"

  # ── 逐 run 核验提示 ──
  echo ""
  echo -e "${YELLOW}逐 run 核验清单（兼任 watchdog 通道复核，devlog 2026-08-03 教训）:${NC}"
  echo "  □ 终端无 [runner] stream aborted（pos/eff/vel_abs 跳闸）"
  echo "    — coulomb 改变 eff 力矩剖面，若 eff_spike/vel_abs 开始贴线"
  echo "      必须先复核通道口径再继续"
  echo "  □ result.json 无 joint_failures"
  echo "  □ meta.json 的机制记录与意图一致"
  echo "    （off run 无 coulomb/integral；on run 有）"
  echo "  □ MOS < 55°C（超过按 SOP-12 §4.5 降温后从该 run 重跑，"
  echo "    不复用失败 run id）"
  echo ""
  echo -e "${YELLOW}核验通过后执行: ./auto_test.sh judge${NC}"
}

# ── Part B: 离线判定 ────────────────────────────────────────────────────

# 参数: coulomb_joints_str integral_joints_str
run_part_b() {
  local coulomb_str="$1"
  local integral_str="$2"

  local COULOMB_JOINTS=()
  local INTEGRAL_JOINTS=()
  _parse_joints "$coulomb_str" COULOMB_JOINTS
  _parse_joints "$integral_str" INTEGRAL_JOINTS

  local N_COULOMB=${#COULOMB_JOINTS[@]}
  local N_INTEGRAL=${#INTEGRAL_JOINTS[@]}
  local TOTAL=$(( N_COULOMB + N_INTEGRAL ))

  if [[ $TOTAL -eq 0 ]]; then
    echo -e "${RED}ERROR: 无选中关节（--coulomb-joints 和 --integral-joints 均为空）${NC}"
    exit 1
  fi

  local O="${TEST_LOG_ROOT:-}"
  if [[ -z "$O" ]]; then
    O="$(cd "$HERE" && ls -d "$TEST_LOG_ROOT/02-a1z/01-output" 2>/dev/null || true)"
    if [[ -z "$O" ]]; then
      echo -e "${RED}ERROR: TEST_LOG_ROOT 未设置且无法推断输出目录${NC}" >&2
      exit 1
    fi
  fi
  O="$O/02-a1z/01-output"

  # 推断日期：查找最新的 g0ext run 目录
  local D=""
  D=$(ls -d "$O"/*-run-g0ext-J1-coulomb-off-r1-tri-01 2>/dev/null | head -1 | grep -oP '\d{4}-\d{2}-\d{2}' || true)
  if [[ -z "$D" ]]; then
    D=$(date +%F)
    echo -e "${YELLOW}未找到已有 g0ext run 产物，使用今天日期: $D${NC}"
  fi

  _section "Part B: 离线判定（日期=$D，零上机）"
  echo "  coulomb 关节: ${COULOMB_JOINTS[*]:-(跳过)}"
  echo "  integral 关节: ${INTEGRAL_JOINTS[*]:-(跳过)}"
  echo "输出目录: $O/$D-run-g0ext-*"
  echo ""

  local FAILED=0
  local COUNT=0

  # ── B1. coulomb 判定 ──
  if [[ ${#COULOMB_JOINTS[@]} -gt 0 ]]; then
    _step "B1. coulomb 判定：J${COULOMB_JOINTS[*]}（tri 取 lag/resid，step 取 ess/overshoot/ts）"
    for J in "${COULOMB_JOINTS[@]}"; do
      COUNT=$((COUNT + 1))
      echo ""
      echo -e "${BOLD}── [$COUNT/$TOTAL] J${J} coulomb ──${NC}"
      if python $TEST_LOG_ROOT/../open-a1z-t/GALAXEA-A1Z/a1z/analysis/script/g0ext_regression.py --joint "$J" --mechanism coulomb \
        --off-tri  "$O/${D}-run-g0ext-J${J}-coulomb-off-r1-tri-01" \
        --on-tri   "$O/${D}-run-g0ext-J${J}-coulomb-on-r1-tri-01" \
        --off-step "$O/${D}-run-g0ext-J${J}-coulomb-off-r1-step-01" \
        --on-step  "$O/${D}-run-g0ext-J${J}-coulomb-on-r1-step-01"; then
        _pass "J${J} coulomb 对 PASS"
      else
        _fail "J${J} coulomb 对 FAIL"
        FAILED=$((FAILED + 1))
      fi
    done
  else
    _step "B1. coulomb 判定：跳过"
  fi

  # ── B2. 积分判定 ──
  if [[ ${#INTEGRAL_JOINTS[@]} -gt 0 ]]; then
    _step "B2. 积分判定：J${INTEGRAL_JOINTS[*]}（tri 取 lag 相位滞后闸，step 取 ess/overshoot/ts）"
    for J in "${INTEGRAL_JOINTS[@]}"; do
      COUNT=$((COUNT + 1))
      echo ""
      echo -e "${BOLD}── [$COUNT/$TOTAL] J${J} integral ──${NC}"
      if python $TEST_LOG_ROOT/../open-a1z-t/GALAXEA-A1Z/a1z/analysis/script/g0ext_regression.py --joint "$J" --mechanism integral \
        --off-tri  "$O/${D}-run-g0ext-J${J}-int-K0-r1-tri-01" \
        --on-tri   "$O/${D}-run-g0ext-J${J}-int-K1-r1-tri-01" \
        --off-step "$O/${D}-run-g0ext-J${J}-int-K0-r1-step-01" \
        --on-step  "$O/${D}-run-g0ext-J${J}-int-K1-r1-step-01"; then
        _pass "J${J} integral 对 PASS"
      else
        _fail "J${J} integral 对 FAIL"
        FAILED=$((FAILED + 1))
      fi
    done
  else
    _step "B2. 积分判定：跳过"
  fi

  # ── 总结 ──
  echo ""
  _section "Part B 判定结果"
  local PASSED=$((TOTAL - FAILED))
  echo "  PASS: $PASSED / $TOTAL"
  echo "  FAIL: $FAILED / $TOTAL"
  echo ""

  if [[ $FAILED -eq 0 ]]; then
    echo -e "${GREEN}${BOLD}$TOTAL 对全部 exit 0 → 准进 §15.4.3 Phase B session${NC}"
    echo ""
    echo "准出判据确认（SOP-11 §15.4.2 C）:"
    [[ ${#COULOMB_JOINTS[@]} -gt 0 ]] && echo "  ✓ coulomb ${#COULOMB_JOINTS[@]} 对全部 PASS → coulomb 维度准进搜索空间（J${COULOMB_JOINTS[*]}）"
    [[ ${#INTEGRAL_JOINTS[@]} -gt 0 ]] && echo "  ✓ 积分 ${#INTEGRAL_JOINTS[@]} 对全部 PASS → 积分维度准进（J${INTEGRAL_JOINTS[*]}）"
    echo ""
    echo "下一步: ./optimize.sh --joint <N> --phase B --name phaseB-J<N> --n-trials 25 --vel-ff"
    return 0
  else
    echo -e "${RED}${BOLD}$FAILED 对 FAIL → 不得进 Phase B session（§7.4）${NC}"
    echo ""
    echo "不通过处置（SOP-11 §15.4.2 C）:"
    echo "  - coulomb 关节 J lag 不降或恶化越闸 → 该关节 coulomb 上界收紧"
    echo "    （_phase_b_coulomb_range）或该关节不开 coulomb 维度（版本注记）"
    echo "  - 积分 ess 不降 → 该关节移出 PHASE_B_INTEGRAL_JOINTS"
    echo "  - 积分 lag 越闸 → 上调 PHASE_B_T_WIND_RANGE 下界（版本注记）"
    echo "  - 任何调整 = 改 cost_spec 常量 + 版本注记（升版本）"
    return 1
  fi
}

# ── 主入口（选项解析 + 命令分发）─────────────────────────────────────────

# 默认值
COULOMB_JOINTS_STR="1,2,3,4,5,6"
INTEGRAL_JOINTS_STR="1,2,3,4,5,6"
CMD=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --coulomb-joints)
      COULOMB_JOINTS_STR="$2"
      shift 2
      ;;
    --integral-joints)
      INTEGRAL_JOINTS_STR="$2"
      shift 2
      ;;
    run|judge|all)
      CMD="$1"
      shift
      ;;
    help|--help|-h)
      usage
      ;;
    *)
      echo -e "${RED}Unknown argument: $1${NC}" >&2
      usage
      ;;
  esac
done

[[ -n "$CMD" ]] || usage

case "$CMD" in
  run)
    run_part_a "$COULOMB_JOINTS_STR" "$INTEGRAL_JOINTS_STR"
    ;;
  judge)
    run_part_b "$COULOMB_JOINTS_STR" "$INTEGRAL_JOINTS_STR"
    ;;
  all)
    run_part_a "$COULOMB_JOINTS_STR" "$INTEGRAL_JOINTS_STR" && \
    run_part_b "$COULOMB_JOINTS_STR" "$INTEGRAL_JOINTS_STR"
    ;;
esac
