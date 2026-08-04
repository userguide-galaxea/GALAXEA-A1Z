#!/usr/bin/env bash
# L0-matched verification wrapper for optimized MIT controller gains.
#
#   ./verify.sh --mode all --gains-file best_gains.json --run verify-01 --repeat 3
#   ./verify.sh --mode joints --gains-file best_gains.json --run verify-01
#   ./verify.sh --mode joint --joint 6 --gains-file best_gains.json --run verify-J6
#   ./verify.sh --mode joint --joint 6 --kp 12.5 --no-vel-ff --run A3-Pbad --repeat 2
#   ./verify.sh --mode joint --joint 6 --run A3-Pdef --repeat 2   # SDK 默认增益
#   ./verify.sh --mode ee --gains-file best_gains.json --run ee-check --dry-run
#   ./verify.sh --help
#
# Excitation signals match the BO L0 preset (cost_spec.py):
#   - triangle: 15 deg, 4 s, 1 cycle, vel-ff ON (unless --no-vel-ff)
#   - square:   2 deg, 4 s, 1 cycle, vel-ff OFF
#   - hold_pre / hold_post: 0.5 s / 0.5 s
#
# Gains source (at most one; none = SDK defaults):
#   --gains-file FILE | --kp VALS [--kd VALS] | (neither)
#
# Requires: conda env 'teleop'; TEST_LOG_ROOT exported (SOP-07).
set -euo pipefail

# Activate the teleop conda env if not already active.
if [[ "${CONDA_DEFAULT_ENV:-}" != "teleop" ]]; then
  # shellcheck disable=SC1091
  source "$(conda info --base)/etc/profile.d/conda.sh" 2>/dev/null || true
  conda activate teleop 2>/dev/null || true
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
cd "$REPO_ROOT"

# Defaults
MODE=""
JOINT=""
GAINS_FILE=""
KP=""
KD=""
COULOMB_FF=""
KI_LEVEL=""
INTEGRAL_JOINTS=""
T_WIND_S=""
CLAMP_SCALE=""
NO_VEL_FF=0
RUN_PREFIX=""
REPEAT=1
CAN="can0"
DRY_RUN=()
OUTPUT_FORMAT="text"

usage() {
  cat <<EOF
Usage: ./verify.sh --mode MODE --run PREFIX [gains source] [options]

Required:
  --mode {joint,joints,ee,all}   Verification scope (same as test.sh)
  --run PREFIX                   Run id prefix; repeats append -rN-{tri,step,ee}-01

Gains source (at most one; none = SDK default gains):
  --gains-file PATH              best_gains.json from BO pipeline
  --kp VALS [--kd VALS]          Ad-hoc gains override (scalar or 6-vector,
                                 same syntax as run_test.py; e.g. A3 P_bad/P_hand/P_aggr)

Optional:
  --joint 1-6                    Required when --mode joint
  --repeat N                     Number of repeats (default 1; use 3 for SOP-11 L1x3)
  --no-vel-ff                    Triangle leg without vel-ff (A3 P_bad; step leg
                                 never uses vel-ff)
  --coulomb-ff SPEC              Coulomb FF forwarded to run_test.py (SOP-11 §7.2;
                                 'hat', 'hat:1.2', or 6-vector literal) — B1 G0-ext
  --ki-level LVL                 Integral level K0/K1/K2/K3 (SOP-09) — B1 G0-ext
  --integral-joints LIST         Comma-separated 1-based joints for integral
  --t-wind-s S                   Integral wind-up time override (SOP-11 §7.3)
  --clamp-scale S                Integral clamp_scale override
  --can CHANNEL                  CAN channel (default can0)
  --dry-run                      Forward to run_test.py (IK gate / joint windows only)
  --output-format {text,json,csv}  Summary format (default text)
  --help                         Show this message

Examples:
  ./verify.sh --mode all --gains-file best_gains.json --run phaseA-L1 --repeat 3
  ./verify.sh --mode joint --joint 6 --gains-file best_gains.json --run verify-J6
  ./verify.sh --mode joint --joint 6 --kp 12.5 --no-vel-ff --run A3-Pbad --repeat 2
EOF
  exit 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --joint)
      JOINT="$2"
      shift 2
      ;;
      --gains-file)
      GAINS_FILE="$2"
      shift 2
      ;;
    --kp)
      KP="$2"
      shift 2
      ;;
    --kd)
      KD="$2"
      shift 2
      ;;
    --no-vel-ff)
      NO_VEL_FF=1
      shift
      ;;
    --coulomb-ff)
      COULOMB_FF="$2"
      shift 2
      ;;
    --ki-level)
      KI_LEVEL="$2"
      shift 2
      ;;
    --integral-joints)
      INTEGRAL_JOINTS="$2"
      shift 2
      ;;
    --t-wind-s)
      T_WIND_S="$2"
      shift 2
      ;;
    --clamp-scale)
      CLAMP_SCALE="$2"
      shift 2
      ;;
    --run)
      RUN_PREFIX="$2"
      shift 2
      ;;
    --repeat)
      REPEAT="$2"
      shift 2
      ;;
    --can)
      CAN="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=("--dry-run")
      shift
      ;;
    --output-format)
      OUTPUT_FORMAT="$2"
      shift 2
      ;;
    --help)
      usage
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      ;;
  esac
done

# Validation
[[ -n "$MODE" ]] || { echo "ERROR: --mode is required" >&2; exit 1; }
[[ "$MODE" =~ ^(joint|joints|ee|all)$ ]] || { echo "ERROR: --mode must be joint|joints|ee|all" >&2; exit 1; }
[[ -n "$RUN_PREFIX" ]] || { echo "ERROR: --run is required" >&2; exit 1; }
# Gains source: at most one of --gains-file / --kp / --kd; none = SDK defaults.
if [[ -n "$GAINS_FILE" && ( -n "$KP" || -n "$KD" ) ]]; then
  echo "ERROR: --gains-file and --kp/--kd are mutually exclusive" >&2; exit 1
fi
[[ -z "$GAINS_FILE" || -f "$GAINS_FILE" ]] || { echo "ERROR: gains file not found: $GAINS_FILE" >&2; exit 1; }

if [[ "$MODE" == "joint" ]]; then
  [[ -n "$JOINT" ]] || { echo "ERROR: --joint is required for --mode joint" >&2; exit 1; }
  [[ "$JOINT" =~ ^[1-6]$ ]] || { echo "ERROR: --joint must be 1-6" >&2; exit 1; }
fi

[[ "$REPEAT" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: --repeat must be a positive integer" >&2; exit 1; }

# Output root mirrors run_test.py logic.
if [[ -n "${TEST_LOG_ROOT:-}" ]]; then
  OUTPUT_ROOT="$TEST_LOG_ROOT/02-a1z/01-output"
else
  OUTPUT_ROOT="$HERE/output"
fi
OUTPUT_ROOT="$(cd "$HERE" && mkdir -p "$OUTPUT_ROOT" && cd "$OUTPUT_ROOT" && pwd)"
DATE=$(date +%Y-%m-%d)

# Optional --joint flag array (empty when not in joint mode)
JOINT_ARG=()
if [[ -n "$JOINT" ]]; then
  JOINT_ARG=("--joint" "$JOINT")
fi

# Gains-source flag array (empty = SDK defaults; mutually exclusive, checked above)
GAINS_ARGS=()
if [[ -n "$GAINS_FILE" ]]; then
  GAINS_ARGS=("--gains-file" "$GAINS_FILE")
elif [[ -n "$KP" || -n "$KD" ]]; then
  [[ -z "$KP" ]] || GAINS_ARGS+=("--kp" "$KP")
  [[ -z "$KD" ]] || GAINS_ARGS+=("--kd" "$KD")
fi

# Triangle leg vel-ff (square leg never uses vel-ff)
VEL_FF_ARGS=("--vel-ff")
if [[ "$NO_VEL_FF" == "1" ]]; then
  VEL_FF_ARGS=()
fi

# Feedforward mechanism flags (B1 G0-ext mechanism pairs, SOP-11 §7.4) —
# forwarded verbatim to every leg's run_test.py call.
MECH_ARGS=()
[[ -z "$COULOMB_FF" ]] || MECH_ARGS+=("--coulomb-ff" "$COULOMB_FF")
[[ -z "$KI_LEVEL" ]] || MECH_ARGS+=("--ki-level" "$KI_LEVEL")
[[ -z "$INTEGRAL_JOINTS" ]] || MECH_ARGS+=("--integral-joints" "$INTEGRAL_JOINTS")
[[ -z "$T_WIND_S" ]] || MECH_ARGS+=("--t-wind-s" "$T_WIND_S")
[[ -z "$CLAMP_SCALE" ]] || MECH_ARGS+=("--clamp-scale" "$CLAMP_SCALE")

run_dirs=()

for i in $(seq 1 "$REPEAT"); do
  echo ""
  echo "[verify] === repeat $i / $REPEAT ==="

  if [[ "$MODE" != "ee" ]]; then
    # Triangle leg (vel-ff unless --no-vel-ff, 15 deg, 4 s, 1 cycle, hold 0.5/0.5)
    tri_run="${RUN_PREFIX}-r${i}-tri-01"
    echo "[verify] triangle leg: $tri_run"
    ./a1z/analysis/test.sh --mode "$MODE" "${JOINT_ARG[@]}" \
              --wave triangle \
              --amp-deg 15 --period 4 --cycles 1 \
              --hold-pre 0.5 --hold-post 0.5 \
              "${VEL_FF_ARGS[@]}" \
              "${GAINS_ARGS[@]}" \
              "${MECH_ARGS[@]}" \
              --can "$CAN" \
              --run "$tri_run" \
              "${DRY_RUN[@]}"
    run_dirs+=("$OUTPUT_ROOT/${DATE}-run-${tri_run}")

    # Small step leg (no vel-ff, 2 deg, 4 s, 1 cycle, hold 0.5/0.5)
    step_run="${RUN_PREFIX}-r${i}-step-01"
    echo "[verify] small-step leg: $step_run"
    ./a1z/analysis/test.sh --mode "$MODE" "${JOINT_ARG[@]}" \
              --wave square \
              --amp-deg 2 --period 4 --cycles 1 \
              --hold-pre 0.5 --hold-post 0.5 \
              "${GAINS_ARGS[@]}" \
              "${MECH_ARGS[@]}" \
              --can "$CAN" \
              --run "$step_run" \
              "${DRY_RUN[@]}"
    run_dirs+=("$OUTPUT_ROOT/${DATE}-run-${step_run}")
  fi

  # EE tracking leg
  if [[ "$MODE" == "ee" || "$MODE" == "all" ]]; then
    ee_run="${RUN_PREFIX}-r${i}-ee-01"
    echo "[verify] EE tracking leg: $ee_run"
    ./a1z/analysis/test.sh --mode ee \
              "${GAINS_ARGS[@]}" \
              "${MECH_ARGS[@]}" \
              --can "$CAN" \
              --run "$ee_run" \
              "${DRY_RUN[@]}"
    run_dirs+=("$OUTPUT_ROOT/${DATE}-run-${ee_run}")
  fi
done

# Summary output directory
SUMMARY_DIR="$OUTPUT_ROOT/${DATE}-run-${RUN_PREFIX}-verify-summary"
mkdir -p "$SUMMARY_DIR"

SUMMARY_JSON="$SUMMARY_DIR/verify-summary.json"
SUMMARY_CSV="$SUMMARY_DIR/verify-summary.csv"
SUMMARY_TXT="$SUMMARY_DIR/verify-summary.txt"

echo ""
echo "[verify] generating summary ..."
SUMMARY_GAINS_ARGS=()
if [[ -n "$GAINS_FILE" ]]; then
  SUMMARY_GAINS_ARGS=("--gains-file" "$GAINS_FILE")
fi
python -m a1z.analysis.script.verify_summary \
  "${SUMMARY_GAINS_ARGS[@]}" \
  --output-format "$OUTPUT_FORMAT" \
  --output-json "$SUMMARY_JSON" \
  --output-csv "$SUMMARY_CSV" \
  --output-text "$SUMMARY_TXT" \
  "${run_dirs[@]}"

echo ""
echo "[verify] Complete. Summary written to:"
echo "  $SUMMARY_JSON"
echo "  $SUMMARY_CSV"
echo "  $SUMMARY_TXT"
