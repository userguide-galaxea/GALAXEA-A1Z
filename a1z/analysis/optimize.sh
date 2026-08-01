#!/usr/bin/env bash
# Thin wrapper around a1z.analysis.script.run_optimize (SOP-11 §3).
#
#   ./optimize.sh --joint 6 --name phaseA-J6 --n-trials 40
#   ./optimize.sh --resume $TEST_LOG_ROOT/02-a1z/02-para-opt/2026-07-30-run-opt-phaseA-J6
#   ./optimize.sh --joint 6 --name phaseA-J6-v5 --n-trials 20 \
#       --warm-start .../2026-07-30-run-opt-phaseA-J6/relabel-v5.csv
#   ./optimize.sh --help
#
# All flags pass straight through to run_optimize.py (see --help).
#
# Requires: conda env 'teleop' with optuna installed
#   pip install -e ".[optimize]"   (from GALAXEA-A1Z/)
set -euo pipefail

# Activate the teleop conda env if not already active.
if [[ "${CONDA_DEFAULT_ENV:-}" != "teleop" ]]; then
  # shellcheck disable=SC1091
  source "$(conda info --base)/etc/profile.d/conda.sh" 2>/dev/null || true
  conda activate teleop 2>/dev/null || true
fi

# Run from the GALAXEA-A1Z repo root so `a1z` is importable.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../.." && pwd)"
cd "$REPO_ROOT"

exec python -m a1z.analysis.script.run_optimize "$@"
