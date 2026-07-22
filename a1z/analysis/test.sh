#!/usr/bin/env bash
# Thin wrapper around a1z.analysis.script.run_test (SOP-03 §6).
#
#   ./test.sh --mode joint  --joint 3 --run 01
#   ./test.sh --mode joints --run 02
#   ./test.sh --mode ee     --run 03
#   ./test.sh --mode ee     --dry-run          # IK gate + preview, no hardware
#   ./test.sh --mode all    --run 04
#
# All flags pass straight through to run_test.py (see --help).
#
# Optional env var TEST_LOG_ROOT (see 01-docs/04-sops/07-测试数据管理-SOP.md):
# if exported, run dirs go to $TEST_LOG_ROOT/02-a1z/01-output/ instead of the
# local (gitignored) a1z/analysis/output/. It is inherited by the python child
# as-is, so just `export TEST_LOG_ROOT=/path/to/02-test-log` before running.
# The same var also redirects can_capture.py --out / can_analyze.py --json-out.
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

exec python -m a1z.analysis.script.run_test "$@"
