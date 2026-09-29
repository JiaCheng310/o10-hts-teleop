#!/usr/bin/env bash
set -euo pipefail
O10_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
  cat <<'EOF'
Usage: bash scripts/teleop.sh <right|left> <check|preview-check|preview|start|stop>
       bash scripts/teleop.sh stop

check          Check dependencies, selected O10 serial permissions and Quest; no motor commands.
preview-check  Check dependencies and Quest only; no O10 connection.
preview        Stream and retarget the selected hand; no O10 connection.
start          Enable the selected physical O10. Fresh HTS frames can move its fingers.
stop           Stop this launcher's HTS receiver and O10 driver.

Device identifiers: config/devices.env (see config/devices.env.example).
Default interpreter: .venv/bin/python; override with O10_PYTHON.
EOF
}
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" || $# == 0 ]]; then usage; exit 0; fi
side="${1}"
action="${2:-preview}"
if [[ "$side" == "stop" && $# == 1 ]]; then side=right; action=stop; fi
if [[ "$side" != "right" && "$side" != "left" ]]; then usage >&2; exit 2; fi
args=(--side "$side")
case "$action" in
  check) args+=(check);;
  preview-check) args+=(--hand-preview check);;
  preview) args+=(--hand-preview run);;
  start) args+=(run);;
  stop) args+=(stop);;
  *) usage >&2; exit 2;;
esac
if [[ $# -gt 2 ]]; then usage >&2; exit 2; fi
if [[ -f "$O10_PROJECT_ROOT/config/devices.env" ]]; then
  set -a
  source "$O10_PROJECT_ROOT/config/devices.env"
  set +a
fi
O10_INTERPRETER="${O10_PYTHON:-$O10_PROJECT_ROOT/.venv/bin/python}"
if [[ ! -x "$O10_INTERPRETER" ]]; then
  echo "Python environment missing: $O10_INTERPRETER. Follow README installation first." >&2
  exit 1
fi
export PYTHONPATH="$O10_PROJECT_ROOT:$O10_PROJECT_ROOT/third_party/dex-retargeting/src${PYTHONPATH:+:$PYTHONPATH}"
cd "$O10_PROJECT_ROOT"
exec "$O10_INTERPRETER" "$O10_PROJECT_ROOT/teleop/hts_local.py" "${args[@]}"
