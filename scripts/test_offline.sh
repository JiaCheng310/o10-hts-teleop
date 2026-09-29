#!/usr/bin/env bash
set -euo pipefail
O10_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
O10_INTERPRETER="${O10_PYTHON:-$O10_PROJECT_ROOT/.venv/bin/python}"
export PYTHONPATH="$O10_PROJECT_ROOT:$O10_PROJECT_ROOT/teleop:$O10_PROJECT_ROOT/tests:$O10_PROJECT_ROOT/third_party/dex-retargeting/src${PYTHONPATH:+:$PYTHONPATH}"
cd "$O10_PROJECT_ROOT"
exec "$O10_INTERPRETER" -m unittest -v test_hts test_hts_right test_direct_sender test_device_config test_quest_recording test_quest_native_capture
