#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec env -u PYTHONPATH PYTHONNOUSERSITE=1 python "$script_dir/play_a2_edu.py" "$@"
