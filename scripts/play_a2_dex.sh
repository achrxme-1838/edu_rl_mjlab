#!/usr/bin/env bash
set -euo pipefail
# Keep ROS/Isaac and user-site Python packages out of this conda environment.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec env -u PYTHONPATH PYTHONNOUSERSITE=1 python "$script_dir/play_a2_dex.py" "$@"
