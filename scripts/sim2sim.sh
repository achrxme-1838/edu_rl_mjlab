#!/usr/bin/env bash
# Isolate shell options and exec even when launched with `source` or `.`.
(
  set -euo pipefail
  script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  exec env -u PYTHONPATH PYTHONNOUSERSITE=1 python "$script_dir/sim2sim.py" "$@"
)
