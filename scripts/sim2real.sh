#!/usr/bin/env bash
# Isolate shell options and exec even when launched with `source` or `.`.
(
  set -euo pipefail

  script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  controller="$script_dir/../deploy/robots/a2/build/a2_ctrl"

  if [[ ! -x "$controller" ]]; then
    printf 'a2_ctrl을 실행할 수 없습니다: %s\n먼저 README.md의 실기용 SDK·C++ 빌드 절에 따라 빌드하세요.\n' "$controller" >&2
    exit 1
  fi

  exec "$controller" "$@"
)
