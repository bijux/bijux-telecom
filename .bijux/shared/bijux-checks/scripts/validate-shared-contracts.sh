#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
artifacts_dir="${BIJUX_CONTRACT_ARTIFACTS_DIR:-${repo_root}/artifacts/contracts/shared}"

if [[ -d "${repo_root}/.bijux/shared/bijux-checks" ]]; then
  shared_root="${repo_root}/.bijux/shared"
elif [[ -d "${repo_root}/shared/bijux-checks" ]]; then
  shared_root="${repo_root}/shared"
else
  echo "ERROR: unable to locate managed Bijux shared directories" >&2
  exit 2
fi

for command_name in bash python3 shellcheck; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "ERROR: ${command_name} is required for shared contract validation" >&2
    exit 2
  fi
done

mkdir -p "${artifacts_dir}/pycache"

shell_files=()
while IFS= read -r -d '' path; do
  shell_files+=("${path}")
done < <(find "${shared_root}" -type f -name '*.sh' -print0 | sort -z)

python_files=()
while IFS= read -r -d '' path; do
  python_files+=("${path}")
done < <(find "${shared_root}" -type f -name '*.py' -print0 | sort -z)

json_files=()
while IFS= read -r -d '' path; do
  json_files+=("${path}")
done < <(find "${shared_root}" -type f -name '*.json' -print0 | sort -z)

if [[ ${#shell_files[@]} -eq 0 ]]; then
  echo "ERROR: managed shared directories contain no shell contracts" >&2
  exit 2
fi

for shell_file in "${shell_files[@]}"; do
  bash -n "${shell_file}"
done
shellcheck --severity=warning "${shell_files[@]}"

if [[ ${#python_files[@]} -gt 0 ]]; then
  PYTHONPYCACHEPREFIX="${artifacts_dir}/pycache" \
    python3 -m py_compile "${python_files[@]}"
fi

if [[ ${#json_files[@]} -gt 0 ]]; then
  python3 - "${json_files[@]}" <<'PY'
import json
import sys
from pathlib import Path

for raw_path in sys.argv[1:]:
    path = Path(raw_path)
    with path.open(encoding="utf-8") as stream:
        json.load(stream)
PY
fi

printf 'shared-contracts: %d shell, %d python, %d json files validated\n' \
  "${#shell_files[@]}" "${#python_files[@]}" "${#json_files[@]}"
