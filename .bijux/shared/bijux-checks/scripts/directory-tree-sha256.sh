#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 1 ]]; then
  echo "usage: directory-tree-sha256.sh <directory>" >&2
  exit 2
fi

target_dir="$1"
if [[ ! -d "${target_dir}" ]]; then
  echo "ERROR: missing directory ${target_dir}" >&2
  exit 1
fi

(
  cd "${target_dir}"
  find . \
    \( -type d \( \
      -name __pycache__ -o \
      -name .mypy_cache -o \
      -name .pytest_cache -o \
      -name .ruff_cache \
    \) -prune \) -o \
    \( -type f \
      ! -name .DS_Store \
      ! -name '*.py[co]' \
      -print \
    \) |
    LC_ALL=C sort |
    while IFS= read -r file_rel; do
      shasum -a 256 "${file_rel}"
    done
) | shasum -a 256 | awk '{print $1}'
