#!/usr/bin/env bash
set -euo pipefail

repo_root="${1:-$(git rev-parse --show-toplevel 2>/dev/null || pwd)}"

# Discover python package roots from pyproject manifests.
# Dependabot expects absolute-style repo paths ("/path").
declare -a py_dirs_raw=()

if command -v rg >/dev/null 2>&1; then
  while IFS= read -r file; do
    py_dirs_raw+=("$(dirname "${file}")")
  done < <(cd "${repo_root}" && rg --files -g 'pyproject.toml')
else
  while IFS= read -r file; do
    file="${file#./}"
    py_dirs_raw+=("$(dirname "${file}")")
  done < <(cd "${repo_root}" && find . -name pyproject.toml -type f)
fi

declare -a py_dirs=()

if ((${#py_dirs_raw[@]} > 0)); then
  while IFS= read -r dir; do
    py_dirs+=("${dir}")
  done < <(
    printf '%s\n' "${py_dirs_raw[@]}" \
      | sed -E 's#^\.$#/#; s#^([^/].*)$#/\1#' \
      | sort -u
  )
fi

cat <<'YAML'
version: 2
updates:
  - package-ecosystem: github-actions
    directory: "/"
    schedule:
      interval: weekly
    open-pull-requests-limit: 10
    labels:
      - dependencies
      - github_actions

  - package-ecosystem: npm
    directory: "/configs"
    schedule:
      interval: weekly
    open-pull-requests-limit: 10
    labels:
      - dependencies
      - javascript
YAML

if ((${#py_dirs[@]} > 0)); then
  for dir in "${py_dirs[@]}"; do
    [[ -n "${dir}" ]] || continue
    cat <<YAML

  - package-ecosystem: pip
    directory: "${dir}"
    schedule:
      interval: weekly
    open-pull-requests-limit: 10
    labels:
      - dependencies
      - python
YAML
  done
fi
