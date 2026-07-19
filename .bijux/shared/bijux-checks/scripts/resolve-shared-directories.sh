#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
config_path="${2:-}"
requested_capabilities="${3:-}"

if [[ "${mode}" != "--all" && "${mode}" != "--select" ]]; then
  echo "usage: resolve-shared-directories.sh <--all|--select> <config> [capabilities]" >&2
  exit 2
fi
if [[ ! -f "${config_path}" ]]; then
  echo "shared standards config is unavailable: ${config_path}" >&2
  exit 1
fi

all_directories() {
  awk '
    /^directories:/ {active=1; next}
    /^(capabilities|remote):/ {active=0}
    active && /^  - / {
      sub(/^  - /, "")
      print
    }
  ' "${config_path}"
}

capability_names() {
  awk '
    /^capabilities:/ {active=1; next}
    /^remote:/ {active=0}
    active && /^  [a-z0-9-]+:$/ {
      name=$0
      sub(/^  /, "", name)
      sub(/:$/, "", name)
      print name
    }
  ' "${config_path}"
}

selected_directories() {
  local capabilities="common ${requested_capabilities}"
  local known_capabilities
  known_capabilities="$(capability_names)"

  for capability in ${capabilities}; do
    if ! grep -Fxq "${capability}" <<<"${known_capabilities}"; then
      echo "unknown shared standards capability: ${capability}" >&2
      exit 2
    fi
  done

  awk -v requested=" ${capabilities} " '
    /^capabilities:/ {active=1; next}
    /^remote:/ {active=0; capability=""; next}
    active && /^  [a-z0-9-]+:$/ {
      capability=$0
      sub(/^  /, "", capability)
      sub(/:$/, "", capability)
      next
    }
    active && /^    - / && index(requested, " " capability " ") {
      value=$0
      sub(/^    - /, "", value)
      if (!seen[value]++) {
        print value
      }
    }
  ' "${config_path}"
}

if [[ "${mode}" == "--all" || -z "${requested_capabilities}" ]]; then
  all_directories
else
  selected_directories
fi
