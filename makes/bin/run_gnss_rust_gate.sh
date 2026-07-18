#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
shared_gate="${repo_root}/.bijux/shared/bijux-makes-rs/scripts/rust_gate.sh"

if [[ ! -x "${shared_gate}" ]]; then
  echo "shared Rust gate is unavailable: ${shared_gate}" >&2
  exit 1
fi

if [[ "${1:-}" == "audit" ]]; then
  artifact_root="${RS_ARTIFACT_ROOT:-${repo_root}/artifacts/rust}"
  cargo_home="${RS_CARGO_HOME:-${artifact_root}/cargo/home}"
  target_dir="${RS_TARGET_DIR:-${artifact_root}/target}"
  tmp_dir="${RS_TMP_DIR:-${artifact_root}/tmp}"
  mkdir -p "${cargo_home}" "${target_dir}" "${tmp_dir}"

  export CARGO_HOME="${cargo_home}"
  export CARGO_TARGET_DIR="${target_dir}"
  export TMPDIR="${tmp_dir}"
  audit_ignore_args="$(
    cargo run --locked -q -p bijux-gnss-dev -- audit-ignore-args
  )"
  export RUST_AUDIT_IGNORE_ARGS="${audit_ignore_args}"
fi

exec "${shared_gate}" "$@"
