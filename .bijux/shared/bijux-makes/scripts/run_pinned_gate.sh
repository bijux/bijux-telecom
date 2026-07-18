#!/usr/bin/env bash
set -euo pipefail

require_tool() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "required tool is unavailable: $1" >&2
    exit 1
  fi
}

require_tool bash
require_tool git
require_tool make
require_tool python3

repo_root="$(git rev-parse --show-toplevel)"
pinned_ref="${PINNED_REF:-HEAD}"
pinned_target="${PINNED_GATE_TARGET:-}"
allowed_targets="${PINNED_ALLOWED_TARGETS:-}"

if [[ ! "${pinned_target}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
  echo "PINNED_GATE_TARGET must name one Make target" >&2
  exit 2
fi

target_allowed=0
for allowed_target in ${allowed_targets}; do
  if [[ "${pinned_target}" == "${allowed_target}" ]]; then
    target_allowed=1
    break
  fi
done
if [[ "${target_allowed}" -ne 1 ]]; then
  echo "PINNED_GATE_TARGET is not allowed: ${pinned_target}" >&2
  exit 2
fi

full_sha="$(git -C "${repo_root}" rev-parse "${pinned_ref}^{commit}")"
short_sha="$(git -C "${repo_root}" rev-parse --short=9 "${full_sha}")"
artifact_root="${repo_root}/artifacts/${short_sha}"
pinned_repo_dir="${artifact_root}/frozen-repo"
lock_dir="${artifact_root}/frozen-repo.lock"
background_dir="${artifact_root}/background"
console_log="${background_dir}/${pinned_target}.console.log"
pid_file="${background_dir}/${pinned_target}.pid"
meta_file="${background_dir}/${pinned_target}.meta"
status_file="${background_dir}/${pinned_target}.exit.status"
launcher_file="${background_dir}/${pinned_target}.launch.sh"

mkdir -p "${artifact_root}" "${background_dir}"

lock_held=0
release_lock() {
  if [[ "${lock_held}" -eq 1 ]]; then
    rm -rf "${lock_dir}"
    lock_held=0
  fi
}
trap release_lock EXIT

while ! mkdir "${lock_dir}" 2>/dev/null; do
  if [[ -f "${lock_dir}/pid" ]]; then
    lock_pid="$(cat "${lock_dir}/pid")"
    if [[ -n "${lock_pid}" ]] && ! kill -0 "${lock_pid}" 2>/dev/null; then
      rm -rf "${lock_dir}"
      continue
    fi
  fi
  sleep 1
done
printf '%s\n' "$$" >"${lock_dir}/pid"
lock_held=1

if [[ -d "${pinned_repo_dir}" ]]; then
  existing_sha="$(git -C "${pinned_repo_dir}" rev-parse HEAD 2>/dev/null || true)"
  if [[ "${existing_sha}" != "${full_sha}" ]]; then
    echo "pinned source commit mismatch: ${pinned_repo_dir}" >&2
    exit 1
  fi
  if [[ -n "$(git -C "${pinned_repo_dir}" status --short)" ]]; then
    echo "pinned source is dirty: ${pinned_repo_dir}" >&2
    exit 1
  fi
else
  git clone --no-local --no-checkout --quiet "${repo_root}" "${pinned_repo_dir}"
  git -C "${pinned_repo_dir}" checkout --detach --force "${full_sha}" >/dev/null
fi

release_lock

if [[ -f "${pid_file}" ]]; then
  existing_pid="$(cat "${pid_file}")"
  if [[ -n "${existing_pid}" ]] && kill -0 "${existing_pid}" 2>/dev/null; then
    echo "${pinned_target} is already running for ${short_sha}: pid ${existing_pid}" >&2
    exit 1
  fi
fi

cat >"${meta_file}" <<EOF
ref=${pinned_ref}
commit=${full_sha}
target=${pinned_target}
source=${pinned_repo_dir}
artifact_root=${artifact_root}
console_log=${console_log}
status_file=${status_file}
EOF

rm -f "${status_file}"
cat >"${launcher_file}" <<EOF
#!/usr/bin/env bash
set -euo pipefail
cd "${pinned_repo_dir}"

unset \
  PROJECT_ROOT \
  ARTIFACT_ROOT \
  RUN_ID \
  RS_ARTIFACT_ROOT \
  RS_RUN_ID \
  RS_TARGET_DIR \
  RS_CARGO_HOME \
  RS_TMP_DIR \
  RS_NEXTEST_CACHE_DIR \
  RS_NEXTEST_CONFIG_HOME \
  NEXTEST_CONFIG_FILE \
  NEXTEST_SLOW_ROSTER \
  NEXTEST_EXPR_BIN \
  RUST_DENY_CONFIG \
  RUSTFMT_CONFIG \
  RUST_CLIPPY_CONFIG_DIR

export PROJECT_ROOT="${pinned_repo_dir}"
export ARTIFACT_ROOT="${artifact_root}"
export RUN_ID="${short_sha}"
status=0
set +e
make "${pinned_target}"
status=\$?
set -e
printf '%s\n' "\${status}" >"${status_file}"
exit "\${status}"
EOF
chmod +x "${launcher_file}"

background_pid="$(
  python3 - "${launcher_file}" "${console_log}" <<'PY'
import subprocess
import sys

launcher_path, console_path = sys.argv[1:3]
with open(console_path, "wb", buffering=0) as console:
    process = subprocess.Popen(
        ["/bin/bash", launcher_path],
        stdin=subprocess.DEVNULL,
        stdout=console,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        close_fds=True,
    )
print(process.pid)
PY
)"
printf '%s\n' "${background_pid}" >"${pid_file}"

printf '%s\n' \
  "started ${pinned_target} for ${short_sha}" \
  "source: ${pinned_repo_dir}" \
  "artifacts: ${artifact_root}" \
  "console: ${console_log}" \
  "status: ${status_file}" \
  "pid: ${background_pid}"
