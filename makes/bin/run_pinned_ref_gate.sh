#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
pinned_ref="${PINNED_REF:-${TEST_ALL_FROZEN_REF:-HEAD}}"
pinned_target="${PINNED_REF_GATE_TARGET:-test-all}"
full_sha="$(git -C "${repo_root}" rev-parse "${pinned_ref}")"
short_sha="$(git -C "${repo_root}" rev-parse --short=9 "${full_sha}")"
artifact_root="${repo_root}/artifacts/${short_sha}"
pinned_repo_dir="${artifact_root}/frozen-repo"
pinned_repo_lock_dir="${artifact_root}/frozen-repo.lock"
background_dir="${artifact_root}/background"
rs_artifact_root="${artifact_root}/rust"
console_log="${background_dir}/${pinned_target}.console.log"
pid_file="${background_dir}/${pinned_target}.pid"
meta_file="${background_dir}/${pinned_target}.meta"
status_file="${background_dir}/${pinned_target}.exit.status"
launcher_file="${background_dir}/${pinned_target}.launch.sh"
artifact_target_dir="${artifact_root}/target/${pinned_target}"
artifact_cargo_home="${artifact_root}/cargo/home/${pinned_target}"
artifact_tmp_dir="${artifact_root}/tmp/${pinned_target}"

case "${pinned_target}" in
  test-all)
    primary_report="${rs_artifact_root}/test/${short_sha}/nextest-all.log"
    gate_description="full test-all"
    ;;
  lint)
    primary_report="${rs_artifact_root}/lint/${short_sha}/report.txt"
    gate_description="lint"
    ;;
  audit)
    primary_report="${rs_artifact_root}/audit/${short_sha}/report.txt"
    gate_description="audit"
    ;;
  *)
    echo "unsupported PINNED_REF_GATE_TARGET: ${pinned_target}" >&2
    exit 2
    ;;
esac

require_tool() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "$1 is required but not installed" >&2
    exit 1
  fi
}

require_tool git
require_tool make
require_tool python3

lock_is_held=0

acquire_lock() {
  local lock_dir="$1"
  while ! mkdir "${lock_dir}" 2>/dev/null; do
    if [ -f "${lock_dir}/pid" ]; then
      local lock_pid
      lock_pid="$(cat "${lock_dir}/pid")"
      if [ -n "${lock_pid}" ] && ! kill -0 "${lock_pid}" 2>/dev/null; then
        rm -rf "${lock_dir}"
        continue
      fi
    fi
    sleep 1
  done

  printf '%s\n' "$$" >"${lock_dir}/pid"
  lock_is_held=1
}

release_lock() {
  local lock_dir="$1"
  if [ "${lock_is_held}" -eq 1 ]; then
    rm -rf "${lock_dir}"
    lock_is_held=0
  fi
}

trap 'release_lock "${pinned_repo_lock_dir}"' EXIT

mkdir -p \
  "${artifact_root}" \
  "${background_dir}" \
  "${artifact_target_dir}" \
  "${artifact_cargo_home}" \
  "${artifact_tmp_dir}" \
  "$(dirname "${primary_report}")"

if [ -f "${pid_file}" ]; then
  existing_pid="$(cat "${pid_file}")"
  if [ -n "${existing_pid}" ] && kill -0 "${existing_pid}" 2>/dev/null; then
    echo "background ${pinned_target} is already running for ${short_sha} (pid ${existing_pid})" >&2
    echo "console log: ${console_log}" >&2
    echo "primary report: ${primary_report}" >&2
    exit 1
  fi
fi

acquire_lock "${pinned_repo_lock_dir}"

if [ -d "${pinned_repo_dir}" ]; then
  if ! git -C "${pinned_repo_dir}" rev-parse --show-toplevel >/dev/null 2>&1; then
    echo "existing path is not a git repository: ${pinned_repo_dir}" >&2
    exit 1
  fi

  pinned_repo_sha="$(git -C "${pinned_repo_dir}" rev-parse HEAD)"
  if [ "${pinned_repo_sha}" != "${full_sha}" ]; then
    echo "existing pinned repo points to ${pinned_repo_sha}, expected ${full_sha}: ${pinned_repo_dir}" >&2
    exit 1
  fi

  if [ -n "$(git -C "${pinned_repo_dir}" status --short)" ]; then
    echo "existing pinned repo is dirty: ${pinned_repo_dir}" >&2
    exit 1
  fi
else
  git clone --no-local --no-checkout --quiet "${repo_root}" "${pinned_repo_dir}"
  git -C "${pinned_repo_dir}" checkout --detach --force "${full_sha}" >/dev/null
fi

release_lock "${pinned_repo_lock_dir}"

cat >"${meta_file}" <<EOF
ref=${pinned_ref}
commit=${full_sha}
short_commit=${short_sha}
repo_root=${repo_root}
pinned_repo=${pinned_repo_dir}
pinned_target=${pinned_target}
artifact_root=${artifact_root}
artifact_target_dir=${artifact_target_dir}
artifact_cargo_home=${artifact_cargo_home}
artifact_tmp_dir=${artifact_tmp_dir}
rs_artifact_root=${rs_artifact_root}
console_log=${console_log}
primary_report=${primary_report}
status_file=${status_file}
launcher=${launcher_file}
EOF

rm -f "${status_file}"
cat >"${launcher_file}" <<EOF
#!/usr/bin/env bash
set -euo pipefail

cd "${pinned_repo_dir}"

export ARTIFACT_ROOT="${artifact_root}"
export CARGO_TARGET_DIR="${artifact_target_dir}"
export CARGO_HOME="${artifact_cargo_home}"
export TMPDIR="${artifact_tmp_dir}"
export TMP="${artifact_tmp_dir}"
export TEMP="${artifact_tmp_dir}"
export RS_ARTIFACT_ROOT="${rs_artifact_root}"
export RS_RUN_ID="${short_sha}"

printf '%s\n' "pinned-ref ${pinned_target} start: ${short_sha}"
printf '%s\n' "pinned repo: ${pinned_repo_dir}"
printf '%s\n' "artifact root: ${artifact_root}"
printf '%s\n' "cargo target dir: ${artifact_target_dir}"
printf '%s\n' "primary report: ${primary_report}"

status=0
set +e
make "${pinned_target}"
status=\$?
set -e

printf '%s\n' "\${status}" >"${status_file}"
printf '%s\n' "pinned-ref ${pinned_target} exit: \${status}"
exit "\${status}"
EOF
chmod +x "${launcher_file}"

background_pid="$(
  python3 - "${launcher_file}" "${console_log}" <<'PY'
import subprocess
import sys

launcher_path = sys.argv[1]
console_path = sys.argv[2]

with open(console_path, "wb", buffering=0) as console_file:
    process = subprocess.Popen(
        ["/bin/bash", launcher_path],
        stdin=subprocess.DEVNULL,
        stdout=console_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        close_fds=True,
    )

print(process.pid)
PY
)"
printf '%s\n' "${background_pid}" >"${pid_file}"

printf '%s\n' "started background ${gate_description} for ${short_sha}"
printf '%s\n' "ref: ${pinned_ref}"
printf '%s\n' "commit: ${full_sha}"
printf '%s\n' "pinned repo: ${pinned_repo_dir}"
printf '%s\n' "artifact root: ${artifact_root}"
printf '%s\n' "console log: ${console_log}"
printf '%s\n' "primary report: ${primary_report}"
printf '%s\n' "pid: ${background_pid}"
