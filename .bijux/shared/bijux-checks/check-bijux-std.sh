#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
bijux_std_artifact_root="${repo_root}/artifacts/bijux-std"
mkdir -p "${bijux_std_artifact_root}/pycache"
export PYTHONPYCACHEPREFIX="${PYTHONPYCACHEPREFIX:-${bijux_std_artifact_root}/pycache}"
default_config_path="${repo_root}/.bijux/shared/bijux-checks/bijux-std-checks.yml"
if [[ ! -f "${default_config_path}" ]]; then
  default_config_path="${repo_root}/shared/bijux-checks/bijux-std-checks.yml"
fi
config_path="${BIJUX_STD_CONFIG:-${default_config_path}}"

if [[ ! -f "${config_path}" ]]; then
  echo "ERROR: missing config ${config_path}" >&2
  exit 1
fi

directory_resolver="${script_dir}/scripts/resolve-shared-directories.sh"
if [[ ! -x "${directory_resolver}" ]]; then
  echo "ERROR: shared directory resolver is unavailable: ${directory_resolver}" >&2
  exit 1
fi

read_scalar() {
  local key="$1"
  awk -F': ' -v key="${key}" '$1 == key {print $2; exit}' "${config_path}" | tr -d '"'
}

read_directories() {
  "${directory_resolver}" --all "${config_path}"
}

read_selected_directories() {
  "${directory_resolver}" --select "${config_path}" "${BIJUX_STD_CAPABILITIES:-}"
}

resolve_local_rel() {
  local rel="$1"
  if [[ "${rel}" == shared/* && -d "${repo_root}/.bijux/shared" && "$(basename "${repo_root}")" != "bijux-std" ]]; then
    printf '.bijux/%s\n' "${rel}"
    return
  fi
  printf '%s\n' "${rel}"
}

verify_no_legacy_root_shared_dirs() {
  if [[ -d "${repo_root}/.bijux/shared" && "$(basename "${repo_root}")" != "bijux-std" ]]; then
    if [[ -d "${repo_root}/shared" ]]; then
      echo "ERROR: legacy root shared directory present: shared/" >&2
      echo "Hint: remove root shared and keep only .bijux/shared/*" >&2
      exit 1
    fi

    local has_legacy=0
    while IFS= read -r remote_dir_rel; do
      if [[ "${remote_dir_rel}" == shared/* && -d "${repo_root}/${remote_dir_rel}" ]]; then
        echo "ERROR: legacy root managed directory present: ${remote_dir_rel}" >&2
        has_legacy=1
      fi
    done < <(read_directories)

    if [[ "${has_legacy}" == "1" ]]; then
      echo "Hint: remove root shared managed directories and keep only .bijux/shared/*" >&2
      exit 1
    fi
  fi
}

manifest_rel="$(read_scalar manifest)"
git_url_default="$(read_scalar '  git_url')"
default_ref="$(read_scalar '  default_ref')"

std_ref="${BIJUX_STD_REF:-${default_ref}}"
std_git_url="${BIJUX_STD_GIT_URL:-${git_url_default}}"
std_root="${BIJUX_STD_ROOT:-${repo_root}/../bijux-std}"
strict_remote="${BIJUX_STD_STRICT_REMOTE:-0}"
require_remote_match="${BIJUX_STD_REQUIRE_REMOTE_MATCH:-0}"
selected_directories="$(read_selected_directories)"
manifest_local_rel="$(resolve_local_rel "${manifest_rel}")"
manifest_path="${repo_root}/${manifest_local_rel}"

if [[ ! -f "${manifest_path}" ]]; then
  echo "ERROR: missing local manifest ${manifest_path}" >&2
  echo "Hint: run make bijux-std-update" >&2
  exit 1
fi

directory_tree_sha256() {
  local target_dir="$1"
  if [[ ! -d "${target_dir}" ]]; then
    echo "ERROR: missing directory ${target_dir}" >&2
    exit 1
  fi

  (
    cd "${target_dir}"
    find . -type f -print | LC_ALL=C sort | while IFS= read -r file_rel; do
      shasum -a 256 "${file_rel}"
    done
  ) | shasum -a 256 | awk '{print $1}'
}

manifest_sha_for_dir() {
  local manifest_file="$1"
  local dir_rel="$2"
  awk -v dir_rel="${dir_rel}" '$2 == dir_rel { print $1 }' "${manifest_file}"
}

verify_dir_against_manifests() {
  local remote_dir_rel="$1"
  local remote_manifest="$2"

  local local_dir_rel
  local local_expected
  local remote_expected
  local actual_sha
  local expected_sha
  local local_dir_abs

  local_dir_rel="$(resolve_local_rel "${remote_dir_rel}")"
  local_dir_abs="${repo_root}/${local_dir_rel}"

  local_expected="$(manifest_sha_for_dir "${manifest_path}" "${remote_dir_rel}")"
  if [[ -z "${local_expected}" && "${local_dir_rel}" != "${remote_dir_rel}" ]]; then
    local_expected="$(manifest_sha_for_dir "${manifest_path}" "${local_dir_rel}")"
  fi
  remote_expected="$(manifest_sha_for_dir "${remote_manifest}" "${remote_dir_rel}")"

  if [[ -z "${local_expected}" ]]; then
    echo "ERROR: local manifest missing entry for ${remote_dir_rel}" >&2
    exit 1
  fi
  if [[ -z "${remote_expected}" ]]; then
    if [[ "${require_remote_match}" == "1" ]]; then
      echo "ERROR: remote manifest missing entry for ${remote_dir_rel}" >&2
      exit 1
    fi
    remote_expected="${local_expected}"
    echo "⚠ remote manifest missing entry for ${remote_dir_rel}" >&2
    echo "  continuing because BIJUX_STD_REQUIRE_REMOTE_MATCH=0" >&2
  fi

  actual_sha="$(directory_tree_sha256 "${local_dir_abs}")"

  expected_sha="${remote_expected}"
  if [[ "${local_expected}" != "${remote_expected}" ]]; then
    if [[ "${require_remote_match}" == "1" ]]; then
      echo "ERROR: local manifest drift for ${local_dir_rel}" >&2
      echo "Local manifest:  ${local_expected}" >&2
      echo "Remote manifest: ${remote_expected}" >&2
      exit 1
    fi
    expected_sha="${local_expected}"
    echo "⚠ remote manifest differs for ${local_dir_rel}" >&2
    echo "  local:  ${local_expected}" >&2
    echo "  remote: ${remote_expected}" >&2
    echo "  continuing because BIJUX_STD_REQUIRE_REMOTE_MATCH=0" >&2
  fi

  if [[ "${actual_sha}" != "${expected_sha}" ]]; then
    echo "ERROR: shared directory drift for ${local_dir_rel}" >&2
    echo "Expected: ${expected_sha}" >&2
    echo "Actual:   ${actual_sha}" >&2
    exit 1
  fi

  echo "✔ ${local_dir_rel} matches expected manifest (${expected_sha})"
}

verify_canonical_mermaid_init() {
  local shared_mermaid_rel
  local shared_mermaid_path
  local docs_mermaid_path="${repo_root}/docs/assets/javascripts/mermaid-init.js"
  shared_mermaid_rel="$(resolve_local_rel "shared/bijux-docs/scripts/mermaid-init.js")"
  shared_mermaid_path="${repo_root}/${shared_mermaid_rel}"

  if [[ ! -f "${shared_mermaid_path}" ]]; then
    echo "ERROR: missing canonical Mermaid initializer ${shared_mermaid_path}" >&2
    exit 1
  fi

  if [[ -f "${docs_mermaid_path}" ]]; then
    if ! cmp -s "${shared_mermaid_path}" "${docs_mermaid_path}"; then
      echo "ERROR: docs Mermaid initializer drift" >&2
      echo "Expected source: ${shared_mermaid_path}" >&2
      echo "Drifted target: ${docs_mermaid_path}" >&2
      echo "Hint: synchronize docs/assets/javascripts/mermaid-init.js from ${shared_mermaid_rel}" >&2
      exit 1
    fi
    echo "✔ Mermaid initializer matches shared canonical source"
    return
  fi

  echo "✔ Mermaid initializer canonical source is present (docs mirror not required)"
}

verify_homepage_sidebar_collapse_contract() {
  local responsive_css_rel
  local responsive_css_path
  responsive_css_rel="$(resolve_local_rel "shared/bijux-docs/styles/08-responsive.css")"
  responsive_css_path="${repo_root}/${responsive_css_rel}"

  if [[ ! -f "${responsive_css_path}" ]]; then
    echo "ERROR: missing responsive stylesheet ${responsive_css_path}" >&2
    exit 1
  fi

  if ! grep -qF '[data-bijux-viewport="normal"] .md-main__inner:has(.md-sidebar--primary .bijux-nav--scoped[data-bijux-nav-empty="true"])' "${responsive_css_path}"; then
    echo "ERROR: missing homepage/sidebar collapse selector for normal viewport" >&2
    exit 1
  fi

  if ! grep -qF '[data-bijux-viewport="desktop"] .md-main__inner:has(.md-sidebar--primary .bijux-nav--scoped[data-bijux-nav-empty="true"])' "${responsive_css_path}"; then
    echo "ERROR: missing homepage/sidebar collapse selector for desktop viewport" >&2
    exit 1
  fi

  if ! grep -qF '[data-bijux-viewport="wide"] .md-main__inner:has(.md-sidebar--primary .bijux-nav--scoped[data-bijux-nav-empty="true"])' "${responsive_css_path}"; then
    echo "ERROR: missing homepage/sidebar collapse selector for wide viewport" >&2
    exit 1
  fi

  if ! grep -qF 'grid-template-columns: minmax(0, 1fr);' "${responsive_css_path}"; then
    echo "ERROR: homepage/sidebar collapse contract missing single-column layout rule" >&2
    exit 1
  fi

  if ! grep -qF 'max-width: 100%;' "${responsive_css_path}"; then
    echo "ERROR: homepage/sidebar collapse contract missing full-width content rule" >&2
    exit 1
  fi

  echo "✔ Homepage scoped-nav-empty sidebar collapse contract is enforced"
}

verify_workflow_run_shell_preambles() {
  local manifest_shell_breaks
  local shared_workflow_rel
  local shared_workflow_dir
  manifest_shell_breaks="$(grep -nE '"run": "set -euo pipefail [^;&"]' "${repo_root}/.github/standards/repo-config.manifest.json" || true)"
  if [[ -n "${manifest_shell_breaks}" ]]; then
    echo "ERROR: malformed workflow shell preamble in standards manifest" >&2
    echo "${manifest_shell_breaks}" >&2
    echo 'Hint: use `set -euo pipefail; ...` or `set -euo pipefail && ...`, not `set -euo pipefail command ...`.' >&2
    exit 1
  fi

  local shared_workflow_breaks
  shared_workflow_rel="$(resolve_local_rel "shared/bijux-gh/workflows")"
  shared_workflow_dir="${repo_root}/${shared_workflow_rel}"
  shared_workflow_breaks="$(grep -RInE 'run: "?set -euo pipefail [^;&"]' "${shared_workflow_dir}" || true)"
  if [[ -n "${shared_workflow_breaks}" ]]; then
    echo "ERROR: malformed workflow shell preamble in shared GitHub workflow templates" >&2
    echo "${shared_workflow_breaks}" >&2
    echo 'Hint: use `set -euo pipefail; ...` or `set -euo pipefail && ...`, not `set -euo pipefail command ...`.' >&2
    exit 1
  fi

  echo "✔ Workflow shell preambles are executable"
}

verify_release_pypi_toolchain_inheritance() {
  local workflow_rel
  local workflow_path
  workflow_rel="$(resolve_local_rel "shared/bijux-gh/workflows/release-pypi.yml")"
  workflow_path="${repo_root}/${workflow_rel}"

  if [[ ! -f "${workflow_path}" ]]; then
    echo "ERROR: missing shared release-pypi workflow ${workflow_path}" >&2
    exit 1
  fi

  if grep -qF '"1.85.0"' "${workflow_path}"; then
    echo "ERROR: shared release-pypi workflow still hardcodes the stale Rust 1.85.0 fallback" >&2
    echo "Hint: inherit the PyPI Rust toolchain from BIJUX_RELEASE_RUST_TOOLCHAIN before using a built-in default." >&2
    exit 1
  fi

  if ! grep -qF 'release_rust_toolchain="$(from_values "" "${BIJUX_RELEASE_RUST_TOOLCHAIN:-}" "${{ vars.BIJUX_RELEASE_RUST_TOOLCHAIN || '\'''\'' }}" "1.86.0")"' "${workflow_path}"; then
    echo "ERROR: shared release-pypi workflow must resolve a shared release Rust toolchain fallback" >&2
    exit 1
  fi

  if ! grep -qF 'rust_toolchain="$(from_values "" "${BIJUX_PYPI_RUST_TOOLCHAIN:-}" "${{ vars.BIJUX_PYPI_RUST_TOOLCHAIN || '\'''\'' }}" "${release_rust_toolchain}")"' "${workflow_path}"; then
    echo "ERROR: shared release-pypi workflow must inherit the release Rust toolchain when BIJUX_PYPI_RUST_TOOLCHAIN is unset" >&2
    exit 1
  fi

  echo "✔ Shared release-pypi workflow inherits the release Rust toolchain"
}

verify_release_env_shell_safety() {
  python3 - "${repo_root}" "${tmp_dir}" <<'PY'
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

repo_root = Path(sys.argv[1])
tmp_dir = Path(sys.argv[2])
manifest_path = repo_root / ".github/standards/repo-config.manifest.json"
render_script_path = repo_root / ".github/scripts/render_repo_configs.py"

spec = importlib.util.spec_from_file_location("render_repo_configs", render_script_path)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)

manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
for repo in manifest["repositories"]:
    release_content = module.render_release_env(repo.get("release_env", []))
    release_path = tmp_dir / f"{repo['name']}.release.env"
    release_path.write_text(release_content, encoding="utf-8")
    result = subprocess.run(
        [
            "bash",
            "-lc",
            "set -euo pipefail; source \"$1\"",
            "bash",
            str(release_path),
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        sys.stderr.write(
            f"ERROR: rendered release.env is not shell-safe for {repo['name']}\n"
        )
        if result.stderr:
            sys.stderr.write(result.stderr)
        raise SystemExit(result.returncode)

print("✔ Rendered release.env files are shell-safe")
PY
}

tmp_dir="$(mktemp -d "${bijux_std_artifact_root}/check.XXXXXX")"
tmp_manifest="${tmp_dir}/manifest.txt"
cleanup() {
  rm -rf "${tmp_dir}"
}
trap cleanup EXIT

clone_std_ref() {
  local destination="${tmp_dir}/bijux-std"
  git init --quiet "${destination}"
  git -C "${destination}" remote add origin "${std_git_url}"
  git -C "${destination}" fetch --quiet --depth 1 origin "${std_ref}"
  git -C "${destination}" checkout --quiet --detach FETCH_HEAD
}

if ! clone_std_ref >/dev/null 2>&1; then
  local_std_manifest="${std_root}/${manifest_rel}"
  if [[ "${strict_remote}" == "1" ]]; then
    echo "ERROR: failed to clone ${std_git_url}@${std_ref} (strict remote mode)" >&2
    exit 1
  fi
  if [[ -f "${local_std_manifest}" ]]; then
    cp "${local_std_manifest}" "${tmp_manifest}"
    echo "→ remote manifest unavailable; using local bijux-std manifest at ${local_std_manifest}"
  else
    echo "ERROR: failed to clone ${std_git_url}@${std_ref}" >&2
    echo "ERROR: local fallback manifest not found at ${local_std_manifest}" >&2
    exit 1
  fi
else
  remote_manifest_path="${tmp_dir}/bijux-std/${manifest_rel}"
  if [[ ! -f "${remote_manifest_path}" ]]; then
    echo "ERROR: missing manifest ${manifest_rel} in ${std_git_url}@${std_ref}" >&2
    exit 1
  fi
  cp "${remote_manifest_path}" "${tmp_manifest}"
fi

while IFS= read -r dir_rel; do
  verify_dir_against_manifests "${dir_rel}" "${tmp_manifest}"
done <<<"${selected_directories}"

verify_no_legacy_root_shared_dirs
if grep -Fxq "shared/bijux-docs" <<<"${selected_directories}"; then
  verify_canonical_mermaid_init
  verify_homepage_sidebar_collapse_contract
fi
verify_workflow_run_shell_preambles
verify_release_pypi_toolchain_inheritance
verify_release_env_shell_safety

echo "✔ bijux-std check passed (ref=${std_ref}, manifest=${manifest_rel}, remote=${git_url_default})"
