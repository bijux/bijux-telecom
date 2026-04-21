#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
consumer_path="${BIJUX_CHECKS_CONSUMER:-${repo_root}/.bijux/checks.consumer.json}"
registry_dir="${BIJUX_CHECKS_REGISTRY_DIR:-${repo_root}/shared/bijux-checks/registry}"
artifacts_dir="${BIJUX_CHECKS_ARTIFACTS_DIR:-${repo_root}/artifacts/bijux-checks}"

if [[ ! -d "${registry_dir}" && -d "${repo_root}/.bijux/shared/bijux-checks/registry" ]]; then
  registry_dir="${repo_root}/.bijux/shared/bijux-checks/registry"
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --consumer)
      consumer_path="$2"
      shift 2
      ;;
    --registry-dir)
      registry_dir="$2"
      shift 2
      ;;
    --artifacts-dir)
      artifacts_dir="$2"
      shift 2
      ;;
    *)
      echo "unknown arg: $1" >&2
      exit 2
      ;;
  esac
done

checks_registry="${registry_dir}/checks.json"
owners_registry="${registry_dir}/owners.json"
if [[ ! -f "${consumer_path}" || ! -f "${checks_registry}" || ! -f "${owners_registry}" ]]; then
  echo "missing required inputs: consumer=${consumer_path} checks=${checks_registry} owners=${owners_registry}" >&2
  exit 2
fi

mkdir -p "${artifacts_dir}/logs"
results_jsonl="${artifacts_dir}/results.jsonl"
: > "${results_jsonl}"

now_ms() { perl -MTime::HiRes=time -e 'print int(time()*1000)'; }
xml_escape() {
  printf '%s' "$1" | sed -e 's/&/\&amp;/g' -e 's/</\&lt;/g' -e 's/>/\&gt;/g' -e 's/"/\&quot;/g' -e "s/'/\&apos;/g"
}

lookup_check() {
  local cid="$1"
  jq -c --arg id "$cid" '.checks[] | select(.id == $id)' "${checks_registry}" | head -n 1
}

check_make_target_exists() {
  local target="$1"
  make -n "$target" >/dev/null 2>&1
}

run_start_ms="$(now_ms)"

consumer_checks_count="$(jq '.checks | length' "${consumer_path}")"
if [[ "${consumer_checks_count}" -eq 0 ]]; then
  echo "consumer has no checks: ${consumer_path}" >&2
  exit 2
fi

required_failures=0

while IFS= read -r entry; do
  cid="$(jq -r '.id' <<<"$entry")"
  required="$(jq -r '.required // false' <<<"$entry")"
  chk="$(lookup_check "$cid")"

  if [[ -z "$chk" ]]; then
    jq -nc --arg id "$cid" --argjson required "$required" --arg msg "check id not found in registry" \
      '{id:$id,status:"error",required:$required,duration_ms:0,message:$msg}' >> "${results_jsonl}"
    if [[ "$required" == "true" ]]; then required_failures=$((required_failures+1)); fi
    continue
  fi

  title="$(jq -r '.title' <<<"$chk")"
  owner_id="$(jq -r '.owner_id' <<<"$chk")"
  severity="$(jq -r '.severity' <<<"$chk")"
  runner_type="$(jq -r '.runner.type' <<<"$chk")"

  start_ms="$(now_ms)"
  status="passed"
  message="ok"
  command=""
  log_path=""

  if [[ "$runner_type" == "make_target" ]]; then
    target="$(jq -r '.runner.target' <<<"$chk")"
    command="make ${target}"
    log_path="${artifacts_dir}/logs/${cid}.log"
    if check_make_target_exists "$target"; then
      if make "$target" >"$log_path" 2>&1; then
        status="passed"
      else
        status="failed"
        message="target failed"
      fi
    else
      status="skipped"
      message="target not available in consumer repository"
    fi
  else
    status="error"
    message="unsupported runner type ${runner_type}"
  fi

  end_ms="$(now_ms)"
  duration_ms=$((end_ms-start_ms))

  if [[ "$status" == "failed" || "$status" == "error" ]]; then
    if [[ "$required" == "true" ]]; then
      required_failures=$((required_failures+1))
    fi
  fi

  jq -nc \
    --arg id "$cid" \
    --arg title "$title" \
    --arg owner_id "$owner_id" \
    --arg severity "$severity" \
    --arg status "$status" \
    --argjson required "$required" \
    --arg message "$message" \
    --arg command "$command" \
    --arg log_path "$log_path" \
    --argjson duration_ms "$duration_ms" \
    '{id:$id,title:$title,owner_id:$owner_id,severity:$severity,status:$status,required:$required,duration_ms:$duration_ms,message:$message,command:$command,log_path:$log_path}' >> "${results_jsonl}"
done < <(jq -c '.checks[]' "${consumer_path}")

checks_json="${artifacts_dir}/checks.json"
jq -s '.' "${results_jsonl}" > "${checks_json}"

passed="$(jq '[.[] | select(.status=="passed")] | length' "${checks_json}")"
failed="$(jq '[.[] | select(.status=="failed")] | length' "${checks_json}")"
skipped="$(jq '[.[] | select(.status=="skipped")] | length' "${checks_json}")"
errored="$(jq '[.[] | select(.status=="error")] | length' "${checks_json}")"
contracts="$(jq 'length' "${checks_json}")"
run_end_ms="$(now_ms)"
run_duration_ms=$((run_end_ms-run_start_ms))
exit_code=0
if [[ ${required_failures} -gt 0 ]]; then
  exit_code=1
fi

report_json="${artifacts_dir}/check-report.json"
consumer_id="$(jq -r '.consumer_id' "${consumer_path}")"
suite_id="$(jq -r '.suite' "${consumer_path}")"
repo_slug="${GITHUB_REPOSITORY:-$(basename "${repo_root}")}"
repo_sha="${GITHUB_SHA:-$(git -C "${repo_root}" rev-parse HEAD 2>/dev/null || echo unknown)}"

jq -n \
  --arg consumer_id "$consumer_id" \
  --arg suite "$suite_id" \
  --arg repository "$repo_slug" \
  --arg sha "$repo_sha" \
  --argjson contracts "$contracts" \
  --argjson passed "$passed" \
  --argjson failed "$failed" \
  --argjson skipped "$skipped" \
  --argjson errored "$errored" \
  --argjson duration_ms "$run_duration_ms" \
  --argjson exit_code "$exit_code" \
  --slurpfile checks "${checks_json}" \
  '{schema_version:1,consumer:{consumer_id:$consumer_id,suite:$suite,repository:$repository,sha:$sha},summary:{contracts:$contracts,passed:$passed,failed:$failed,skipped:$skipped,errored:$errored,duration_ms:$duration_ms,exit_code:$exit_code},checks:$checks[0]}' > "${report_json}"

report_md="${artifacts_dir}/check-report.md"
{
  echo "# Bijux Checks Report"
  echo
  echo "- consumer: ${consumer_id}"
  echo "- suite: ${suite_id}"
  echo "- repository: ${repo_slug}"
  echo "- sha: ${repo_sha}"
  echo "- summary: contracts=${contracts}, passed=${passed}, failed=${failed}, skipped=${skipped}, errored=${errored}, exit_code=${exit_code}"
  echo
  echo "| CheckID | Status | Required | Owner | Severity | Duration(ms) | Message |"
  echo "|---|---|---:|---|---|---:|---|"
  jq -r '.[] | "| \(.id) | \(.status) | \(.required) | \(.owner_id // "") | \(.severity // "") | \(.duration_ms) | \(.message // "") |"' "${checks_json}"
} > "${report_md}"

junit_xml="${artifacts_dir}/junit.xml"
{
  echo "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
  echo "<testsuite name=\"bijux-checks\" tests=\"${contracts}\" failures=\"$((failed+errored))\" skipped=\"${skipped}\">"
  while IFS= read -r line; do
    cid="$(jq -r '.id' <<<"$line")"
    status="$(jq -r '.status' <<<"$line")"
    req="$(jq -r '.required' <<<"$line")"
    secs="$(jq -r '.duration_ms/1000' <<<"$line")"
    msg="$(jq -r '.message // ""' <<<"$line")"
    echo "  <testcase classname=\"bijux-checks\" name=\"$(xml_escape "$cid")\" time=\"${secs}\">"
    if [[ "$status" == "skipped" ]]; then
      echo "    <skipped message=\"$(xml_escape "$msg")\"/>"
    elif [[ "$status" == "failed" || "$status" == "error" ]]; then
      echo "    <failure message=\"$(xml_escape "$msg")\">$(xml_escape "required=${req}")</failure>"
    fi
    echo "  </testcase>"
  done < <(jq -c '.[]' "${checks_json}")
  echo "</testsuite>"
} > "${junit_xml}"

echo "wrote ${report_json}"
echo "wrote ${report_md}"
echo "wrote ${junit_xml}"

exit "${exit_code}"
