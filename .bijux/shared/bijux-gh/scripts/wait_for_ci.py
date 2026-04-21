#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
import time
import urllib.parse
import urllib.request
from datetime import datetime, timedelta, timezone
from typing import Any


def require_env(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise SystemExit(f"missing required environment variable: {name}")
    return value


def parse_github_time(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00")).astimezone(timezone.utc)


def github_get_json(url: str, token: str) -> dict[str, Any]:
    request = urllib.request.Request(
        url,
        headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {token}",
            "X-GitHub-Api-Version": "2022-11-28",
        },
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


def format_run(run: dict[str, Any]) -> str:
    run_id = run.get("id", "unknown")
    created_at = run.get("created_at", "unknown")
    status = run.get("status", "unknown")
    conclusion = run.get("conclusion") or "pending"
    html_url = run.get("html_url", "")
    return (
        f"run_id={run_id} created_at={created_at} "
        f"status={status} conclusion={conclusion} {html_url}"
    ).strip()


def latest_ci_run(
    runs: list[dict[str, Any]],
    started_at: datetime,
    lookback_seconds: int,
    target_ref_name: str,
) -> dict[str, Any] | None:
    window_start = started_at - timedelta(seconds=lookback_seconds)
    candidates = [
        run
        for run in runs
        if run.get("name") == "CI"
        and run.get("event") == "push"
        and parse_github_time(run["created_at"]) >= window_start
    ]
    if not candidates:
        return None

    if target_ref_name:
        ref_matched = [run for run in candidates if run.get("head_branch") == target_ref_name]
        if ref_matched:
            candidates = ref_matched

    candidates.sort(key=lambda run: parse_github_time(run["created_at"]), reverse=True)
    return candidates[0]


def run_is_current_enough(run: dict[str, Any], started_at: datetime) -> bool:
    created_at = parse_github_time(run["created_at"])
    updated_at = parse_github_time(run["updated_at"])
    if created_at >= started_at:
        return True
    if updated_at >= started_at:
        return True
    return run.get("status") != "completed"


def main() -> int:
    token = require_env("GITHUB_TOKEN")
    repository = require_env("GITHUB_REPOSITORY")
    target_sha = require_env("TARGET_SHA")
    started_at = parse_github_time(require_env("CI_WAIT_STARTED_AT"))
    target_ref_name = os.environ.get("TARGET_REF_NAME", "").strip()

    api_root = os.environ.get("GITHUB_API_URL", "https://api.github.com").rstrip("/")
    workflow_file = os.environ.get("GH_RELEASE_CI_WORKFLOW_FILE", "ci.yml")
    timeout_seconds = int(os.environ.get("GH_RELEASE_CI_WAIT_TIMEOUT_SECONDS", "1800"))
    poll_seconds = int(os.environ.get("GH_RELEASE_CI_POLL_INTERVAL_SECONDS", "15"))
    lookback_seconds = int(os.environ.get("GH_RELEASE_CI_LOOKBACK_SECONDS", "120"))
    grace_seconds = int(os.environ.get("GH_RELEASE_CI_APPEARANCE_GRACE_SECONDS", "20"))

    url = (
        f"{api_root}/repos/{repository}/actions/workflows/"
        f"{urllib.parse.quote(workflow_file, safe='')}/runs"
        f"?event=push&head_sha={urllib.parse.quote(target_sha, safe='')}&per_page=20"
    )

    print(
        "Waiting for CI workflow to finish before release publish:",
        f"workflow={workflow_file}",
        f"sha={target_sha}",
        f"timeout_seconds={timeout_seconds}",
        sep=" ",
    )
    if grace_seconds > 0:
        print(f"Allowing {grace_seconds}s for the CI run to appear in the Actions API.")
        time.sleep(grace_seconds)

    deadline = time.monotonic() + timeout_seconds
    last_summary = ""
    while time.monotonic() < deadline:
        payload = github_get_json(url, token)
        runs = payload.get("workflow_runs", [])
        run = latest_ci_run(runs, started_at, lookback_seconds, target_ref_name)
        if run is None:
            print("CI run not visible yet; polling again soon.")
            time.sleep(poll_seconds)
            continue

        summary = format_run(run)
        if summary != last_summary:
            print(f"Observed CI run: {summary}")
            last_summary = summary

        if not run_is_current_enough(run, started_at):
            print("Latest matching CI run completed before this release started; waiting for the tag run.")
            time.sleep(poll_seconds)
            continue

        status = run.get("status")
        conclusion = run.get("conclusion")
        if status != "completed":
            time.sleep(poll_seconds)
            continue
        if conclusion == "success":
            print("CI gate passed; release workflow may continue.")
            return 0
        print(f"CI gate failed with conclusion={conclusion}; stopping release publish.")
        return 1

    print(f"Timed out waiting for CI workflow {workflow_file} on {target_sha}.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
