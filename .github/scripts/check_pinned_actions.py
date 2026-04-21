#!/usr/bin/env python3
from __future__ import annotations

import re
import sys
import json
from pathlib import Path

USE_PATTERN = re.compile(r"^\s*-\s*uses:\s*([^\s#]+)")
PINNED_PATTERN = re.compile(r"^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+@[0-9a-f]{40}$")


def manifest_managed_paths(manifest_path: Path) -> list[str]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    entries = manifest.get("workflow_inventory", {}).get("managed_workflows", [])
    paths: list[str] = []
    for entry in entries:
        source = entry.get("source")
        runtime = entry.get("consumer_runtime")
        if isinstance(source, str) and source and Path(source).exists():
            paths.append(source)
        if isinstance(runtime, str) and runtime and Path(runtime).exists():
            paths.append(runtime)
    return paths


def gather_workflow_files(raw_paths: list[str]) -> tuple[list[Path], list[str]]:
    files: list[Path] = []
    missing: list[str] = []
    seen: set[Path] = set()

    for raw_path in raw_paths:
        path = Path(raw_path)
        if not path.exists():
            missing.append(raw_path)
            continue
        if path.is_dir():
            recursive_matches = sorted(path.rglob("*.yml")) + sorted(path.rglob("*.yaml"))
            for item in recursive_matches:
                if not item.is_file():
                    continue
                resolved = item.resolve()
                if resolved in seen:
                    continue
                seen.add(resolved)
                files.append(item)
            continue

        resolved = path.resolve()
        if resolved in seen:
            continue
        seen.add(resolved)
        files.append(path)

    return files, missing


def main() -> int:
    raw_args = sys.argv[1:]
    extra_paths: list[str] = []
    direct_paths: list[str] = []
    i = 0
    while i < len(raw_args):
        arg = raw_args[i]
        if arg == "--manifest-managed":
            if i + 1 >= len(raw_args):
                print("error: --manifest-managed requires <manifest-path>", file=sys.stderr)
                return 2
            extra_paths.extend(manifest_managed_paths(Path(raw_args[i + 1])))
            i += 2
            continue
        direct_paths.append(arg)
        i += 1

    if not direct_paths and not extra_paths:
        print("usage: check_pinned_actions.py [--manifest-managed <manifest.json>] <workflow-path> [<workflow-path> ...]", file=sys.stderr)
        return 2

    workflow_files, missing_paths = gather_workflow_files(direct_paths + extra_paths)
    violations: list[str] = []
    for missing in missing_paths:
        violations.append(f"missing path: {missing}")

    for path in workflow_files:
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            match = USE_PATTERN.match(line)
            if not match:
                continue

            reference = match.group(1)
            if reference.startswith("./"):
                continue
            if reference.startswith("docker://"):
                continue
            if PINNED_PATTERN.match(reference):
                continue
            violations.append(f"{path}:{line_number}: unpinned action reference '{reference}'")

    if violations:
        print("Found unpinned action references:", file=sys.stderr)
        for violation in violations:
            print(f"  - {violation}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
