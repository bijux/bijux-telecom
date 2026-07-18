#!/usr/bin/env python3
"""Validate the bijux-telecom Rust publication boundary."""

from __future__ import annotations

import argparse
import sys
import tomllib
from pathlib import Path
from typing import Any


def load_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as stream:
        return tomllib.load(stream)


def inherited(package: dict[str, Any], workspace: dict[str, Any], field: str) -> Any:
    value = package.get(field)
    if isinstance(value, dict) and value.get("workspace") is True:
        return workspace.get(field)
    return value


def validate(repo_root: Path, spec_path: Path) -> list[str]:
    errors: list[str] = []
    workspace_manifest = load_toml(repo_root / "Cargo.toml")
    workspace = workspace_manifest["workspace"]
    workspace_package = workspace["package"]
    spec = load_toml(spec_path)

    publish = spec["publish"]
    allowed = publish["allow"]
    denied = publish["deny"]
    if len(set(allowed)) != len(allowed):
        errors.append("publish.allow contains duplicate package names")
    if len(set(denied)) != len(denied):
        errors.append("publish.deny contains duplicate package names")
    if set(allowed) & set(denied):
        errors.append("publish.allow and publish.deny overlap")

    manifests: dict[str, tuple[Path, dict[str, Any]]] = {}
    for member in workspace["members"]:
        manifest_path = repo_root / member / "Cargo.toml"
        manifest = load_toml(manifest_path)
        manifests[manifest["package"]["name"]] = (manifest_path, manifest["package"])

    configured = set(allowed) | set(denied)
    if configured != set(manifests):
        missing = sorted(set(manifests) - configured)
        unknown = sorted(configured - set(manifests))
        if missing:
            errors.append(f"workspace packages missing from release spec: {', '.join(missing)}")
        if unknown:
            errors.append(f"release spec names unknown packages: {', '.join(unknown)}")

    required = spec["metadata_requirements"]["required_package_fields"]
    root_license = (repo_root / "LICENSE").read_bytes()
    order = {name: index for index, name in enumerate(allowed)}
    for name in allowed:
        if name not in manifests:
            continue
        manifest_path, package = manifests[name]
        for field in required:
            value = inherited(package, workspace_package, field)
            if value is None or value == "" or value == []:
                errors.append(f"{name} is missing package metadata: {field}")
        if inherited(package, workspace_package, "version") != workspace_package["version"]:
            errors.append(f"{name} does not inherit the workspace release version")
        license_path = manifest_path.parent / "LICENSE"
        if not license_path.is_file():
            errors.append(f"{name} does not package a LICENSE file")
        elif license_path.read_bytes() != root_license:
            errors.append(f"{name} LICENSE differs from the repository license")

        package_manifest = load_toml(manifest_path)
        dependencies = package_manifest.get("dependencies", {})
        for dependency in dependencies:
            if dependency in order and order[dependency] >= order[name]:
                errors.append(
                    f"{name} appears before its published dependency {dependency}"
                )
        dev_dependencies = package_manifest.get("dev-dependencies", {})
        for dependency, requirement in dev_dependencies.items():
            if dependency not in denied or not isinstance(requirement, dict):
                continue
            if "version" in requirement or requirement.get("workspace") is True:
                errors.append(
                    f"{name} must keep repository-only dev dependency "
                    f"{dependency} path-only"
                )

    for name in denied:
        if name not in manifests:
            continue
        if manifests[name][1].get("publish") is not False:
            errors.append(f"{name} must declare publish = false")

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--print-publish-order", action="store_true")
    args = parser.parse_args()

    repo_root = args.repo_root.resolve()
    spec_path = args.spec
    if not spec_path.is_absolute():
        spec_path = repo_root / spec_path

    errors = validate(repo_root, spec_path)
    if errors:
        for error in errors:
            print(f"release contract error: {error}", file=sys.stderr)
        return 1

    allowed = load_toml(spec_path)["publish"]["allow"]
    if args.print_publish_order:
        print(" ".join(allowed))
    else:
        print(
            f"release contract valid: {len(allowed)} published crates in dependency order"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
