#!/usr/bin/env python3
"""Materialize governed repository artifact alias symlinks."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys


ROOT_ALIAS_LAYOUT = {
    ".venv": Path("artifacts/root/check-venv"),
    ".tox": Path("artifacts/root/tox"),
    ".hypothesis": Path("artifacts/root/hypothesis"),
    ".benchmarks": Path("artifacts/root/benchmarks"),
}

PACKAGE_ALIAS_LAYOUT = {
    "artifacts": Path("artifacts/{package}"),
    ".venv": Path("artifacts/{package}/venv"),
    ".hypothesis": Path("artifacts/{package}/hypothesis"),
    ".benchmarks": Path("artifacts/{package}/benchmarks"),
}


def _relative_target(*, link_path: Path, target_path: Path) -> str:
    return os.path.relpath(target_path, start=link_path.parent)


def _materialize_alias(*, link_path: Path, target_path: Path) -> None:
    link_path.parent.mkdir(parents=True, exist_ok=True)
    target_path.mkdir(parents=True, exist_ok=True)
    expected_target = _relative_target(link_path=link_path, target_path=target_path)

    if link_path.is_symlink():
        current_target = os.readlink(link_path)
        if current_target == expected_target:
            return
        link_path.unlink()
    elif link_path.exists():
        raise RuntimeError(
            f"refusing to replace non-symlink path '{link_path}' with alias to "
            f"'{expected_target}'"
        )

    link_path.symlink_to(expected_target)


def _materialize_root_aliases(*, repo_root: Path) -> None:
    for alias_name, target_rel in ROOT_ALIAS_LAYOUT.items():
        _materialize_alias(
            link_path=repo_root / alias_name,
            target_path=repo_root / target_rel,
        )


def _materialize_package_aliases(*, repo_root: Path, package_dir: Path) -> None:
    package_name = package_dir.name
    for alias_name, target_template in PACKAGE_ALIAS_LAYOUT.items():
        target_rel = Path(str(target_template).format(package=package_name))
        _materialize_alias(
            link_path=package_dir / alias_name,
            target_path=repo_root / target_rel,
        )


def _discover_package_dirs(*, packages_dir: Path) -> list[Path]:
    if not packages_dir.is_dir():
        return []
    return sorted(
        child
        for child in packages_dir.iterdir()
        if child.is_dir() and (child / "pyproject.toml").is_file()
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Materialize repository and package artifact alias symlinks."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    root_parser = subparsers.add_parser(
        "root",
        help="materialize root aliases and every package alias under packages/",
    )
    root_parser.add_argument("--repo-root", required=True, type=Path)
    root_parser.add_argument("--packages-dir", type=Path)

    package_parser = subparsers.add_parser(
        "package",
        help="materialize aliases for one package root",
    )
    package_parser.add_argument("--repo-root", required=True, type=Path)
    package_parser.add_argument("--package-dir", required=True, type=Path)

    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    repo_root = args.repo_root.resolve()

    if args.command == "root":
        packages_dir = (
            args.packages_dir.resolve()
            if args.packages_dir is not None
            else repo_root / "packages"
        )
        _materialize_root_aliases(repo_root=repo_root)
        for package_dir in _discover_package_dirs(packages_dir=packages_dir):
            _materialize_package_aliases(repo_root=repo_root, package_dir=package_dir)
        return 0

    if args.command == "package":
        _materialize_package_aliases(
            repo_root=repo_root,
            package_dir=args.package_dir.resolve(),
        )
        return 0

    raise AssertionError(f"unsupported command: {args.command}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
