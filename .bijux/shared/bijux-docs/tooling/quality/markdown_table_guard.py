#!/usr/bin/env python3
"""Fail on malformed Markdown pipe tables in the docs tree."""

from __future__ import annotations

import argparse
import pathlib
import sys


def table_columns(line: str) -> int:
    stripped = line.strip()
    if not stripped.startswith("|") or not stripped.endswith("|"):
      return 0

    cells = [cell.strip() for cell in stripped.strip("|").split("|")]
    return len(cells)


def is_separator_row(line: str) -> bool:
    stripped = line.strip()
    if not stripped.startswith("|") or not stripped.endswith("|"):
        return False

    cells = [cell.strip() for cell in stripped.strip("|").split("|")]
    if not cells:
        return False

    allowed = set("-: ")
    return all(cell and set(cell) <= allowed and "-" in cell for cell in cells)


def scan_file(path: pathlib.Path) -> list[str]:
    issues: list[str] = []
    lines = path.read_text(encoding="utf-8").splitlines()

    for index in range(len(lines) - 1):
        header = lines[index]
        separator = lines[index + 1]

        if table_columns(header) == 0 or not is_separator_row(separator):
            continue

        header_columns = table_columns(header)
        separator_columns = table_columns(separator)
        if header_columns != separator_columns:
            issues.append(
                f"{path}:{index + 1}: header has {header_columns} columns but separator has {separator_columns}"
            )

        next_index = index + 2
        if next_index < len(lines):
            data_row = lines[next_index]
            data_columns = table_columns(data_row)
            if data_columns and data_columns != header_columns:
                issues.append(
                    f"{path}:{next_index + 1}: table row has {data_columns} columns but header has {header_columns}"
                )

    return issues


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("docs_dir", type=pathlib.Path)
    args = parser.parse_args()

    issues: list[str] = []
    for path in sorted(args.docs_dir.rglob("*.md")):
        issues.extend(scan_file(path))

    if issues:
        for issue in issues:
            print(issue, file=sys.stderr)
        return 1

    print(f"Markdown table guard passed for {args.docs_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
