#!/usr/bin/env python3

"""Generate the BeiDou B1I reference catalog used by signal validation tests.

The catalog is derived from the published B1I open-service phase assignments and
rendered by an independent Python implementation of the 11-stage Gold-code
generator.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
from dataclasses import dataclass
from pathlib import Path

CATALOG_SCHEMA = "beidou_b1i_reference_catalog.v1"
OUTPUT_RELATIVE_PATH = "crates/bijux-gnss-signal/tests/data/beidou_b1i_reference_catalog.toml"
CODE_LENGTH = 2046
WINDOW_LENGTH = 32
MIDDLE_START = 1007
BOUNDARY_START = 2030


@dataclass(frozen=True)
class BeidouB1iReference:
    prn: int
    g2_taps: tuple[int, int]


REFERENCES: tuple[BeidouB1iReference, ...] = (
    BeidouB1iReference(1, (1, 3)),
    BeidouB1iReference(2, (1, 4)),
    BeidouB1iReference(3, (1, 5)),
    BeidouB1iReference(4, (1, 6)),
    BeidouB1iReference(5, (1, 8)),
    BeidouB1iReference(6, (1, 9)),
    BeidouB1iReference(7, (1, 10)),
    BeidouB1iReference(8, (1, 11)),
    BeidouB1iReference(9, (2, 7)),
    BeidouB1iReference(10, (3, 4)),
    BeidouB1iReference(11, (3, 5)),
    BeidouB1iReference(12, (3, 6)),
    BeidouB1iReference(13, (3, 8)),
    BeidouB1iReference(14, (3, 9)),
    BeidouB1iReference(15, (3, 10)),
    BeidouB1iReference(16, (3, 11)),
    BeidouB1iReference(17, (4, 5)),
    BeidouB1iReference(18, (4, 6)),
    BeidouB1iReference(19, (4, 8)),
    BeidouB1iReference(20, (4, 9)),
    BeidouB1iReference(21, (4, 10)),
    BeidouB1iReference(22, (4, 11)),
    BeidouB1iReference(23, (5, 6)),
    BeidouB1iReference(24, (5, 8)),
    BeidouB1iReference(25, (5, 9)),
    BeidouB1iReference(26, (5, 10)),
    BeidouB1iReference(27, (5, 11)),
    BeidouB1iReference(28, (6, 8)),
    BeidouB1iReference(29, (6, 9)),
    BeidouB1iReference(30, (6, 10)),
    BeidouB1iReference(31, (6, 11)),
    BeidouB1iReference(32, (8, 9)),
    BeidouB1iReference(33, (8, 10)),
    BeidouB1iReference(34, (8, 11)),
    BeidouB1iReference(35, (9, 10)),
    BeidouB1iReference(36, (9, 11)),
    BeidouB1iReference(37, (10, 11)),
)


def main() -> int:
    args = parse_args()
    repo_root = resolve_repo_root(args.repo_root)
    output_path = repo_root / args.output
    catalog = render_catalog()

    if args.check:
        existing = output_path.read_text(encoding="utf-8")
        if existing != catalog:
            sys.stderr.write(
                f"reference catalog drift: regenerate {output_path.relative_to(repo_root)}\n"
            )
            return 1
        return 0

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(catalog, encoding="utf-8")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the BeiDou B1I reference catalog used by signal validation tests."
    )
    parser.add_argument(
        "--repo-root",
        default=Path(__file__).resolve().parents[4],
        type=Path,
        help="workspace repository root",
    )
    parser.add_argument(
        "--output",
        default=OUTPUT_RELATIVE_PATH,
        help="catalog output path relative to the repository root",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if the checked-in catalog differs from regenerated content",
    )
    return parser.parse_args()


def resolve_repo_root(repo_root: Path) -> Path:
    repo_root = repo_root.resolve()
    if not (repo_root / "Cargo.toml").is_file():
        raise SystemExit(f"repo root does not contain Cargo.toml: {repo_root}")
    return repo_root


def render_catalog() -> str:
    lines = [
        f'schema = "{CATALOG_SCHEMA}"',
        'reference_origin = "published BDS open-service B1I phase assignments with independent catalog generation"',
        f"code_length = {CODE_LENGTH}",
        f"window_length = {WINDOW_LENGTH}",
        f"middle_start = {MIDDLE_START}",
        f"boundary_start = {BOUNDARY_START}",
        "",
    ]

    for reference in REFERENCES:
        logical_bits = generate_b1i_logical_bits(reference.g2_taps)
        lines.extend(
            [
                "[[code]]",
                f"prn = {reference.prn}",
                f"g2_taps = [{reference.g2_taps[0]}, {reference.g2_taps[1]}]",
                f'bit_sha256 = "{sha256_hex(logical_bits)}"',
                f'bit_prefix = "{logical_bits[:WINDOW_LENGTH]}"',
                f'bit_middle = "{logical_bits[MIDDLE_START : MIDDLE_START + WINDOW_LENGTH]}"',
                f'bit_suffix = "{logical_bits[-WINDOW_LENGTH:]}"',
                f'bit_boundary = "{wrapped_window(logical_bits, BOUNDARY_START, WINDOW_LENGTH)}"',
                "",
            ]
        )

    return "\n".join(lines).rstrip() + "\n"


def generate_b1i_logical_bits(g2_taps: tuple[int, int]) -> str:
    g1 = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0]
    g2 = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0]
    tap1, tap2 = g2_taps[0] - 1, g2_taps[1] - 1
    bits: list[str] = []

    for _ in range(CODE_LENGTH):
        chip = g1[10] ^ g2[tap1] ^ g2[tap2]
        bits.append("1" if chip else "0")

        g1_feedback = g1[0] ^ g1[6] ^ g1[7] ^ g1[8] ^ g1[9] ^ g1[10]
        g2_feedback = g2[0] ^ g2[1] ^ g2[2] ^ g2[3] ^ g2[4] ^ g2[7] ^ g2[8] ^ g2[10]
        g1 = [g1_feedback & 1, *g1[:-1]]
        g2 = [g2_feedback & 1, *g2[:-1]]

    return "".join(bits)


def wrapped_window(bits: str, start: int, length: int) -> str:
    if start + length <= len(bits):
        return bits[start : start + length]
    tail = bits[start:]
    head = bits[: length - len(tail)]
    return f"{tail}{head}"


def sha256_hex(payload: str) -> str:
    return hashlib.sha256(payload.encode("ascii")).hexdigest()


if __name__ == "__main__":
    raise SystemExit(main())
