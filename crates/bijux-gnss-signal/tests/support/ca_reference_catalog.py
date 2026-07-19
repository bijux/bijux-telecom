#!/usr/bin/env python3

"""Generate the GPS C/A reference catalog used by signal validation tests.

The catalog is derived from the published GPS SPS phase-assignment table and
rendered by an independent Python implementation of the G1/G2 generators.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
from dataclasses import dataclass
from pathlib import Path

CATALOG_SCHEMA = "ca_reference_catalog.v1"
OUTPUT_RELATIVE_PATH = "crates/bijux-gnss-signal/tests/data/ca_reference_catalog.toml"
CODE_LENGTH = 1023
WINDOW_LENGTH = 32
MIDDLE_START = 495
BOUNDARY_START = 1007


@dataclass(frozen=True)
class CaReference:
    prn: int
    g2_taps: tuple[int, int]
    g2_delay_chips: int
    first_ten_chips_octal: int


EXPECTED_CA_CODE_REFERENCES: tuple[CaReference, ...] = (
    CaReference(1, (2, 6), 5, 1440),
    CaReference(2, (3, 7), 6, 1620),
    CaReference(3, (4, 8), 7, 1710),
    CaReference(4, (5, 9), 8, 1744),
    CaReference(5, (1, 9), 17, 1133),
    CaReference(6, (2, 10), 18, 1455),
    CaReference(7, (1, 8), 139, 1131),
    CaReference(8, (2, 9), 140, 1454),
    CaReference(9, (3, 10), 141, 1626),
    CaReference(10, (2, 3), 251, 1504),
    CaReference(11, (3, 4), 252, 1642),
    CaReference(12, (5, 6), 254, 1750),
    CaReference(13, (6, 7), 255, 1764),
    CaReference(14, (7, 8), 256, 1772),
    CaReference(15, (8, 9), 257, 1775),
    CaReference(16, (9, 10), 258, 1776),
    CaReference(17, (1, 4), 469, 1156),
    CaReference(18, (2, 5), 470, 1467),
    CaReference(19, (3, 6), 471, 1633),
    CaReference(20, (4, 7), 472, 1715),
    CaReference(21, (5, 8), 473, 1746),
    CaReference(22, (6, 9), 474, 1763),
    CaReference(23, (1, 3), 509, 1063),
    CaReference(24, (4, 6), 512, 1706),
    CaReference(25, (5, 7), 513, 1743),
    CaReference(26, (6, 8), 514, 1761),
    CaReference(27, (7, 9), 515, 1770),
    CaReference(28, (8, 10), 516, 1774),
    CaReference(29, (1, 6), 859, 1127),
    CaReference(30, (2, 7), 860, 1453),
    CaReference(31, (3, 8), 861, 1625),
    CaReference(32, (4, 9), 862, 1712),
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
        description="Generate the GPS C/A reference catalog used by signal validation tests."
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
        'reference_origin = "published GPS SPS signal specification assignments with independent catalog generation"',
        f"code_length = {CODE_LENGTH}",
        f"window_length = {WINDOW_LENGTH}",
        f"middle_start = {MIDDLE_START}",
        f"boundary_start = {BOUNDARY_START}",
        "",
    ]

    for reference in EXPECTED_CA_CODE_REFERENCES:
        logical_bits = generate_ca_logical_bits(reference.g2_taps)
        expected_prefix = logical_bits[:WINDOW_LENGTH]
        expected_middle = logical_bits[MIDDLE_START : MIDDLE_START + WINDOW_LENGTH]
        expected_suffix = logical_bits[-WINDOW_LENGTH:]
        expected_boundary = wrapped_window(logical_bits, BOUNDARY_START, WINDOW_LENGTH)

        lines.extend(
            [
                "[[code]]",
                f"prn = {reference.prn}",
                f"g2_taps = [{reference.g2_taps[0]}, {reference.g2_taps[1]}]",
                f"g2_delay_chips = {reference.g2_delay_chips}",
                f"first_ten_chips_octal = {reference.first_ten_chips_octal}",
                f'bit_sha256 = "{sha256_hex(logical_bits)}"',
                f'bit_prefix = "{expected_prefix}"',
                f'bit_middle = "{expected_middle}"',
                f'bit_suffix = "{expected_suffix}"',
                f'bit_boundary = "{expected_boundary}"',
                "",
            ]
        )

    return "\n".join(lines).rstrip() + "\n"


def generate_ca_logical_bits(g2_taps: tuple[int, int]) -> str:
    g1 = [1] * 10
    g2 = [1] * 10
    tap1, tap2 = g2_taps[0] - 1, g2_taps[1] - 1
    bits: list[str] = []

    for _ in range(CODE_LENGTH):
        g2_tap = g2[tap1] ^ g2[tap2]
        bits.append("1" if (g1[9] ^ g2_tap) else "0")

        g1_feedback = g1[2] ^ g1[9]
        g2_feedback = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]
        g1 = [g1_feedback, *g1[:-1]]
        g2 = [g2_feedback, *g2[:-1]]

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
