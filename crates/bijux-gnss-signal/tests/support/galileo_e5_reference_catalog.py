#!/usr/bin/env python3

"""Generate the Galileo E5 reference catalog used by signal validation tests.

The catalog is derived from the checked-in Galileo ICD table transcription in
`galileo_e5_tables.rs` and rendered by an independent Python implementation of
the E5 primary and secondary code generation rules.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
from pathlib import Path

CATALOG_SCHEMA = "galileo_e5_reference_catalog.v1"
TABLES_RELATIVE_PATH = "crates/bijux-gnss-signal/src/codes/galileo_e5_tables.rs"
OUTPUT_RELATIVE_PATH = "crates/bijux-gnss-signal/tests/data/galileo_e5_reference_catalog.toml"
PRIMARY_CODE_LENGTH = 10_230
PRIMARY_WINDOW_LENGTH = 32
PRIMARY_MIDDLE_START = 5_099
PRIMARY_BOUNDARY_START = 10_214
REGISTER_MASK = 0x3FFF
E5A_REG1_TAPS = int("40503", 8)
E5A_REG2_TAPS = int("50661", 8)
E5B_I_REG1_TAPS = int("64021", 8)
E5B_I_REG2_TAPS = int("51445", 8)
E5B_Q_REG1_TAPS = int("64021", 8)
E5B_Q_REG2_TAPS = int("43143", 8)


def main() -> int:
    args = parse_args()
    repo_root = resolve_repo_root(args.repo_root)
    table_path = repo_root / TABLES_RELATIVE_PATH
    output_path = repo_root / args.output
    catalog = render_catalog(table_path)

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
        description="Generate the Galileo E5 reference catalog used by signal validation tests."
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


def render_catalog(table_path: Path) -> str:
    source = table_path.read_text(encoding="utf-8")
    e5a_i_starts = parse_string_array(source, "GALILEO_E5A_I_START_VALUES_OCTAL")
    e5a_i_first24 = parse_string_array(source, "GALILEO_E5A_I_INITIAL_SEQUENCE_HEX")
    e5a_q_starts = parse_string_array(source, "GALILEO_E5A_Q_START_VALUES_OCTAL")
    e5a_q_first24 = parse_string_array(source, "GALILEO_E5A_Q_INITIAL_SEQUENCE_HEX")
    e5b_i_starts = parse_string_array(source, "GALILEO_E5B_I_START_VALUES_OCTAL")
    e5b_i_first24 = parse_string_array(source, "GALILEO_E5B_I_INITIAL_SEQUENCE_HEX")
    e5b_q_starts = parse_string_array(source, "GALILEO_E5B_Q_START_VALUES_OCTAL")
    e5b_q_first24 = parse_string_array(source, "GALILEO_E5B_Q_INITIAL_SEQUENCE_HEX")
    e5a_i_secondary = parse_string_constant(source, "GALILEO_E5A_I_SECONDARY_HEX")
    e5a_q_secondary = parse_string_array(source, "GALILEO_E5A_Q_SECONDARY_HEX")
    e5b_i_secondary = parse_string_constant(source, "GALILEO_E5B_I_SECONDARY_HEX")
    e5b_q_secondary = parse_string_array(source, "GALILEO_E5B_Q_SECONDARY_HEX")

    lines = [
        f'schema = "{CATALOG_SCHEMA}"',
        'reference_origin = "checked-in Galileo ICD table transcription with independent catalog generation"',
        f'table_source = "{TABLES_RELATIVE_PATH}"',
        f"primary_code_length = {PRIMARY_CODE_LENGTH}",
        f"primary_window_length = {PRIMARY_WINDOW_LENGTH}",
        f"primary_middle_start = {PRIMARY_MIDDLE_START}",
        f"primary_boundary_start = {PRIMARY_BOUNDARY_START}",
        "",
    ]

    for component, starts, first24, reg1_taps, reg2_taps in (
        ("E5AI", e5a_i_starts, e5a_i_first24, E5A_REG1_TAPS, E5A_REG2_TAPS),
        ("E5AQ", e5a_q_starts, e5a_q_first24, E5A_REG1_TAPS, E5A_REG2_TAPS),
        ("E5BI", e5b_i_starts, e5b_i_first24, E5B_I_REG1_TAPS, E5B_I_REG2_TAPS),
        ("E5BQ", e5b_q_starts, e5b_q_first24, E5B_Q_REG1_TAPS, E5B_Q_REG2_TAPS),
    ):
        for prn, (start_octal, first24_hex) in enumerate(zip(starts, first24), start=1):
            logical_bits = generate_primary_bits(reg1_taps, reg2_taps, int(start_octal, 8))
            if first24_from_bits(logical_bits) != first24_hex:
                raise ValueError(f"{component} PRN {prn} first-24-chip mismatch")
            lines.extend(
                [
                    "[[primary_code]]",
                    f'component = "{component}"',
                    f"prn = {prn}",
                    f'register_2_start_octal = "{start_octal}"',
                    f'bit_sha256 = "{sha256_hex(logical_bits)}"',
                    f'bit_prefix = "{logical_bits[:PRIMARY_WINDOW_LENGTH]}"',
                    f'bit_middle = "{logical_bits[PRIMARY_MIDDLE_START:PRIMARY_MIDDLE_START + PRIMARY_WINDOW_LENGTH]}"',
                    f'bit_suffix = "{logical_bits[-PRIMARY_WINDOW_LENGTH:]}"',
                    f'bit_boundary = "{wrapped_window(logical_bits, PRIMARY_BOUNDARY_START, PRIMARY_WINDOW_LENGTH)}"',
                    "",
                ]
            )

    secondary_entries = [
        ("E5AI", None, decode_hex_bits(e5a_i_secondary, 20)),
        *[
            ("E5AQ", prn, decode_hex_bits(bits, 100))
            for prn, bits in enumerate(e5a_q_secondary, start=1)
        ],
        ("E5BI", None, decode_hex_bits(e5b_i_secondary, 4)),
        *[
            ("E5BQ", prn, decode_hex_bits(bits, 100))
            for prn, bits in enumerate(e5b_q_secondary, start=1)
        ],
    ]

    for component, prn, logical_bits in secondary_entries:
        window_length = secondary_window_length(len(logical_bits))
        middle_start = max(0, (len(logical_bits) // 2) - (window_length // 2))
        boundary_start = len(logical_bits) - max(1, window_length // 2)
        entry_lines = [
            "[[secondary_code]]",
            f'component = "{component}"',
        ]
        if prn is not None:
            entry_lines.append(f"prn = {prn}")
        entry_lines.extend(
            [
                f"bit_length = {len(logical_bits)}",
                f"window_length = {window_length}",
                f"middle_start = {middle_start}",
                f"boundary_start = {boundary_start}",
                f'bit_sha256 = "{sha256_hex(logical_bits)}"',
                f'bit_prefix = "{logical_bits[:window_length]}"',
                f'bit_middle = "{logical_bits[middle_start:middle_start + window_length]}"',
                f'bit_suffix = "{logical_bits[-window_length:]}"',
                f'bit_boundary = "{wrapped_window(logical_bits, boundary_start, window_length)}"',
                "",
            ]
        )
        lines.extend(entry_lines)

    return "\n".join(lines).rstrip() + "\n"


def parse_string_array(source: str, name: str) -> list[str]:
    pattern = re.compile(rf"pub const {name}: \[&str; \d+\] = \[(.*?)\];", re.S)
    match = pattern.search(source)
    if match is None:
        raise ValueError(f"missing array {name}")
    return re.findall(r'"([^"]+)"', match.group(1))


def parse_string_constant(source: str, name: str) -> str:
    pattern = re.compile(rf'pub const {name}: &str = "([^"]+)";')
    match = pattern.search(source)
    if match is None:
        raise ValueError(f"missing constant {name}")
    return match.group(1)


def generate_primary_bits(reg1_taps: int, reg2_taps: int, reg2_start: int) -> str:
    reg1 = 0x3FFF
    reg2 = reg2_start & REGISTER_MASK
    bits: list[str] = []
    reg1_tapmask = (reg1_taps >> 1) & REGISTER_MASK
    reg2_tapmask = (reg2_taps >> 1) & REGISTER_MASK

    for _ in range(PRIMARY_CODE_LENGTH):
        chip = ((reg1 >> 13) & 1) ^ ((reg2 >> 13) & 1)
        bits.append("1" if chip else "0")
        reg1_feedback = parity(reg1 & reg1_tapmask)
        reg2_feedback = parity(reg2 & reg2_tapmask)
        reg1 = ((reg1 << 1) | reg1_feedback) & REGISTER_MASK
        reg2 = ((reg2 << 1) | reg2_feedback) & REGISTER_MASK

    return "".join(bits)


def decode_hex_bits(hex_value: str, bit_length: int) -> str:
    bits = "".join(f"{int(symbol, 16):04b}" for symbol in hex_value)
    return bits[:bit_length]


def first24_from_bits(logical_bits: str) -> str:
    return "".join(f"{int(logical_bits[index:index + 4], 2):X}" for index in range(0, 24, 4))


def secondary_window_length(bit_length: int) -> int:
    if bit_length >= 16:
        return 16
    if bit_length >= 8:
        return 8
    return bit_length


def parity(value: int) -> int:
    return value.bit_count() & 1


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
