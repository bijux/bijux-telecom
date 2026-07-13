#!/usr/bin/env python3

"""Generate the GPS L5 reference catalog used by signal validation tests.

The catalog is derived from the published IS-GPS-705J code-phase assignments and
rendered by an independent Python implementation of the XA/XB code generators.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    import fitz
except ImportError as exc:  # pragma: no cover - support script error path
    raise SystemExit(
        "PyMuPDF is required to regenerate the GPS L5 reference catalog; "
        "run with artifacts/l2c_spec/venv/bin/python or install pymupdf"
    ) from exc

CATALOG_SCHEMA = "gps_l5_reference_catalog.v1"
SPEC_RELATIVE_PATH = "artifacts/IS-GPS-705J.pdf"
OUTPUT_RELATIVE_PATH = "crates/bijux-gnss-signal/tests/data/gps_l5_reference_catalog.toml"
PRIMARY_CODE_LENGTH = 10_230
WINDOW_LENGTH = 32
MIDDLE_START = 5_099
BOUNDARY_START = 10_214
XA_CHIPS = 8_190
XB_CHIPS = 8_191
REGISTER_STAGES = 13
PRIMARY_PAGES = (19, 20, 21)
L5I_NH_BITS = "0000110101"
L5Q_NH_BITS = "00000100110101001110"


@dataclass(frozen=True)
class L5Assignment:
    channel: str
    prn: int
    xb_advance_chips: int
    xb_initial_state_bits: str


def main() -> int:
    args = parse_args()
    repo_root = resolve_repo_root(args.repo_root)
    spec_path = repo_root / SPEC_RELATIVE_PATH
    output_path = repo_root / args.output
    assignments = extract_assignments(spec_path)
    catalog = render_catalog(assignments)

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
        description="Generate the GPS L5 reference catalog used by signal validation tests."
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


def extract_assignments(spec_path: Path) -> list[L5Assignment]:
    pdf = fitz.open(spec_path)
    assignments: list[L5Assignment] = []

    for page_number in PRIMARY_PAGES:
        table = pdf[page_number - 1].find_tables().tables[0].extract()
        data_row_index = 3 if page_number == 20 else 2
        prns = parse_column_as_ints(table[data_row_index][0])
        i_advance = parse_column_as_ints(table[data_row_index][1])
        q_advance = parse_column_as_ints(table[data_row_index][2])
        i_state = parse_column(table[data_row_index][3])
        q_state = parse_column(table[data_row_index][4])

        if not (len(prns) == len(i_advance) == len(q_advance) == len(i_state) == len(q_state)):
            raise ValueError(f"table length mismatch on page {page_number}")

        for prn, advance, state in zip(prns, i_advance, i_state):
            assignments.append(L5Assignment("I", prn, advance, state))
        for prn, advance, state in zip(prns, q_advance, q_state):
            assignments.append(L5Assignment("Q", prn, advance, state))

    if len(assignments) != 126:
        raise ValueError(f"expected 126 published GPS L5 assignments, found {len(assignments)}")

    return assignments


def parse_column(cell: str) -> list[str]:
    return [line.strip() for line in cell.splitlines() if line.strip()]


def parse_column_as_ints(cell: str) -> list[int]:
    return [int(value) for value in parse_column(cell)]


def render_catalog(assignments: list[L5Assignment]) -> str:
    lines = [
        f'schema = "{CATALOG_SCHEMA}"',
        'reference_origin = "published IS-GPS-705J L5 code-phase assignments with independent catalog generation"',
        f'spec_source = "{SPEC_RELATIVE_PATH}"',
        'published_tables = "Table 3-Ia and Table 3-Ib"',
        f"code_length = {PRIMARY_CODE_LENGTH}",
        f"window_length = {WINDOW_LENGTH}",
        f"middle_start = {MIDDLE_START}",
        f"boundary_start = {BOUNDARY_START}",
        f'l5i_nh_bits = "{L5I_NH_BITS}"',
        f'l5i_nh_sha256 = "{sha256_hex(L5I_NH_BITS)}"',
        f'l5q_nh_bits = "{L5Q_NH_BITS}"',
        f'l5q_nh_sha256 = "{sha256_hex(L5Q_NH_BITS)}"',
        "",
    ]

    for assignment in assignments:
        logical_bits = generate_primary_bits(assignment.xb_advance_chips)
        lines.extend(
            [
                "[[primary_code]]",
                f'channel = "{assignment.channel}"',
                f"prn = {assignment.prn}",
                f"xb_advance_chips = {assignment.xb_advance_chips}",
                f'xb_initial_state_bits = "{assignment.xb_initial_state_bits}"',
                f'bit_sha256 = "{sha256_hex(logical_bits)}"',
                f'bit_prefix = "{logical_bits[:WINDOW_LENGTH]}"',
                f'bit_middle = "{logical_bits[MIDDLE_START : MIDDLE_START + WINDOW_LENGTH]}"',
                f'bit_suffix = "{logical_bits[-WINDOW_LENGTH:]}"',
                f'bit_boundary = "{wrapped_window(logical_bits, BOUNDARY_START, WINDOW_LENGTH)}"',
                "",
            ]
        )

    return "\n".join(lines).rstrip() + "\n"


def generate_primary_bits(xb_advance_chips: int) -> str:
    xa = generate_xa_sequence()
    xb = generate_xb_sequence()
    bits: list[str] = []
    for chip_index in range(PRIMARY_CODE_LENGTH):
        xa_chip = xa[chip_index % XA_CHIPS]
        xb_chip = xb[(chip_index + xb_advance_chips) % XB_CHIPS]
        bits.append("1" if xa_chip ^ xb_chip else "0")
    return "".join(bits)


def generate_xa_sequence() -> list[int]:
    sequence: list[int] = []
    register = [1] * REGISTER_STAGES
    for _ in range(XA_CHIPS):
        sequence.append(register[-1])
        feedback = register[8] ^ register[9] ^ register[11] ^ register[12]
        register = [feedback, *register[:-1]]
    return sequence


def generate_xb_sequence() -> list[int]:
    sequence: list[int] = []
    register = [1] * REGISTER_STAGES
    for _ in range(XB_CHIPS):
        sequence.append(register[-1])
        feedback = (
            register[0]
            ^ register[2]
            ^ register[3]
            ^ register[5]
            ^ register[6]
            ^ register[7]
            ^ register[11]
            ^ register[12]
        )
        register = [feedback, *register[:-1]]
    return sequence


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
