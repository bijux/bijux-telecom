#!/usr/bin/env python3

"""Generate the GPS L2C CM reference catalog used by signal validation tests.

The catalog is derived from the checked-in IS-GPS-200L specification tables and
rendered by an independent Python implementation of the L2C CM register
generator.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
from pathlib import Path

try:
    import fitz
except ImportError as exc:  # pragma: no cover - support script error path
    raise SystemExit(
        "PyMuPDF is required to regenerate the GPS L2C CM reference catalog; "
        "run with artifacts/l2c_spec/venv/bin/python or install pymupdf"
    ) from exc

CATALOG_SCHEMA = "gps_l2c_cm_reference_catalog.v1"
SPEC_RELATIVE_PATH = "artifacts/l2c_spec/IS-GPS-200L.pdf"
OUTPUT_RELATIVE_PATH = (
    "crates/bijux-gnss-signal/tests/data/gps_l2c_cm_reference_catalog.toml"
)
SPEC_TABLE_PAGES = (24, 25, 26, 73, 74)
L2C_CM_CODE_CHIPS = 10_230
PREFIX_BITS = 64
SUFFIX_BITS = 64
FEEDBACK_DESTINATIONS = {3, 6, 8, 11, 14, 16, 18, 21, 22, 23, 24}


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
                "reference catalog drift: regenerate "
                f"{output_path.relative_to(repo_root)}\n"
            )
            return 1
        return 0

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(catalog, encoding="utf-8")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the GPS L2C CM reference catalog used by signal validation tests."
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


def extract_assignments(spec_path: Path) -> list[tuple[int, str, str]]:
    pdf = fitz.open(spec_path)
    assignments: list[tuple[int, str, str]] = []

    for page_number in SPEC_TABLE_PAGES:
        table = pdf[page_number - 1].find_tables().tables[0].extract()
        data_row = table[2]
        if page_number in {24, 25, 26}:
            prns = parse_column_as_ints(data_row[1])
            initial_states = parse_column(data_row[2])
            end_states = parse_column(data_row[4])
        else:
            prns = parse_column_as_ints(data_row[0])
            initial_states = parse_column(data_row[1])
            end_states = parse_column(data_row[3])

        if len(prns) != len(initial_states) or len(prns) != len(end_states):
            raise ValueError(f"table length mismatch on page {page_number}")

        assignments.extend(zip(prns, initial_states, end_states))

    if len(assignments) != 115:
        raise ValueError(f"expected 115 published L2C CM assignments, found {len(assignments)}")

    expected_rows = {
        1: ("742417664", "552566002"),
        63: ("111460621", "602375063"),
        159: ("604055104", "425373114"),
        210: ("140633660", "465052527"),
    }
    for prn, (initial_state, end_state) in expected_rows.items():
        actual_initial, actual_end = next(
            (row_initial, row_end)
            for row_prn, row_initial, row_end in assignments
            if row_prn == prn
        )
        if actual_initial != initial_state or actual_end != end_state:
            raise ValueError(f"unexpected published assignment for PRN {prn}")

    return assignments


def parse_column(cell: str) -> list[str]:
    return [line.strip() for line in cell.splitlines() if line.strip()]


def parse_column_as_ints(cell: str) -> list[int]:
    return [int(value) for value in parse_column(cell)]


def render_catalog(assignments: list[tuple[int, str, str]]) -> str:
    lines = [
        f'schema = "{CATALOG_SCHEMA}"',
        'reference_origin = "published IS-GPS-200L L2 CM assignments with independent catalog generation"',
        f'spec_source = "{SPEC_RELATIVE_PATH}"',
        'published_tables = "Table 3-IIa, Table 3-IIb, Table 6-II"',
        f"chip_length = {L2C_CM_CODE_CHIPS}",
        f"prefix_length = {PREFIX_BITS}",
        f"suffix_length = {SUFFIX_BITS}",
        f"published_prn_count = {len(assignments)}",
        "",
    ]

    for prn, initial_state_octal, end_state_octal in assignments:
        logical_bits = generate_l2c_cm_bits(initial_state_octal)
        derived_end_state = derive_published_end_state(initial_state_octal)
        if derived_end_state != end_state_octal:
            raise ValueError(
                f"published end-state mismatch for PRN {prn}: "
                f"{derived_end_state} != {end_state_octal}"
            )

        lines.extend(
            [
                "[[code]]",
                f"prn = {prn}",
                f'initial_state_octal = "{initial_state_octal}"',
                f'end_state_octal = "{end_state_octal}"',
                f'bit_sha256 = "{sha256_hex(logical_bits)}"',
                f'bit_prefix = "{logical_bits[:PREFIX_BITS]}"',
                f'bit_suffix = "{logical_bits[-SUFFIX_BITS:]}"',
                "",
            ]
        )

    return "\n".join(lines).rstrip() + "\n"


def generate_l2c_cm_bits(initial_state_octal: str) -> str:
    state = register_state_from_octal(initial_state_octal)
    bits: list[str] = []
    for _ in range(L2C_CM_CODE_CHIPS):
        bits.append("1" if state[26] else "0")
        state = advance_register_state(state)
    return "".join(bits)


def derive_published_end_state(initial_state_octal: str) -> str:
    state = register_state_from_octal(initial_state_octal)
    for _ in range(L2C_CM_CODE_CHIPS - 1):
        state = advance_register_state(state)
    return register_state_to_octal(state)


def register_state_from_octal(state_octal: str) -> list[int]:
    return [int(bit) for bit in f"{int(state_octal, 8):027b}"]


def register_state_to_octal(state: list[int]) -> str:
    return f"{int(''.join(str(bit) for bit in state), 2):09o}"


def advance_register_state(state: list[int]) -> list[int]:
    output_bit = state[26]
    next_state = [0] * len(state)
    next_state[0] = output_bit

    for destination_index in range(1, len(state)):
        next_value = state[destination_index - 1]
        if destination_index in FEEDBACK_DESTINATIONS:
            next_value ^= output_bit
        next_state[destination_index] = next_value

    return next_state


def sha256_hex(payload: str) -> str:
    return hashlib.sha256(payload.encode("ascii")).hexdigest()


if __name__ == "__main__":
    raise SystemExit(main())
