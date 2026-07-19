#!/usr/bin/env python3

"""Generate the GPS L2C CL reference catalog used by signal validation tests.

The catalog is derived from the checked-in IS-GPS-200L specification tables and
rendered by an independent Python implementation of the L2C CL register
generator.
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
        "PyMuPDF is required to regenerate the GPS L2C CL reference catalog; "
        "run with artifacts/l2c_spec/venv/bin/python or install pymupdf"
    ) from exc

CATALOG_SCHEMA = "gps_l2c_cl_reference_catalog.v1"
SPEC_RELATIVE_PATH = "artifacts/l2c_spec/IS-GPS-200L.pdf"
OUTPUT_RELATIVE_PATH = (
    "crates/bijux-gnss-signal/tests/data/gps_l2c_cl_reference_catalog.toml"
)
SPEC_TABLE_PAGES = (24, 25, 26, 73, 74)
L2C_CL_CODE_CHIPS = 767_250
RANGE_LENGTH = 64
RANGE_OFFSETS = (0, 10_230, 255_731, 511_463, 767_186, 767_218)
FEEDBACK_DESTINATIONS = {3, 6, 8, 11, 14, 16, 18, 21, 22, 23, 24}
FEEDBACK_MASK = sum(1 << (26 - destination) for destination in FEEDBACK_DESTINATIONS)


@dataclass(frozen=True)
class GeneratedReference:
    bit_sha256: str
    range_bits: list[str]
    published_end_state: str


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
        description="Generate the GPS L2C CL reference catalog used by signal validation tests."
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
    assignments: list[tuple[int, str, str]] = []

    with fitz.open(spec_path) as pdf:
        for page_number in SPEC_TABLE_PAGES:
            table = pdf[page_number - 1].find_tables().tables[0].extract()
            data_row = table[2]
            if page_number in {24, 25, 26}:
                prns = parse_column_as_ints(data_row[1])
                initial_states = parse_column(data_row[3])
                end_states = parse_column(data_row[5])
            else:
                prns = parse_column_as_ints(data_row[0])
                initial_states = parse_column(data_row[2])
                end_states = parse_column(data_row[4])

            if len(prns) != len(initial_states) or len(prns) != len(end_states):
                raise ValueError(f"table length mismatch on page {page_number}")

            assignments.extend(zip(prns, initial_states, end_states))

    if len(assignments) != 115:
        raise ValueError(f"expected 115 published L2C CL assignments, found {len(assignments)}")

    expected_rows = {
        1: ("624145772", "267724236"),
        38: ("101232630", "463624741"),
        159: ("605253024", "044547544"),
        210: ("513322453", "113765506"),
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
        'reference_origin = "published IS-GPS-200L L2C CL assignments with independent catalog generation"',
        f'spec_source = "{SPEC_RELATIVE_PATH}"',
        'published_tables = "Table 3-IIa, Table 3-IIb, Table 6-II"',
        f"chip_length = {L2C_CL_CODE_CHIPS}",
        f"range_length = {RANGE_LENGTH}",
        "range_offsets = [" + ", ".join(str(offset) for offset in RANGE_OFFSETS) + "]",
        f"published_prn_count = {len(assignments)}",
        "",
    ]

    for prn, initial_state_octal, end_state_octal in assignments:
        reference = generate_reference(initial_state_octal)
        if reference.published_end_state != end_state_octal:
            raise ValueError(
                f"published end-state mismatch for PRN {prn}: "
                f"{reference.published_end_state} != {end_state_octal}"
            )
        encoded_range_bits = ", ".join(f'"{bits}"' for bits in reference.range_bits)

        lines.extend(
            [
                "[[code]]",
                f"prn = {prn}",
                f'initial_state_octal = "{initial_state_octal}"',
                f'end_state_octal = "{end_state_octal}"',
                f'bit_sha256 = "{reference.bit_sha256}"',
                f"range_bits = [{encoded_range_bits}]",
                "",
            ]
        )

    return "\n".join(lines).rstrip() + "\n"


def generate_reference(initial_state_octal: str) -> GeneratedReference:
    state = register_state_from_octal(initial_state_octal)
    logical_bits = bytearray(L2C_CL_CODE_CHIPS)
    published_end_state = ""

    for chip_index in range(L2C_CL_CODE_CHIPS):
        logical_bits[chip_index] = 49 if state & 1 else 48
        if chip_index == L2C_CL_CODE_CHIPS - 1:
            published_end_state = register_state_to_octal(state)
        state = advance_register_state(state)

    return GeneratedReference(
        bit_sha256=sha256_hex(logical_bits),
        range_bits=[
            extract_range_bits(logical_bits, start_chip, RANGE_LENGTH)
            for start_chip in RANGE_OFFSETS
        ],
        published_end_state=published_end_state,
    )


def extract_range_bits(logical_bits: bytearray, start_chip: int, length: int) -> str:
    if start_chip + length <= len(logical_bits):
        return logical_bits[start_chip : start_chip + length].decode("ascii")
    tail = logical_bits[start_chip:]
    head = logical_bits[: length - len(tail)]
    return (tail + head).decode("ascii")


def register_state_from_octal(state_octal: str) -> int:
    return int(state_octal, 8)


def register_state_to_octal(state: int) -> str:
    return f"{state:09o}"


def advance_register_state(state: int) -> int:
    output_bit = state & 1
    next_state = (state >> 1) | (output_bit << 26)
    if output_bit:
        next_state ^= FEEDBACK_MASK
    return next_state


def sha256_hex(payload: bytes | bytearray) -> str:
    return hashlib.sha256(payload).hexdigest()


if __name__ == "__main__":
    raise SystemExit(main())
