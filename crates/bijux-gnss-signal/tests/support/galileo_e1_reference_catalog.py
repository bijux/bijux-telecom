#!/usr/bin/env python3

"""Generate the Galileo E1 reference catalog used by signal validation tests.

The catalog is derived from the independently maintained
`harshadms-galileo-sdr-sim` primary-code tables and cross-checked against the
separate `mx4-gnss-rcv` transcription before any output is emitted.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
from pathlib import Path

SECONDARY_CODE_MARKER = "GALILEO_E1_SECONDARY_CODE"
SECONDARY_CODE_RUST_PATTERN = re.compile(r'E1C_SECONDARY_CODE: &str = "([01]{25})"')
PRIMARY_CODE_LENGTH_HEX = 1023
PRIMARY_CODE_LENGTH_BITS = 4092


def main() -> int:
    args = parse_args()
    repo_root = resolve_repo_root(args.repo_root)

    harshadms_header = repo_root / "artifacts/external/harshadms-galileo-sdr-sim/include/constants.h"
    mx4_codes = repo_root / "artifacts/external/mx4-gnss-rcv/src/galileo_e1_codes.rs"
    output_path = repo_root / args.output

    harshadms_text = harshadms_header.read_text(encoding="utf-8")
    mx4_text = mx4_codes.read_text(encoding="utf-8")

    e1b_harshadms = parse_harshadms_primary_codes(harshadms_text, "GALILEO_E1_B_PRIMARY_CODE")
    e1c_harshadms = parse_harshadms_primary_codes(harshadms_text, "GALILEO_E1_C_PRIMARY_CODE")
    e1b_mx4 = parse_mx4_primary_codes(mx4_text, "E1B_HEX")
    e1c_mx4 = parse_mx4_primary_codes(mx4_text, "E1C_HEX")

    require_equal("E1-B primary codes", e1b_harshadms, e1b_mx4)
    require_equal("E1-C primary codes", e1c_harshadms, e1c_mx4)

    secondary_harshadms = parse_harshadms_secondary_code(harshadms_text)
    secondary_mx4 = parse_mx4_secondary_code(mx4_text)
    require_equal("E1-C secondary code", secondary_harshadms, secondary_mx4)

    catalog = render_catalog(
        harshadms_header.relative_to(repo_root),
        mx4_codes.relative_to(repo_root),
        secondary_harshadms,
        e1b_harshadms,
        e1c_harshadms,
    )

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
        description="Generate the Galileo E1 reference catalog used by signal validation tests."
    )
    parser.add_argument(
        "--repo-root",
        default=Path(__file__).resolve().parents[4],
        type=Path,
        help="workspace repository root",
    )
    parser.add_argument(
        "--output",
        default="crates/bijux-gnss-signal/tests/data/galileo_e1_reference_catalog.toml",
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


def parse_harshadms_primary_codes(source: str, marker: str) -> list[str]:
    body = extract_block(source, marker, "{", "};")
    codes: list[str] = []
    fragments: list[str] = []
    for line in body.splitlines():
        fragments.extend(re.findall(r'"([0-9A-F]+)"', line))
        if "// PRN" in line:
            code = "".join(fragments)
            validate_primary_code(marker, len(codes) + 1, code)
            codes.append(code)
            fragments = []
    if fragments:
        raise ValueError(f"incomplete primary-code block for {marker}")
    return codes


def parse_mx4_primary_codes(source: str, marker: str) -> list[str]:
    body = extract_block(source, f"pub const {marker}", "[", "];")
    codes = re.findall(r'"([0-9A-F]{1023})"', body)
    if len(codes) != 50:
        raise ValueError(f"{marker} expected 50 codes, found {len(codes)}")
    for prn, code in enumerate(codes, start=1):
        validate_primary_code(marker, prn, code)
    return codes


def parse_harshadms_secondary_code(source: str) -> str:
    body = extract_block(source, SECONDARY_CODE_MARKER, "{", "};")
    digits = re.findall(r"\b([01])\b", body)
    code = "".join(digits)
    if len(code) != 25:
        raise ValueError(f"{SECONDARY_CODE_MARKER} expected 25 bits, found {len(code)}")
    return code


def parse_mx4_secondary_code(source: str) -> str:
    match = SECONDARY_CODE_RUST_PATTERN.search(source)
    if match is None:
        raise ValueError("missing E1C secondary code in mx4 reference file")
    return match.group(1)


def extract_block(source: str, marker: str, open_delimiter: str, close_delimiter: str) -> str:
    start = source.find(marker)
    if start < 0:
        raise ValueError(f"missing marker {marker}")
    begin = source.find(open_delimiter, start)
    if begin < 0:
        raise ValueError(f"missing opening delimiter for {marker}")
    end = source.find(close_delimiter, begin)
    if end < 0:
        raise ValueError(f"missing closing delimiter for {marker}")
    return source[begin + 1 : end]


def validate_primary_code(marker: str, prn: int, code: str) -> None:
    if len(code) != PRIMARY_CODE_LENGTH_HEX:
        raise ValueError(f"{marker} PRN {prn} expected 1023 hex symbols, found {len(code)}")
    if not re.fullmatch(r"[0-9A-F]{1023}", code):
        raise ValueError(f"{marker} PRN {prn} contains non-hex symbols")


def require_equal(label: str, left: object, right: object) -> None:
    if left != right:
        raise ValueError(f"{label} differ between independent reference sources")


def render_catalog(
    harshadms_path: Path,
    mx4_path: Path,
    secondary_bits: str,
    e1b_codes: list[str],
    e1c_codes: list[str],
) -> str:
    lines = [
        'schema = "galileo_e1_reference_catalog.v1"',
        'reference_origin = "independent upstream code tables cross-checked before catalog generation"',
        f'harshadms_source = "{harshadms_path.as_posix()}"',
        f'mx4_source = "{mx4_path.as_posix()}"',
        f"primary_code_length_bits = {PRIMARY_CODE_LENGTH_BITS}",
        'secondary_code_name = "CS25"',
        f'secondary_code_bits = "{secondary_bits}"',
        f'secondary_code_sha256 = "{sha256_hex(secondary_bits)}"',
        "",
    ]

    for channel, codes in (("E1B", e1b_codes), ("E1C", e1c_codes)):
        for prn, code in enumerate(codes, start=1):
            bits = logical_bits_from_hex(code)
            lines.extend(
                [
                    "[[primary_code]]",
                    f'channel = "{channel}"',
                    f"prn = {prn}",
                    f'bit_sha256 = "{sha256_hex(bits)}"',
                    f'bit_prefix = "{bits[:32]}"',
                    f'bit_suffix = "{bits[-32:]}"',
                    "",
                ]
            )

    return "\n".join(lines).rstrip() + "\n"


def logical_bits_from_hex(code: str) -> str:
    return "".join(f"{int(symbol, 16):04b}" for symbol in code)


def sha256_hex(payload: str) -> str:
    return hashlib.sha256(payload.encode("ascii")).hexdigest()


if __name__ == "__main__":
    raise SystemExit(main())
