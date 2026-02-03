# bijux-gnss

A Rust-first GNSS software stack. This workspace hosts multiple crates with the `bijux-gnss-*` naming scheme.

## Crates
- `bijux-gnss-receiver` – core receiver library (acquisition, tracking, navigation)
- `bijux-gnss-cli` – command-line interface (`bijux gnss ...`)
- `gnss-signal` – shared DSP utilities

## Workspace Layout
- `crates/` – Rust crates
- `data/` – sample datasets (optional)
- `-old-ref/` – legacy MATLAB reference (not used in builds)

## Build
```bash
cargo build
```

## Test
```bash
cargo test
```

## CLI Examples
```bash
cargo run -p bijux-gnss-cli -- gnss ca-code --prn 1 --count 16
```
