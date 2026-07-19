# Commands

This file is the single source of truth for the `bijux-gnss-dev` command inventory.

## Audit governance commands

### `bijux-gnss-dev audit-allowlist`

Validates `audit-allowlist.toml`:
- every advisory id is a valid `RUSTSEC-YYYY-NNNN`
- every advisory has `why`, `owner`, `link`, and `expiry`
- every expiry is ISO-formatted and not in the past

### `bijux-gnss-dev deny-policy-deviations`

Validates `configs/rust/deny.deviations.toml`:
- every deviation has an id, owner, reason, review link, and expiry
- every review link is HTTP(S)
- every review link references `bijux-std`

### `bijux-gnss-dev audit-ignore-args`

Reads `audit-allowlist.toml` and prints the matching `cargo audit --ignore ...` arguments. This is
an adapter command for CI and Makefile workflows that need one reviewed source of exceptions.

## Benchmark governance command

### `bijux-gnss-dev bench-compare`

Runs the owned benchmark set for `bijux-gnss-receiver` and `bijux-gnss-nav`, writes outputs under
`artifacts/` and `benchmarks/`, and compares the current snapshot against the checked-in baseline.

Flags:
- `--strict` fails the command on threshold violations
- `--threshold <ratio>` sets the regression threshold
- `--workspace-root <path>` overrides the repository root

The workflow-level contract and output locations are documented in the
[workflow guide](WORKFLOWS.md).
