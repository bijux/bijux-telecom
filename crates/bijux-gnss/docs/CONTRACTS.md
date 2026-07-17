# Contracts

`bijux-gnss` owns operator command contracts rather than foundational scientific contracts.

## Command contract

This crate owns the stable shape of:
- command families
- arguments and flags
- report-format switches
- operator-facing success and failure output

## Orchestration contract

The crate owns how commands assemble lower-level crates into a user-facing workflow. It does not
own the scientific behavior of those lower-level crates.

The workflow families and their composition boundary are described in [WORKFLOWS.md](WORKFLOWS.md).

## Façade contract

`src/lib.rs` owns a narrow package-level façade over the lower-level GNSS crates. That façade is a
package convenience, not a new mixed-ownership domain.

The façade boundary is described in [FACADE.md](FACADE.md), and operator-facing reporting ownership
is described in [REPORTING.md](REPORTING.md).
