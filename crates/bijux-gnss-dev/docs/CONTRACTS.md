# Contracts

`bijux-gnss-dev` owns repository-governance contracts rather than scientific contracts.

## Audit allowlist contract

The crate owns the quality gate for `audit-allowlist.toml`:
- reviewed advisories are explicit
- every exception has an owner and an expiry
- every exception carries an auditable rationale and review link

This contract keeps security exceptions governed instead of becoming invisible CI folklore.

The owned governance files are enumerated in [GOVERNANCE_FILES.md](GOVERNANCE_FILES.md).
The exception-policy boundary is detailed in [AUDIT_POLICY.md](AUDIT_POLICY.md).

## Deny deviation contract

The crate owns the quality gate for `configs/rust/deny.deviations.toml`:
- deviations must declare owner, reason, review, and expiry
- reviews must point back to `bijux-std`

This keeps local deviations narrow and traceable.

## Benchmark comparison contract

The crate owns:
- benchmark execution for the curated crate set
- snapshot persistence for the current run
- baseline comparison and regression reporting

It does not own benchmark definitions inside product crates; it owns maintainer evaluation of their
output.

The benchmark-governance boundary is detailed in [BENCHMARKS.md](BENCHMARKS.md).
The output locations that carry benchmark evidence are detailed in [OUTPUTS.md](OUTPUTS.md).
