# Audit Policy

`bijux-gnss-dev` owns the repository policy around controlled security and standards exceptions.

## Owned policy checks

The crate currently governs:

- `audit-allowlist.toml` quality and expiry discipline
- `configs/rust/deny.deviations.toml` ownership, review-link, and expiry discipline
- derived `cargo audit --ignore ...` arguments from the reviewed allowlist

## Why this is a crate-owned surface

These files are not casual configuration. They are documented exceptions to normal security and
standards posture. If the repository cannot explain who owns them, why they exist, and when they
expire, they become silent debt.

## Boundary rule

This crate owns policy enforcement over those exception files. It does not own the underlying
third-party advisories or upstream standards decisions themselves.
