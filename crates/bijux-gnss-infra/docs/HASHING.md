# Hashing

`bijux-gnss-infra` owns repository-facing provenance hashing, not scientific identity.

## Owned helpers

The hashing surface currently owns:

- configuration hashing through `hash_config`
- repository provenance capture through `git_hash` and `git_dirty`
- environment/provenance capture through `cpu_features`

## Why this belongs here

These values are used to describe how a run was prepared and executed inside the repository context.
They are not foundational identifiers in `core`, and they are not command-only concerns in the CLI.

## Boundary rule

Hashing here is about provenance and reproducibility evidence. It should not grow into a general
cryptographic utility bucket or into product-level scientific identity semantics.
