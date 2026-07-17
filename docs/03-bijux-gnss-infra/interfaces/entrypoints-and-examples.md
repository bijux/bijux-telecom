---
title: Entrypoints and Examples
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Entrypoints and Examples

The best infra examples are repository-state examples.

## Example: Load A Dataset Registry

```rust
use std::path::Path;
use bijux_gnss_infra::api::DatasetRegistry;

let registry = DatasetRegistry::load(Path::new("datasets/registry.toml"))?;
```

## Example: Expand A Sweep

```rust
use bijux_gnss_infra::api::expand_sweep;

let spec = vec![
    ("sampling_hz".to_string(), vec!["1023000".to_string(), "4092000".to_string()]),
];
let cases = expand_sweep(&spec);
```

## Example: Validate Persisted References

```rust
use bijux_gnss_infra::api::{validate_reference, ReferenceAlign};

let _aligned = validate_reference(&solutions, &reference_epochs, ReferenceAlign::Closest)?;
```

These examples stay small on purpose. The crate’s public value is typed
repository interpretation, not full workflow narration.

## Protecting Proof

Inspect `crates/bijux-gnss-infra/src/api.rs`,
`crates/bijux-gnss-infra/docs/PUBLIC_API.md`,
`crates/bijux-gnss-infra/docs/CONTRACTS.md`, and the most relevant dataset,
run-layout, or validation tests to confirm these examples still match real
repository entrypoints.
