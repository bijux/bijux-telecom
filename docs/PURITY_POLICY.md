# Purity Policy

## Pure Zones
The following modules are treated as pure/functional and must remain deterministic:
- `crates/bijux-gnss-receiver/src/pipeline/`
- `crates/bijux-gnss-core/src/` (data models and math)

### Forbidden In Pure Zones
- Environment access (`std::env`, `env::var`)
- Wall-clock time (`Instant::now`, `SystemTime::now`)
- Randomness (`rand::`, `thread_rng`, `StdRng` without explicit seeding)
- Direct printing/logging (`println!`, `eprintln!`, `tracing::`, `log::`)
- File/network I/O (`std::fs`, `File`, `OpenOptions`, `std::net`)

## Effects Boundary
Side effects are only allowed in:
- CLI crate (`crates/bijux-gnss-cli`)
- Receiver runtime boundary (`crates/bijux-gnss-receiver/src/engine/runtime.rs` and sink implementations)
- Explicit I/O adapters under `crates/bijux-gnss-receiver/src/io/`

## Rationale
Keeping the pipeline deterministic makes outputs reproducible, simplifies testing, and allows safe caching and parallelism.

## Enforcement
Guardrails checks scan the pure zones for forbidden symbols. Violations must be fixed or justified with a design review.
