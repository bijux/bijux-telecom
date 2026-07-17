# bijux-gnss

`bijux-gnss` owns the operator-facing GNSS command boundary for the workspace.

## Scope

This crate owns:

- the `bijux` binary
- command names, arguments, and top-level workflow composition
- runtime setup and operator-facing reporting during command execution
- a narrow package façade over lower-level GNSS crates

This crate does not own low-level signal implementations, standalone navigation science, receiver
stage internals, or repository persistence contracts.

## Public surface

This crate has two public surfaces:

- the `bijux` binary for operators
- the small Rust façade from `src/lib.rs` for package-level convenience

Both surfaces are intentionally thin over lower-level crates that own the science and infrastructure
they expose.

## Source map

- `src/main.rs` assembles the binary command surface.
- `src/cli/command_line.rs` owns command parsing and stable argument shape.
- `src/cli/command_runtime.rs` and `src/cli/execution_support.rs` own runtime setup and workflow
  support.
- `src/cli/report.rs` owns operator-facing output rendering.
- `src/lib.rs` owns the package façade over lower-level crates.

## Documentation

The crate root intentionally contains only this README. Durable crate documentation lives under
`docs/`:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/COMMANDS.md](docs/COMMANDS.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/EXECUTION.md](docs/EXECUTION.md)
- [docs/FACADE.md](docs/FACADE.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/REPORTING.md](docs/REPORTING.md)
- [docs/TESTS.md](docs/TESTS.md)
- [docs/VALIDATION.md](docs/VALIDATION.md)
- [docs/WORKFLOWS.md](docs/WORKFLOWS.md)

## Verification

Run from the repository root when changing this crate:

```sh
cargo test -p bijux-gnss --test integration_validate_config
cargo test -p bijux-gnss --test integration_nav_decode
cargo test -p bijux-gnss --test integration_validate_synthetic_navigation
```

Repository-wide expectations are documented in [../../README.md](../../README.md).
