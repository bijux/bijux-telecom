# Makes Public Surface

This directory owns the repository's stable `make` contract. Public targets stay meaningful across
refactors; helper targets remain internal.

## Stable Targets

- `fmt`
- `lint`
- `audit`
- `test`
- `test-slow`
- `test-all`
- `test-all-frozen`
- `lint-frozen`
- `audit-frozen`
- `coverage`
- `docs`
- `docs-check`
- `docs-serve`
- `rustdoc-check`
- `ci`

All other targets are implementation details.

## Frozen Gates

Frozen gates run a detached background lane from a pinned repository commit instead of from the
mutable working tree. The canonical pin variable is `PINNED_REF`; `TEST_ALL_FROZEN_REF` remains a
cross-repository compatibility alias.

Examples:

```bash
PINNED_REF=HEAD make lint-frozen
PINNED_REF=01d26ba9 make audit-frozen
TEST_ALL_FROZEN_REF=01d26ba9 make test-all-frozen
```

The launcher resolves the target commit, clones that exact snapshot under
`artifacts/<sha>/frozen-repo/`, and runs the requested gate from the pinned checkout. This keeps
the code, reports, and cargo state aligned to the same immutable source tree.

### Artifact Layout

- `artifacts/<sha>/frozen-repo/`: detached checkout for the pinned commit
- `artifacts/<sha>/rust/`: primary gate reports
- `artifacts/<sha>/target/<gate>/`: isolated cargo target directory
- `artifacts/<sha>/cargo/home/<gate>/`: isolated cargo home for registries and advisory data
- `artifacts/<sha>/tmp/<gate>/`: gate-local temporary directory
- `artifacts/<sha>/background/`: launcher metadata, console logs, pid files, and exit status

### Background Control Files

- `<gate>.console.log`: merged stdout/stderr for the detached launcher
- `<gate>.pid`: process identifier for the detached launcher
- `<gate>.meta`: resolved ref, commit, and artifact paths
- `<gate>.exit.status`: final numeric exit status written when the gate completes

## Target Mapping

- `fmt` -> `cargo fmt --all -- --check`
- `lint` -> `cargo clippy --workspace --all-targets --all-features --locked -- -D warnings`
- `audit` -> `cargo run -q -p bijux-gnss-dev -- audit-allowlist` + `deny-policy-deviations` + `cargo deny` + `cargo audit`
- `test` -> nextest lane excluding `slow__` tests and the governed slow roster
- `test-slow` -> nextest lane containing only `slow__` tests and the governed slow roster
- `test-all` -> full nextest lane with ignored tests enabled and no fast-lane filtering
- `test-all-frozen` -> pinned `test-all`
- `lint-frozen` -> pinned `lint`
- `audit-frozen` -> pinned `audit`
- `docs` -> strict MkDocs build under `artifacts/docs/site`
- `docs-check` -> shared shell, Markdown table, strict build, and rendered-site validation
- `docs-serve` -> local MkDocs development server with automatic reloads
- `rustdoc-check` -> workspace Rust API documentation build

## Boundary

The make surface owns orchestration, artifact placement, and pinned-run isolation. It does not own
GNSS behavior, validation policy semantics, or domain logic; those remain in the workspace crates
that implement the corresponding checks.
