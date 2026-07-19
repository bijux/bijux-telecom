# Public API

`bijux-gnss-dev` does not publish a Rust library API. Its durable public surface
is the `bijux-gnss-dev` binary command set used by maintainers.

## Public Surface Flow

```mermaid
flowchart LR
    maintainer["maintainer"]
    binary["bijux-gnss-dev binary"]
    governed["governed repository files"]
    report["maintainer report"]

    maintainer --> binary
    binary --> governed
    governed --> report
```

## Public Surface

| surface | responsibility |
| --- | --- |
| binary command set | Maintainer workflows for audit allowlists, deny deviations, benchmark comparison, and repository checks. |
| `src/main.rs` | Binary-owned implementation boundary. |
| absence of `lib.rs` | Prevents product crates from depending on maintainer-only internals. |

## Boundary Rules

- Downstream crates should not depend on this package as a library.
- Reusable product behavior belongs in the owning product crate.
- Reusable policy execution belongs in `bijux-gnss-policies`.
- This binary may orchestrate maintainer workflows, but it should not become a
  general-purpose product API.

## Review Checks

- New command behavior needs docs and tests or documented verification.
- If a capability here needs reuse by product crates, extract it to the owning
  crate and call it from this binary.
- Do not add a library surface only to make command internals easier to test.
