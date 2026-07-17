# Change Rules

`bijux-gnss-core` is the most amplified crate in the GNSS workspace. Small edits here travel
through every higher-level crate.

## When a change belongs here

A change belongs in `bijux-gnss-core` when it defines shared GNSS meaning that multiple crates must
exchange without reinterpretation:

- contract types
- typed identifiers
- time and unit semantics
- shared diagnostics
- versioned artifact payload rules

If a concept is local to one runtime, one parser, one repository workflow, or one command path, it
does not belong here.

## Required discipline

1. Prefer additive evolution over silent semantic rewrites.
2. When serialized meaning changes, update [SERIALIZATION.md](SERIALIZATION.md),
   [CONTRACTS.md](CONTRACTS.md), and relevant validation tests in the same change set.
3. When public exports change, update [PUBLIC_API.md](PUBLIC_API.md) and keep `api.rs` curated.
4. When invariants change, document the new downstream assumption in [INVARIANTS.md](INVARIANTS.md)
   and keep the protecting tests aligned.

## Smells that usually mean "wrong crate"

- needs filesystem paths or manifests
- needs source scheduling or runtime state machines
- needs domain-specific navigation estimation internals
- exists only to help one CLI workflow

Those belong in `infra`, `receiver`, `nav`, or `gnss`, not here.
