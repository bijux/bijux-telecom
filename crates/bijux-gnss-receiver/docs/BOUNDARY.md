# Boundary

Owner: receiver runtime orchestration and stage execution

## Scope

`bijux-gnss-receiver` owns:
- receiver configuration and runtime state
- acquisition, tracking, observation, and navigation stage orchestration
- source/sink and clock boundary abstractions
- receiver-run artifact aggregation
- synthetic receiver execution and validation-report helpers exposed at the receiver boundary

## What this crate must not own

- repository run directories and manifest persistence
- operator command parsing, formatting, and workflow policy
- low-level signal-code generation or spectrum math already owned by `signal`
- standalone navigation science and file-format ownership already owned by `nav`

## Dependency rule

This crate may depend downward on `core`, `signal`, and optional `nav`, but higher-level crates
should interact with it through the public API rather than stage internals.

## Effect model

This crate is allowed to own runtime-facing effects such as pulling samples from sources, sending
artifacts to sinks, and carrying runtime metrics/logging state. Repository-facing persistence
contracts still belong in `infra`.
