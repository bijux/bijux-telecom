# Boundary

Owner: workspace structural policy and guardrail enforcement

## Scope

`bijux-gnss-policies` owns executable repository rules such as:
- dependency direction checks
- public API exposure discipline
- source-tree topology constraints
- content-policy checks
- workspace-wide structural assertions

## What this crate must own

- typed configuration for guardrail enforcement
- reusable guardrail checks other crates can invoke in tests
- repository-level invariants that are too important to leave as convention
- focused reporting for maintainers reviewing structural drift

## What this crate must not own

- product runtime behavior
- GNSS signal, receiver, navigation, or infrastructure logic
- broad utility helpers unrelated to repository policy
- undocumented project governance hidden in shell-only automation

## Effect model

This crate is allowed to read repository sources and manifests because inspection is its job. It is
not allowed to mutate product state or become a general-purpose workspace tool executor.

## Dependency rule

The crate should prefer generic parsing and filesystem libraries. Policy checks may inspect product
crates, but product crates should not be forced to call into this crate for their runtime behavior.
