# Boundary

Owner: maintainer tooling for the GNSS repository

## Scope

`bijux-gnss-dev` owns repository maintenance workflows implemented as typed commands:
- audit allowlist validation
- deny-policy governance validation
- derived `cargo audit` ignore arguments
- benchmark comparison and benchmark evidence emission

## What this crate must own

- command-line parsing for maintainer-only commands
- repository-file validation logic for the workflows above
- benchmark comparison logic that belongs to repository maintenance rather than product execution
- explicit artifact writing for benchmark evidence

## What this crate must not own

- product CLI behavior for operators
- GNSS signal processing, navigation estimation, or receiver orchestration
- generic repository scripting that has no stable owner
- hidden writes outside governed output locations

## Dependency rule

This crate should depend only on general-purpose libraries and policy crates needed to perform its
maintainer workflow. It should not pull product crates into scope just to reach around typed
interfaces.

## Effect model

This crate is allowed to read repository files, print diagnostics, and write benchmark artifacts.
Those effects are the point of the crate, but they must remain explicit and repository-scoped.
