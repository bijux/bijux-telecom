# Changelog

Changes to `bijux-gnss-policies` executable repository policy are recorded
here. Workspace-wide release notes live in [../../CHANGELOG.md](../../CHANGELOG.md).

## Unreleased

### Added

- Package changelog entrypoint for structural guardrails, dependency rules,
  import layering checks, content policy, policy snapshots, and purity reports.

### Changed

- Public API documentation now presents policy execution as a small read-only
  guardrail contract instead of an internal implementation note.

## What Belongs Here

- Guardrail runner behavior and typed policy configuration.
- Workspace structure, dependency-direction, public-surface, and content-policy
  assertions.
- Policy snapshot inputs and reviewable purity reports.
- Changes that alter what repository structure is accepted or refused.

## What Belongs Elsewhere

- Product runtime behavior belongs to product crates.
- Scientific semantics belong to signal, receiver, navigation, or core.
- Maintainer command UX belongs to `bijux-gnss-dev`.
- Release automation belongs to repository workflow documentation.

## Entry Rules

- Record the rule family and the repository behavior it protects.
- Explain whether a snapshot change is an intentional policy change or expected
  serialization movement.
- Do not describe policy failures as style preferences; document the boundary
  being protected.
