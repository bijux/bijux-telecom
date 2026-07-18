# Changelog

Changes to `bijux-gnss-dev` maintainer tooling are recorded here.
Workspace-wide release notes live in [../../CHANGELOG.md](../../CHANGELOG.md).

## Unreleased

### Added

- Package changelog entrypoint for repository maintenance commands, audit
  policy, benchmark comparison, governed output locations, and test-lane
  selection.

### Changed

- Documentation now distinguishes maintainer command governance from product
  CLI behavior and crate runtime APIs.

## What Belongs Here

- Maintainer command behavior under the `bijux-gnss-dev` binary.
- Audit allowlist checks, deny-policy deviation checks, and benchmark evidence.
- Repository test selection policy, slow-lane roster guardrails, and local
  output governance.
- Changes to maintainer-facing command reports.

## What Belongs Elsewhere

- Product CLI workflows belong to `bijux-gnss`.
- GNSS receiver behavior belongs to `bijux-gnss-receiver`.
- Scientific models belong to signal, nav, core, or testkit as appropriate.
- Package runtime APIs belong to library crates, not this private binary crate.

## Entry Rules

- Record how a maintainer command changes repository evidence or enforcement.
- Mention governed files when a command starts validating them.
- Do not document product behavior here just because a maintainer command checks
  it.
