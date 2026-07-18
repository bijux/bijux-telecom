---
title: Release And Versioning
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Release And Versioning

`bijux-gnss` releases six crates under one version. Three workspace crates
remain repository-only. This page defines that boundary and the evidence a
maintainer must inspect before any registry receives a release.

## What Publishes

Publish in dependency order:

1. `bijux-gnss-core`
2. `bijux-gnss-signal`
3. `bijux-gnss-nav`
4. `bijux-gnss-receiver`
5. `bijux-gnss-infra`
6. `bijux-gnss`

Do not publish `bijux-gnss-dev`, `bijux-gnss-policies`, or
`bijux-gnss-testkit`. Their manifests declare `publish = false`, and public
packages keep references to them path-only and development-only so Cargo omits
them from registry manifests.

The [crate release contract](https://github.com/bijux/bijux-gnss/blob/main/configs/release/crates.toml) is the
machine-readable source for the allowlist, denylist, package roles, required
metadata, and publication order.

## Release Channels

| channel | public artifact | ownership |
| --- | --- | --- |
| crates.io | six Rust packages and the `bijux` binary from `bijux-gnss` | Cargo manifests and crate archives |
| docs.rs | Rust API documentation for each published package | package `documentation` metadata |
| GHCR | one ORAS source bundle per published package | release artifact builder plus shared workflow |
| GitHub Releases | tagged release record and attached package bundles | shared workflow and release configuration |

GHCR bundles are source release artifacts, not runnable container images. Do
not describe them as Docker images or imply that a library crate starts a
service.

## Local Proof

Validate the complete publication boundary:

```sh
make release-check
```

Exercise Cargo's publish path without uploading:

```sh
make publish-rs
```

`publish-rs` defaults to dry-run mode and follows the configured dependency
order. An intentional external publication must set
`RUST_PUBLISH_DRY_RUN=0`; that action requires registry credentials and a
reviewed release tag.

Build the source bundle consumed by GHCR and GitHub release automation:

```sh
make -f makes/release.mk \
  -C crates/bijux-gnss-core \
  release-crate-build \
  ARTIFACTS_DIR=artifacts/dist/bijux-gnss-core
```

Repeat that target through the release build matrix rather than assembling
archives by hand.

## Managed Workflow Boundary

The release workflows and `.github/release.env` are synchronized from
`bijux-std`; they are not repository-owned templates. Enabling the external
release lanes requires an accepted `bijux-std` profile that:

- enables crates.io, GHCR, and GitHub release channels
- installs the shared release workflows in this repository
- supplies six per-crate build and GHCR package matrix entries
- points crates.io publication at `make publish-rs`
- points release builds at `makes/release.mk`

Do not hand-edit synchronized workflow copies or create local lookalikes. The
repository-owned manifests, licenses, READMEs, validator, and bundle builder
must be ready before the standards profile is activated.

## Version And Tag Rules

- all six public crates inherit the workspace version
- a stable release tag is `v<major>.<minor>.<patch>`
- the tag version must equal the workspace version
- package changelogs and the workspace changelog must describe the same
  release
- crates.io publication follows dependency order; GitHub and GHCR artifacts
  use the same tagged source
- a failed channel is visible release evidence, not permission to retag or
  silently skip a package

## First Proof Check

Run `make release-check`, inspect the normalized manifest with
`cargo metadata --no-deps --format-version 1`, and build at least the first
dependency's source bundle. Before an external release, also verify that the
accepted standards profile installed and enabled all three release workflows.
