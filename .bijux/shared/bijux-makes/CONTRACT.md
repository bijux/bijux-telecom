# Shared Make Contract

`bijux-makes` defines the language-neutral Make contract for Bijux repositories.
Repositories consume the synchronized copy under `.bijux/shared/bijux-makes/`.

## Inclusion

The repository root `Makefile` should include one repository-owned entrypoint:

```make
include makes/root.mk
```

The repository-owned entrypoint configures capabilities before loading the shared
contract:

```make
BIJUX_MAKE_COMPONENTS := docs rust
include .bijux/shared/bijux-makes/bijux.mk
```

Supported components are `docs` and `rust`. The Python Make library remains an
independent contract under `bijux-makes-py`.

## Public Behavior

- The default goal is `help`.
- `fmt` and `lint` verify source without modifying it.
- Source mutation uses explicit targets such as `format` and `format-rs`.
- Unsupported language gates are absent. They must not succeed as no-ops.
- `ci` is the canonical pull-request lane and delegates to `ci-pr`.
- `ci-fast`, `ci-pr`, `ci-nightly`, and `ci-docs` are assembled from enabled
  components.
- Generated output, caches, reports, and isolated source trees stay under
  `artifacts/`.
- `clean` refuses to remove paths outside the repository's `artifacts/`
  boundary.
- Commands preserve nonzero exit status through logging pipelines.

## Extension

Components register concrete targets through these aggregate variables:

```make
BIJUX_FORMAT_TARGETS
BIJUX_FMT_TARGETS
BIJUX_LINT_TARGETS
BIJUX_TEST_TARGETS
BIJUX_TEST_SLOW_TARGETS
BIJUX_TEST_ALL_TARGETS
BIJUX_AUDIT_TARGETS
BIJUX_SECURITY_TARGETS
BIJUX_COVERAGE_TARGETS
BIJUX_DOCTOR_TARGETS
BIJUX_CI_FAST_TARGETS
BIJUX_CI_PR_TARGETS
BIJUX_CI_NIGHTLY_TARGETS
BIJUX_CI_DOCS_TARGETS
```

Repository-owned extensions may append durable domain targets to these variables
before including `bijux.mk`. A configured target that does not exist is an error
reported by Make.

## Documentation

The common documentation component owns MkDocs execution and artifact placement.
The separate `bijux-docs` standard owns the shared visual shell, assets, and shell
validation. Repositories connect those concerns through `DOCS_PREPARE_TARGETS`
and `DOCS_SOURCE_CHECK_TARGETS`.

## Pinned Gates

`scripts/run_pinned_gate.sh` launches an allowed Make target from an immutable
commit checkout under `artifacts/<commit>/frozen-repo/`. The launcher records its
commit, process, log, and exit status under the same artifact root. Repository-relative
Make and Rust paths are recomputed from the immutable checkout instead of inherited
from the invoking worktree. `PINNED_REF` is the canonical commit selector;
`TEST_ALL_FROZEN_REF` remains supported for established full-suite invocations.
