# Automation and Contributor Identity Baseline

This repository intentionally records commits from three identities:

1. `bijux` (Bijan Mousavi)
2. `dependabot[bot]`
3. `github-actions[bot]`

## Responsibility Boundaries

### `bijux` (Bijan Mousavi)

Primary maintainer and owner for repository direction and release governance:

- defines architecture and repository policy
- approves and merges pull requests
- performs direct `main` maintenance when required by repository policy
- owns security decisions and dependency-risk acceptance
- owns release intent and final publish decisions

### `dependabot[bot]`

Automated dependency maintenance only:

- opens pull requests for dependency and ecosystem updates
- does not own product behavior or policy decisions
- changes are reviewed and merged by the maintainer

### `github-actions[bot]`

Workflow automation identity:

- creates workflow-origin commits and release automation updates
- updates generated automation outputs when workflows are designed to commit
- should use the canonical noreply identity when a workflow configures Git:
  - `user.name`: `github-actions[bot]`
  - `user.email`: `41898282+github-actions[bot]@users.noreply.github.com`
- does not define repository policy; executes maintainer-defined workflows

## Governance Rule

Bot-authored commit provenance is intentionally retained on `main` so history keeps clear authorship for human decisions, dependency automation, and workflow automation.

## Workflow Ownership Contract

Shared governance workflows are generated from `bijux-std` and synchronized into
consumer repositories. Treat these as managed outputs:

- `.github/workflows/automerge-pr.yml`
- `.github/workflows/bijux-std.yml`
- `.github/workflows/deploy-docs.yml`
- `.github/workflows/github-policy.yml`
- `.github/workflows/release-*.yml`
- `.github/workflows/ci.yml` and `.github/workflows/verify.yml` when rendered
  from manifest wrappers

Contributor rules:

- do not hand-edit generated workflow copies in consumer repositories
- make standards changes in `bijux-std` source-of-truth files
- run standards sync/render to propagate managed updates
- keep repository-owned workflows separate and outside the managed inventory
