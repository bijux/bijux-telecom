# deploy-docs adoption for Bijux repositories

This checklist maps each repository to the minimum configuration needed to adopt the shared `.github/workflows/deploy-docs.yml` without behavior drift.

## Common rollout steps

1. Run `make bijux-standard-sync` in the consumer repository.
2. If defaults are insufficient, add `.github/docs-deploy.env` from `workflows/deploy-docs.env.example`.
3. Run `make bijux-standard-check`.
4. Open one pull request per repository and validate `deploy-docs` on `workflow_dispatch` before merging.

## Repository matrix

### `bijux-atlas`

- Expected to work with defaults for build/install/toolchain.
- Recommended optional override:
  - `BIJUX_DOCS_VERIFY_COMMAND=make gh-docs-verify` after target is added locally.

### `bijux-canon`

- Recommended override:
  - `BIJUX_DOCS_BUILD_COMMAND=make docs`
- Reason: preserves current deploy behavior and publish-site asset layout.

### `bijux-core`

- Expected to work with defaults (`make gh-docs-install`, `make docs-check`).

### `bijux-masterclass`

- Requires explicit overrides because root `Makefile` does not expose series docs targets.
- Required overrides:
  - `BIJUX_DOCS_INSTALL_COMMAND=make -f makes/series-docs.mk series-docs-install`
  - `BIJUX_DOCS_BUILD_COMMAND=make -f makes/series-docs.mk series-docs-build`
  - `BIJUX_DOCS_SITE_DIR=artifacts/site/bijux-masterclass`

### `bijux-pollenomics`

- Recommended override:
  - `BIJUX_DOCS_BUILD_COMMAND=make docs`
- Reason: matches current deploy flow.

### `bijux-proteomics`

- Recommended override:
  - `BIJUX_DOCS_BUILD_COMMAND=make docs`
- Reason: matches current deploy flow.

### `bijux.github.io`

- Expected to work with defaults.
- Optional override (if a dedicated install step is desired):
  - `BIJUX_DOCS_INSTALL_COMMAND=python -m pip install -r configs/docs/requirements-docs.txt`

## Standardization follow-up

For long-term zero-override operation, each repository should eventually expose the same make contract:

- `gh-docs-install`
- `gh-docs-build`
- `gh-docs-verify`
- `gh-docs-site-dir`

Once those four targets exist everywhere, `docs-deploy.env` overrides can be removed.
