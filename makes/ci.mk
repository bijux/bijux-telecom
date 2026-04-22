.PHONY: ci ci-fast ci-pr ci-nightly ci-docs ci-dependency-lock-refresh

ci: ci-fast ## Canonical CI entrypoint

ci-fast: fmt-rs lint-rs audit-rs test-rs ## Fast lane

ci-pr: ci-fast ## Pull-request lane

ci-nightly: fmt-rs lint-rs audit-rs test-all-rs ## Nightly lane

ci-docs: docs-check ## Docs lane

ci-dependency-lock-refresh: ## Refresh dependency lockfile
	@CARGO_TARGET_DIR="$(RS_TARGET_DIR)" cargo update --workspace
