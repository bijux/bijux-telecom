.PHONY: ci-dependency-lock-refresh

ci-dependency-lock-refresh: ## Refresh dependency lockfile
	@mkdir -p "$(RS_CARGO_HOME)" "$(RS_TMP_DIR)" "$(RS_TARGET_DIR)"
	@CARGO_HOME="$(RS_CARGO_HOME)" \
		CARGO_TARGET_DIR="$(RS_TARGET_DIR)" \
		TMPDIR="$(RS_TMP_DIR)" \
		cargo update --workspace

BIJUX_CI_DOCS_TARGETS += docs-check
BIJUX_HELP_TARGETS += audit-policy-rs bench-compare ci-dependency-lock-refresh
BIJUX_HELP_audit-policy-rs := Validate GNSS audit policy governance
BIJUX_HELP_bench-compare := Compare GNSS benchmarks with the governed baseline
BIJUX_HELP_ci-dependency-lock-refresh := Refresh the Cargo dependency lockfile
