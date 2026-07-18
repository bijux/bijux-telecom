GNSS_RUST_GATE_BIN ?= makes/bin/run_gnss_rust_gate.sh
RUST_GATE_BIN ?= $(GNSS_RUST_GATE_BIN)
NEXTEST_SLOW_NAME_EXPR ?= test(/::slow__/)
RUST_AUDIT_PREREQUISITES += audit-policy-rs

.PHONY: audit-policy-rs bench-compare

audit-policy-rs: ## Validate GNSS audit allowlist and deny-policy governance
	@mkdir -p "$(RS_CARGO_HOME)" "$(RS_TMP_DIR)" "$(RS_TARGET_DIR)"
	@CARGO_HOME="$(RS_CARGO_HOME)" \
		CARGO_TARGET_DIR="$(RS_TARGET_DIR)" \
		TMPDIR="$(RS_TMP_DIR)" \
		cargo run --locked -q -p bijux-gnss-dev -- audit-allowlist
	@CARGO_HOME="$(RS_CARGO_HOME)" \
		CARGO_TARGET_DIR="$(RS_TARGET_DIR)" \
		TMPDIR="$(RS_TMP_DIR)" \
		cargo run --locked -q -p bijux-gnss-dev -- deny-policy-deviations

bench-compare: ## Run benchmark comparison through bijux-gnss-dev
	@mkdir -p "$(RS_CARGO_HOME)" "$(RS_TMP_DIR)" "$(RS_TARGET_DIR)"
	@CARGO_HOME="$(RS_CARGO_HOME)" \
		CARGO_TARGET_DIR="$(RS_TARGET_DIR)" \
		TMPDIR="$(RS_TMP_DIR)" \
		cargo run --locked -q -p bijux-gnss-dev -- bench-compare \
			$(if $(filter 1 true yes,$(BENCH_STRICT)),--strict,)
