RUST_GATE_BIN ?= $(BIJUX_MAKES_RS_DIR)/scripts/rust_gate.sh
NEXTEST_EXPR_BIN ?= $(BIJUX_MAKES_RS_DIR)/scripts/nextest_expr.sh
PINNED_GATE_BIN ?= $(BIJUX_MAKES_SHARED_ROOT)/bijux-makes/scripts/run_pinned_gate.sh

RS_ARTIFACT_ROOT ?= $(ARTIFACT_ROOT_ABS)/rust
RS_RUN_ID ?= $(RUN_ID)
RS_TARGET_DIR ?= $(RS_ARTIFACT_ROOT)/target
RS_CARGO_HOME ?= $(RS_ARTIFACT_ROOT)/cargo/home
RS_TMP_DIR ?= $(RS_ARTIFACT_ROOT)/tmp
RS_NEXTEST_CACHE_DIR ?= $(RS_TARGET_DIR)/nextest
RS_NEXTEST_CONFIG_HOME ?= $(RS_ARTIFACT_ROOT)/nextest/config

NEXTEST_CONFIG_FILE ?= $(PROJECT_ROOT)/configs/rust/nextest.toml
NEXTEST_SLOW_ROSTER ?= $(PROJECT_ROOT)/configs/rust/nextest-slow-roster.txt
NEXTEST_PROFILE_FAST ?= default
NEXTEST_PROFILE_SLOW ?= default
NEXTEST_PROFILE_ALL ?= default
NEXTEST_STATUS_LEVEL ?= all
NEXTEST_FINAL_STATUS_LEVEL ?= all
NEXTEST_THREADS ?=
NEXTEST_THREADS_ALL ?=
NEXTEST_SLOW_NAME_EXPR ?= test(/(^|::)slow_/)

RUST_DENY_CONFIG ?= $(PROJECT_ROOT)/configs/rust/deny.toml
RUSTFMT_CONFIG ?=
RUST_CLIPPY_CONFIG_DIR ?= $(PROJECT_ROOT)/configs/rust
RUST_CLIPPY_EXCLUDES ?=
RUST_AUDIT_IGNORE_ARGS ?=
RUST_CARGO_LOCKED ?= 1
RUSTDOC_DENY_WARNINGS ?= 1

RUST_FMT_PREREQUISITES ?=
RUST_LINT_PREREQUISITES ?=
RUST_TEST_PREREQUISITES ?=
RUST_TEST_SLOW_PREREQUISITES ?=
RUST_TEST_ALL_PREREQUISITES ?=
RUST_AUDIT_PREREQUISITES ?=
RUST_COVERAGE_PREREQUISITES ?=
RUSTDOC_PREREQUISITES ?=

export PROJECT_ROOT ARTIFACT_ROOT RS_ARTIFACT_ROOT RS_RUN_ID RS_TARGET_DIR
export RS_CARGO_HOME RS_TMP_DIR RS_NEXTEST_CACHE_DIR RS_NEXTEST_CONFIG_HOME
export NEXTEST_CONFIG_FILE NEXTEST_SLOW_ROSTER NEXTEST_PROFILE_FAST
export NEXTEST_PROFILE_SLOW NEXTEST_PROFILE_ALL NEXTEST_STATUS_LEVEL
export NEXTEST_FINAL_STATUS_LEVEL NEXTEST_THREADS NEXTEST_THREADS_ALL
export NEXTEST_SLOW_NAME_EXPR NEXTEST_EXPR_BIN
export RUST_DENY_CONFIG RUSTFMT_CONFIG RUST_CLIPPY_CONFIG_DIR
export RUST_CLIPPY_EXCLUDES RUST_AUDIT_IGNORE_ARGS RUST_CARGO_LOCKED
export RUSTDOC_DENY_WARNINGS

BIJUX_FORMAT_TARGETS += format-rs
BIJUX_FMT_TARGETS += fmt-rs
BIJUX_LINT_TARGETS += lint-rs
BIJUX_TEST_TARGETS += test-rs
BIJUX_TEST_SLOW_TARGETS += test-slow-rs
BIJUX_TEST_ALL_TARGETS += test-all-rs
BIJUX_AUDIT_TARGETS += audit-rs
BIJUX_SECURITY_TARGETS += audit-rs
BIJUX_COVERAGE_TARGETS += coverage-rs
BIJUX_DOCTOR_TARGETS += doctor-rs
BIJUX_CI_FAST_TARGETS += fmt lint test
BIJUX_CI_PR_TARGETS += ci-fast audit
BIJUX_CI_NIGHTLY_TARGETS += ci-pr test-all coverage
BIJUX_CI_DOCS_TARGETS += rustdoc-check

BIJUX_HELP_TARGETS += audit-frozen audit-rs coverage-rs doctor-rs fmt-rs
BIJUX_HELP_TARGETS += format-rs lint-frozen lint-rs rustdoc-check
BIJUX_HELP_TARGETS += test-all-frozen test-all-rs test-rs test-slow-rs
BIJUX_HELP_audit-frozen := Launch a pinned Rust audit gate
BIJUX_HELP_audit-rs := Run cargo-deny and cargo-audit
BIJUX_HELP_coverage-rs := Generate Rust coverage reports
BIJUX_HELP_doctor-rs := Verify Rust gate inputs and tools
BIJUX_HELP_fmt-rs := Verify Rust formatting
BIJUX_HELP_format-rs := Apply Rust formatting
BIJUX_HELP_lint-frozen := Launch a pinned Rust lint gate
BIJUX_HELP_lint-rs := Run Rust Clippy with warnings denied
BIJUX_HELP_rustdoc-check := Build Rust API documentation with warnings denied
BIJUX_HELP_test-all-frozen := Launch a pinned complete Rust test gate
BIJUX_HELP_test-all-rs := Run all Rust tests including ignored tests
BIJUX_HELP_test-rs := Run the fast Rust test lane
BIJUX_HELP_test-slow-rs := Run the slow Rust test lane

.PHONY: audit-frozen audit-rs coverage-rs doctor-rs fmt-rs format-rs
.PHONY: lint-frozen lint-rs rustdoc-check test-all-frozen test-all-rs test-rs test-slow-rs

doctor-rs: ## Verify Rust gate inputs and tools
	@$(call require_tool,cargo)
	@$(call require_file,$(PROJECT_ROOT)/Cargo.toml)
	@$(call require_file,$(NEXTEST_CONFIG_FILE))
	@$(call require_file,$(RUST_DENY_CONFIG))
	@$(call require_executable,$(RUST_GATE_BIN))
	@$(call require_executable,$(NEXTEST_EXPR_BIN))
	@$(call require_executable,$(PINNED_GATE_BIN))

format-rs: $(RUST_FMT_PREREQUISITES) ## Apply Rust formatting
	@"$(RUST_GATE_BIN)" format

fmt-rs: $(RUST_FMT_PREREQUISITES) ## Verify Rust formatting
	@"$(RUST_GATE_BIN)" fmt

lint-rs: $(RUST_LINT_PREREQUISITES) ## Run Rust Clippy with warnings denied
	@"$(RUST_GATE_BIN)" lint

test-rs: $(RUST_TEST_PREREQUISITES) ## Run the fast Rust test lane
	@"$(RUST_GATE_BIN)" test

test-slow-rs: $(RUST_TEST_SLOW_PREREQUISITES) ## Run the slow Rust test lane
	@"$(RUST_GATE_BIN)" test-slow

test-all-rs: $(RUST_TEST_ALL_PREREQUISITES) ## Run all Rust tests including ignored tests
	@"$(RUST_GATE_BIN)" test-all

audit-rs: $(RUST_AUDIT_PREREQUISITES) ## Run cargo-deny and cargo-audit
	@"$(RUST_GATE_BIN)" audit

coverage-rs: $(RUST_COVERAGE_PREREQUISITES) ## Generate Rust coverage reports
	@"$(RUST_GATE_BIN)" coverage

rustdoc-check: $(RUSTDOC_PREREQUISITES) ## Build Rust API documentation with warnings denied
	@"$(RUST_GATE_BIN)" rustdoc

test-all-frozen: ## Launch a pinned complete Rust test gate
	@PINNED_GATE_TARGET=test-all PINNED_ALLOWED_TARGETS="test-all lint audit" "$(PINNED_GATE_BIN)"

lint-frozen: ## Launch a pinned Rust lint gate
	@PINNED_GATE_TARGET=lint PINNED_ALLOWED_TARGETS="test-all lint audit" "$(PINNED_GATE_BIN)"

audit-frozen: ## Launch a pinned Rust audit gate
	@PINNED_GATE_TARGET=audit PINNED_ALLOWED_TARGETS="test-all lint audit" "$(PINNED_GATE_BIN)"
