RS_ARTIFACT_ROOT ?= artifacts/rust
RS_RUN_ID ?= local

RS_TARGET_DIR ?= $(abspath $(RS_ARTIFACT_ROOT)/target)
RS_NEXTEST_CACHE_DIR ?= $(RS_TARGET_DIR)/nextest
RS_NEXTEST_CONFIG_HOME ?= $(abspath $(RS_ARTIFACT_ROOT)/nextest/config)
RS_PROFRAW_DIR ?= $(abspath $(RS_ARTIFACT_ROOT)/coverage/profraw)
RS_LLVM_PROFILE_FILE ?= $(RS_PROFRAW_DIR)/default_%m_%p.profraw

RS_FMT_REPORT ?= $(RS_ARTIFACT_ROOT)/fmt/$(RS_RUN_ID)/report.txt
RS_LINT_REPORT ?= $(RS_ARTIFACT_ROOT)/lint/$(RS_RUN_ID)/report.txt
RS_TEST_REPORT ?= $(RS_ARTIFACT_ROOT)/test/$(RS_RUN_ID)/nextest.log
RS_TEST_ALL_REPORT ?= $(RS_ARTIFACT_ROOT)/test/$(RS_RUN_ID)/nextest-all.log
RS_AUDIT_REPORT ?= $(RS_ARTIFACT_ROOT)/audit/$(RS_RUN_ID)/report.txt

NEXTEST_PROFILE ?= default
NEXTEST_STATUS_LEVEL ?= all
NEXTEST_FINAL_STATUS_LEVEL ?= all

define rs_require_tool
	@command -v $(1) >/dev/null 2>&1 || { \
		echo "$(1) is required but not installed"; \
		exit 1; \
	}
endef

define rs_nextest_summary
	summary_line=$$(perl -pe 's/\e\[[0-9;]*[[:alpha:]]//g' "$(1)" | grep 'Summary \[' | tail -n 1 || true); \
	printf '\033[1;36m%s\033[0m %s\n' "nextest-summary:" "$${summary_line:-unavailable}"
endef

.PHONY: fmt-rs lint-rs test-rs test-all-rs audit-rs bench-compare docs-check ci-docs

fmt-rs: ## Run Rust formatting checks
	@mkdir -p "$(dir $(RS_FMT_REPORT))"
	@set -o pipefail; \
	CARGO_TARGET_DIR="$(RS_TARGET_DIR)" \
	CARGO_TERM_COLOR="$(CARGO_TERM_COLOR)" \
	cargo fmt --all -- --check 2>&1 | tee "$(RS_FMT_REPORT)"

lint-rs: ## Run Rust clippy checks with -D warnings
	@mkdir -p "$(dir $(RS_LINT_REPORT))"
	@set -o pipefail; \
	CLIPPY_CONF_DIR="configs/rust" \
	CARGO_TARGET_DIR="$(RS_TARGET_DIR)" \
	CARGO_TERM_COLOR="$(CARGO_TERM_COLOR)" \
	cargo clippy --workspace --all-targets --all-features --locked -- -D warnings 2>&1 | tee "$(RS_LINT_REPORT)"

test-rs: ## Run Rust fast tests with nextest
	$(call rs_require_tool,cargo-nextest)
	@mkdir -p "$(dir $(RS_TEST_REPORT))" "$(RS_PROFRAW_DIR)" "$(RS_NEXTEST_CONFIG_HOME)"
	@status=0; \
	LLVM_PROFILE_FILE="$(RS_LLVM_PROFILE_FILE)" \
	XDG_CONFIG_HOME="$(RS_NEXTEST_CONFIG_HOME)" \
	CARGO_TARGET_DIR="$(RS_TARGET_DIR)" \
	NEXTEST_CACHE_DIR="$(RS_NEXTEST_CACHE_DIR)" \
	CARGO_TERM_COLOR="$(CARGO_TERM_COLOR)" \
	cargo nextest run \
		--workspace \
		--config-file configs/rust/nextest.toml \
		--profile "$(NEXTEST_PROFILE)" \
		--status-level "$(NEXTEST_STATUS_LEVEL)" \
		--final-status-level "$(NEXTEST_FINAL_STATUS_LEVEL)" \
		2>&1 | tee "$(RS_TEST_REPORT)" || status=$$?; \
	$(call rs_nextest_summary,$(RS_TEST_REPORT)); \
	test $$status -eq 0

test-all-rs: ## Run full Rust tests with nextest including ignored
	$(call rs_require_tool,cargo-nextest)
	@mkdir -p "$(dir $(RS_TEST_ALL_REPORT))" "$(RS_PROFRAW_DIR)" "$(RS_NEXTEST_CONFIG_HOME)"
	@status=0; \
	LLVM_PROFILE_FILE="$(RS_LLVM_PROFILE_FILE)" \
	XDG_CONFIG_HOME="$(RS_NEXTEST_CONFIG_HOME)" \
	CARGO_TARGET_DIR="$(RS_TARGET_DIR)" \
	NEXTEST_CACHE_DIR="$(RS_NEXTEST_CACHE_DIR)" \
	CARGO_TERM_COLOR="$(CARGO_TERM_COLOR)" \
	cargo nextest run \
		--workspace \
		--run-ignored all \
		--retries 0 \
		--config-file configs/rust/nextest.toml \
		--profile "$(NEXTEST_PROFILE)" \
		--status-level "$(NEXTEST_STATUS_LEVEL)" \
		--final-status-level "$(NEXTEST_FINAL_STATUS_LEVEL)" \
		2>&1 | tee "$(RS_TEST_ALL_REPORT)" || status=$$?; \
	$(call rs_nextest_summary,$(RS_TEST_ALL_REPORT)); \
	test $$status -eq 0

audit-rs: ## Run cargo-deny and cargo-audit
	$(call rs_require_tool,cargo-deny)
	$(call rs_require_tool,cargo-audit)
	@mkdir -p "$(dir $(RS_AUDIT_REPORT))"
	@set -o pipefail; \
	deny_status=0; \
	audit_status=0; \
	{ \
		echo "run: cargo run -q -p bijux-telecom-dev -- audit-allowlist"; \
		CARGO_TARGET_DIR="$(RS_TARGET_DIR)" cargo run -q -p bijux-telecom-dev -- audit-allowlist || audit_status=$$?; \
		echo; \
		echo "run: cargo deny check bans licenses sources --config configs/rust/deny.toml"; \
		CARGO_TARGET_DIR="$(RS_TARGET_DIR)" cargo deny check bans licenses sources --config configs/rust/deny.toml || deny_status=$$?; \
		echo; \
		echo "run: cargo audit"; \
		CARGO_TARGET_DIR="$(RS_TARGET_DIR)" cargo audit || audit_status=$$?; \
	} 2>&1 | tee "$(RS_AUDIT_REPORT)"; \
	test $$deny_status -eq 0; \
	test $$audit_status -eq 0

bench-compare: ## Run benchmark comparison through bijux-telecom-dev
	@CARGO_TARGET_DIR="$(RS_TARGET_DIR)" cargo run --locked -q -p bijux-telecom-dev -- bench-compare $(if $(filter 1 true yes,$(BENCH_STRICT)),--strict,)

docs-check: ## Build rustdoc as docs lane baseline
	@CARGO_TARGET_DIR="$(RS_TARGET_DIR)" cargo doc --workspace --no-deps

ci-docs: docs-check ## Alias for docs-check
