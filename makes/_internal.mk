.DELETE_ON_ERROR:
.DEFAULT_GOAL := help
.SHELLFLAGS := -eu -o pipefail -c
SHELL := bash

RUN_ID ?= local
CARGO_TERM_COLOR ?= always
CARGO_TERM_PROGRESS_WHEN ?= always
CARGO_TERM_PROGRESS_WIDTH ?= 120
CARGO_TERM_VERBOSE ?= false

all: ci-fast ## Run the default Rust gate lane

fmt: fmt-rs ## Run formatting checks
fmt-check: fmt-rs ## Run formatting checks (compatibility alias)
lint: lint-rs ## Run lint checks
clippy: lint-rs ## Run clippy checks (compatibility alias)
test: test-rs ## Run fast test checks
test-all: test-all-rs ## Run full Rust tests including ignored tests
test-full: test-all-rs ## Run full Rust tests (compatibility alias)
coverage: coverage-rs ## Run workspace coverage with cargo llvm-cov + nextest
security: audit-rs ## Run security checks
audit: audit-rs ## Run security checks (compatibility alias)
doc: docs-check ## Run docs checks (compatibility alias)

help: ## Show available make targets
	@awk 'BEGIN{FS=":.*##"; OFS="";} \
	  /^[a-zA-Z0-9_.-]+:.*##/ {printf "  \033[36m%-28s\033[0m %s\n", $$1, $$2}' \
	  $(MAKEFILE_LIST)
.PHONY: all fmt fmt-check lint clippy test test-all test-full coverage security audit doc help
