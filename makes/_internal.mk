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
lint: lint-rs ## Run lint checks
test: test-rs ## Run fast test checks
security: audit-rs ## Run security checks

help: ## Show available make targets
	@awk 'BEGIN{FS=":.*##"; OFS="";} \
	  /^[a-zA-Z0-9_.-]+:.*##/ {printf "  \033[36m%-28s\033[0m %s\n", $$1, $$2}' \
	  $(MAKEFILE_LIST)
.PHONY: all fmt lint test security help
