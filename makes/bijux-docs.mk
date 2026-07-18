# Shared Bijux docs shell synchronization and contract enforcement.

PYTHON_BIN ?= $(shell command -v python3 2>/dev/null)
UV_BIN ?= $(shell command -v uv 2>/dev/null)
DOCS_REQUIREMENTS ?= configs/docs/requirements-docs.txt
BIJUX_DOCS_SYNC_SCRIPT ?= .bijux/shared/bijux-docs/tooling/scripts/sync_bijux_docs.sh
BIJUX_DOCS_SOT_GUARD ?= .bijux/shared/bijux-docs/tooling/scripts/verify_bijux_docs_source_of_truth.sh
BIJUX_DOCS_CONTRACT_GUARD ?= .bijux/shared/bijux-docs/tooling/quality/validate_bijux_docs_contract.py
BIJUX_DOCS_ARTIFACTS_DIR ?= artifacts/bijux-docs
BIJUX_DOCS_LOG_DIR ?= $(BIJUX_DOCS_ARTIFACTS_DIR)/logs
BIJUX_DOCS_PYCACHE_DIR ?= $(BIJUX_DOCS_ARTIFACTS_DIR)/pycache
BIJUX_DOCS_XDG_CACHE_DIR ?= $(BIJUX_DOCS_ARTIFACTS_DIR)/xdg-cache
BIJUX_DOCS_HYPOTHESIS_DIR ?= $(BIJUX_DOCS_ARTIFACTS_DIR)/hypothesis
BIJUX_DOCS_CONTRACT_VENV_DIR ?= $(BIJUX_DOCS_ARTIFACTS_DIR)/venv
BIJUX_DOCS_PYTHON_BIN = $(or $(DOCS_PYTHON_BIN),$(PYTHON_BIN))

ifeq ($(strip $(UV_BIN)),)
BIJUX_DOCS_CONTRACT_RUN = "$(BIJUX_DOCS_PYTHON_BIN)"
else
BIJUX_DOCS_CONTRACT_RUN = UV_PROJECT_ENVIRONMENT="$(BIJUX_DOCS_CONTRACT_VENV_DIR)" "$(UV_BIN)" run --with-requirements "$(DOCS_REQUIREMENTS)" python
endif

.PHONY: bijux-docs-sync bijux-docs-check shell-sync shell-check

bijux-docs-sync: ## Synchronize the shared Bijux docs shell into docs assets
	@mkdir -p "$(BIJUX_DOCS_LOG_DIR)"
	@bash -o pipefail -c 'bash "$(BIJUX_DOCS_SYNC_SCRIPT)" 2>&1 | tee "$(BIJUX_DOCS_LOG_DIR)/sync.log"'

bijux-docs-check: ## Validate the Bijux docs shell contract and generated assets
	@mkdir -p "$(BIJUX_DOCS_LOG_DIR)" "$(BIJUX_DOCS_PYCACHE_DIR)" "$(BIJUX_DOCS_XDG_CACHE_DIR)" "$(BIJUX_DOCS_HYPOTHESIS_DIR)" "$(BIJUX_DOCS_CONTRACT_VENV_DIR)"
	@PYTHONPYCACHEPREFIX="$(abspath $(BIJUX_DOCS_PYCACHE_DIR))" XDG_CACHE_HOME="$(abspath $(BIJUX_DOCS_XDG_CACHE_DIR))" HYPOTHESIS_STORAGE_DIRECTORY="$(abspath $(BIJUX_DOCS_HYPOTHESIS_DIR))" $(BIJUX_DOCS_CONTRACT_RUN) "$(BIJUX_DOCS_CONTRACT_GUARD)" . 2>&1 | tee "$(BIJUX_DOCS_LOG_DIR)/contract.log"
	@bash -o pipefail -c 'bash "$(BIJUX_DOCS_SOT_GUARD)" 2>&1 | tee "$(BIJUX_DOCS_LOG_DIR)/source-of-truth.log"'

shell-sync: bijux-docs-sync ## Compatibility alias for bijux-docs-sync
shell-check: bijux-docs-check ## Compatibility alias for bijux-docs-check
