.PHONY: install lock lock-check all clean root-check-env clean-root-artifacts

ROOT_INSTALL_PREREQS ?=
ROOT_INSTALL_COMMAND ?= @true
ROOT_LOCK_FLAGS ?= --python "$(PYTHON)"
ROOT_LOCK_CHECK_FLAGS ?= --check --python "$(PYTHON)"
ROOT_ALL_TARGETS ?= test lint quality security docs api build sbom
ROOT_DEFINE_CLEAN ?= 1
ROOT_CLEAN_PREREQS ?=
ROOT_CLEAN_COMMAND ?= @rm -rf "$(PROJECT_ARTIFACTS_DIR)"
ROOT_CHECK_ENV_PREREQS ?=
ROOT_CHECK_ENV_COMMAND ?= @true
ROOT_CLEAN_ROOT_ARTIFACTS_COMMAND ?= @true

install: $(ROOT_INSTALL_PREREQS) ## Install or sync repository dependencies
	$(ROOT_INSTALL_COMMAND)

lock: ## Refresh uv.lock from pyproject inputs
	@$(UV) lock $(ROOT_LOCK_FLAGS)

lock-check: ## Verify uv.lock matches current pyproject inputs
	@$(UV) lock $(ROOT_LOCK_CHECK_FLAGS)

all: $(ROOT_ALL_TARGETS) ## Run the repository pipeline

ifeq ($(ROOT_DEFINE_CLEAN),1)
clean: $(ROOT_CLEAN_PREREQS) ## Remove repository artifacts and root environments
	$(ROOT_CLEAN_COMMAND)
endif

root-check-env: $(ROOT_CHECK_ENV_PREREQS) ## Validate required root environment inputs
	$(ROOT_CHECK_ENV_COMMAND)

clean-root-artifacts: ## Remove forbidden root-level artifacts
	$(ROOT_CLEAN_ROOT_ARTIFACTS_COMMAND)
