VENV_PYTHON ?= python3
RUFF        ?= $(VENV_PYTHON) -m ruff
MYPY        ?= $(VENV_PYTHON) -m mypy
CODESPELL   ?= $(if $(ACT),$(ACT)/codespell,codespell)
PYDOCSTYLE  ?= $(VENV_PYTHON) -m pydocstyle
RADON       ?= $(VENV_PYTHON) -m radon

include $(abspath $(dir $(lastword $(MAKEFILE_LIST))))/util.mk

LINT_SCOPE             ?=
LINT_DIRS              ?= src tests
FMT_DIRS               ?= $(if $(LINT_SCOPE),$(LINT_SCOPE),$(LINT_DIRS))
LINT_TARGETS           ?= $(if $(LINT_SCOPE),$(LINT_SCOPE),$(LINT_DIRS))
MYPY_TARGETS           ?= $(LINT_TARGETS)
CODESPELL_TARGETS      ?= $(LINT_TARGETS)
RADON_TARGETS          ?= $(LINT_TARGETS)
PYDOCSTYLE_TARGETS     ?= $(LINT_TARGETS)
LINT_PRE_TARGETS       ?=

LINT_ARTIFACTS_DIR     ?= $(PROJECT_ARTIFACTS_DIR)/lint
FMT_LOG                ?= $(LINT_ARTIFACTS_DIR)/fmt.log
RUFF_CACHE_DIR         ?= $(LINT_ARTIFACTS_DIR)/.ruff_cache
MYPY_CACHE_DIR         ?= $(LINT_ARTIFACTS_DIR)/.mypy_cache
LINT_PYCACHE_PREFIX    ?= $(LINT_ARTIFACTS_DIR)/pycache
LINT_SELF_MAKE         ?= $(SELF_MAKE)

RUFF_CONFIG            ?= $(CONFIG_DIR)/ruff.toml
MYPY_CONFIG            ?= $(CONFIG_DIR)/mypy.ini
MYPY_FLAGS             ?= --strict
MYPY_CORE_CONFIG       ?=
MYPY_CORE_FLAGS        ?= --strict
MYPY_CORE_TARGETS      ?=
MYPY_EXTENDED_CONFIG   ?=
MYPY_EXTENDED_FLAGS    ?=
MYPY_EXTENDED_TARGETS  ?=
PYDOCSTYLE_ARGS        ?= --convention=google
RADON_COMPLEXITY_MAX   ?=

ENABLE_MYPY            ?= 1
ENABLE_CODESPELL       ?= 1
ENABLE_RADON           ?= 1
ENABLE_PYDOCSTYLE      ?= 0
RUFF_CHECK_FIX         ?= 0
FMT_RUN_RUFF_CHECK_FIX ?= 0

RUFF_FIX_FLAG := $(if $(filter 1,$(RUFF_CHECK_FIX)),--fix,)
LINT_PYCACHE_ENV := PYTHONPYCACHEPREFIX="$(abspath $(LINT_PYCACHE_PREFIX))"
MYPY_RUN_DIR ?= $(MONOREPO_ROOT)
MYPY_CONFIG_ABS := $(abspath $(MYPY_CONFIG))
MYPY_CACHE_DIR_ABS := $(abspath $(MYPY_CACHE_DIR))
MYPY_TARGETS_ABS := $(foreach target,$(MYPY_TARGETS),$(if $(filter /%,$(target)),$(target),$(abspath $(target))))
MYPY_CORE_CONFIG_ABS := $(if $(MYPY_CORE_CONFIG),$(abspath $(MYPY_CORE_CONFIG)))
MYPY_CORE_TARGETS_ABS := $(foreach target,$(MYPY_CORE_TARGETS),$(if $(filter /%,$(target)),$(target),$(abspath $(target))))
MYPY_EXTENDED_CONFIG_ABS := $(if $(MYPY_EXTENDED_CONFIG),$(abspath $(MYPY_EXTENDED_CONFIG)))
MYPY_EXTENDED_TARGETS_ABS := $(foreach target,$(MYPY_EXTENDED_TARGETS),$(if $(filter /%,$(target)),$(target),$(abspath $(target))))

.PHONY: fmt fmt-artifacts lint lint-artifacts lint-file lint-dir lint-clean mypy-core mypy-extended

fmt: fmt-artifacts
	@echo "✔ Formatting completed (logs in '$(FMT_LOG)')"

fmt-artifacts: | $(VENV)
	@mkdir -p "$(LINT_ARTIFACTS_DIR)" "$(RUFF_CACHE_DIR)"
	@$(LINT_PYCACHE_ENV) $(RUFF) format --config "$(RUFF_CONFIG)" --cache-dir "$(RUFF_CACHE_DIR)" $(FMT_DIRS) 2>&1 | tee "$(FMT_LOG)"; test $${PIPESTATUS[0]} -eq 0
	@if [ "$(FMT_RUN_RUFF_CHECK_FIX)" = "1" ]; then \
	  $(LINT_PYCACHE_ENV) $(RUFF) check --config "$(RUFF_CONFIG)" --fix --cache-dir "$(RUFF_CACHE_DIR)" $(FMT_DIRS) 2>&1 | tee "$(LINT_ARTIFACTS_DIR)/fmt-ruff-fix.log"; test $${PIPESTATUS[0]} -eq 0; \
	fi

lint: lint-artifacts
	@echo "✔ Linting completed (logs in '$(LINT_ARTIFACTS_DIR)')"

lint-artifacts: | $(VENV)
	@mkdir -p "$(LINT_ARTIFACTS_DIR)" "$(RUFF_CACHE_DIR)" "$(MYPY_CACHE_DIR)"
	$(call run_make_targets,$(LINT_PRE_TARGETS),$(LINT_SELF_MAKE))
	@{ \
	  echo "→ Ruff format (check)"; \
	  $(LINT_PYCACHE_ENV) $(RUFF) format --check --config "$(RUFF_CONFIG)" --cache-dir "$(RUFF_CACHE_DIR)" $(LINT_TARGETS); \
	} 2>&1 | tee "$(LINT_ARTIFACTS_DIR)/ruff-format.log"; test $${PIPESTATUS[0]} -eq 0
	@$(LINT_PYCACHE_ENV) $(RUFF) check $(RUFF_FIX_FLAG) --config "$(RUFF_CONFIG)" --cache-dir "$(RUFF_CACHE_DIR)" $(LINT_TARGETS) 2>&1 | tee "$(LINT_ARTIFACTS_DIR)/ruff.log"; test $${PIPESTATUS[0]} -eq 0
	@if [ "$(ENABLE_MYPY)" != "1" ]; then \
	  echo "✖ Mypy must remain enabled for $(PROJECT_SLUG)" | tee "$(LINT_ARTIFACTS_DIR)/mypy.log"; \
	  exit 1; \
	fi
	@cd "$(MYPY_RUN_DIR)" && $(LINT_PYCACHE_ENV) $(MYPY) --config-file "$(MYPY_CONFIG_ABS)" $(MYPY_FLAGS) --cache-dir "$(MYPY_CACHE_DIR_ABS)" $(MYPY_TARGETS_ABS) 2>&1 | tee "$(LINT_ARTIFACTS_DIR)/mypy.log"; test $${PIPESTATUS[0]} -eq 0
	@if [ "$(ENABLE_CODESPELL)" = "1" ]; then \
	  $(CODESPELL) $(CODESPELL_TARGETS) 2>&1 | tee "$(LINT_ARTIFACTS_DIR)/codespell.log"; test $${PIPESTATUS[0]} -eq 0; \
	else \
	  echo "→ Skipping codespell" | tee "$(LINT_ARTIFACTS_DIR)/codespell.log"; \
	fi
	@if [ "$(ENABLE_RADON)" = "1" ]; then \
	  $(LINT_PYCACHE_ENV) $(RADON) cc -s -a $(RADON_TARGETS) 2>&1 | tee "$(LINT_ARTIFACTS_DIR)/radon.log"; test $${PIPESTATUS[0]} -eq 0; \
	  if [ -n "$(RADON_COMPLEXITY_MAX)" ]; then \
	    $(LINT_PYCACHE_ENV) $(RADON) cc -j $(RADON_TARGETS) | $(LINT_PYCACHE_ENV) $(VENV_PYTHON) -c 'import json, sys; payload=json.load(sys.stdin); max_score=int(sys.argv[1]); violations=[]; [violations.append((path, item.get("name"), item.get("complexity", 0))) for path, items in payload.items() for item in items if item.get("type") in {"function", "method"} and item.get("complexity", 0) > max_score]; print(f"Radon complexity threshold exceeded (>{max_score})") if violations else None; [print(f"{path}: {name} ({complexity})") for path, name, complexity in violations]; sys.exit(1 if violations else 0)' "$(RADON_COMPLEXITY_MAX)"; \
	  fi; \
	else \
	  echo "→ Skipping radon" | tee "$(LINT_ARTIFACTS_DIR)/radon.log"; \
	fi
	@if [ "$(ENABLE_PYDOCSTYLE)" = "1" ]; then \
	  $(LINT_PYCACHE_ENV) $(PYDOCSTYLE) $(PYDOCSTYLE_ARGS) $(PYDOCSTYLE_TARGETS) 2>&1 | tee "$(LINT_ARTIFACTS_DIR)/pydocstyle.log"; test $${PIPESTATUS[0]} -eq 0; \
	else \
	  echo "→ Skipping pydocstyle" | tee "$(LINT_ARTIFACTS_DIR)/pydocstyle.log"; \
	fi
	@printf "OK\n" > "$(LINT_ARTIFACTS_DIR)/_passed"

lint-file:
ifndef file
	$(error Usage: make lint-file file=path/to/file.py)
endif
	@$(MAKE) LINT_SCOPE="$(file)" lint-artifacts

lint-dir:
ifndef dir
	$(error Usage: make lint-dir dir=<directory_path>)
endif
	@$(MAKE) LINT_SCOPE="$(dir)" lint-artifacts

mypy-core:
	@if [ -n "$(MYPY_CORE_CONFIG)" ]; then \
	  cd "$(MYPY_RUN_DIR)" && $(LINT_PYCACHE_ENV) $(MYPY) --config-file "$(MYPY_CORE_CONFIG_ABS)" $(MYPY_CORE_FLAGS) --cache-dir "$(MYPY_CACHE_DIR_ABS)" $(MYPY_CORE_TARGETS_ABS); \
	else \
	  echo "→ mypy-core is not configured for $(PROJECT_SLUG)"; \
	fi

mypy-extended:
	@if [ -n "$(MYPY_EXTENDED_CONFIG)" ]; then \
	  cd "$(MYPY_RUN_DIR)" && $(LINT_PYCACHE_ENV) $(MYPY) --config-file "$(MYPY_EXTENDED_CONFIG_ABS)" $(MYPY_EXTENDED_FLAGS) --cache-dir "$(MYPY_CACHE_DIR_ABS)" $(MYPY_EXTENDED_TARGETS_ABS); \
	else \
	  echo "→ mypy-extended is not configured for $(PROJECT_SLUG)"; \
	fi

lint-clean:
	@echo "→ Cleaning lint artifacts"
	@rm -rf "$(LINT_ARTIFACTS_DIR)" || true
	@echo "✔ done"

##@ Lint
fmt: ## Apply code formatting and write logs under $(LINT_ARTIFACTS_DIR)
lint: ## Run lint checks and write logs under $(LINT_ARTIFACTS_DIR)
lint-artifacts: ## Same as 'lint' (explicit), generates logs
lint-file: ## Lint a single file (requires file=<path>)
lint-dir: ## Lint a directory (requires dir=<path>)
lint-clean: ## Remove lint artifacts, including caches
mypy-core: ## Run the core mypy configuration when configured for this package
mypy-extended: ## Run the extended mypy configuration when configured for this package
