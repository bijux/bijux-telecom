TEST_PATHS                ?= tests
TEST_PATHS_UNIT           ?= tests/unit
TEST_PATHS_E2E            ?=
TEST_PATHS_REGRESSION     ?=
TEST_PATHS_EVALUATION     ?=
TEST_REAL_LOCAL_PATH      ?=

TEST_ARTIFACTS_DIR        ?= $(PROJECT_ARTIFACTS_DIR)/test
JUNIT_XML                 ?= $(TEST_ARTIFACTS_DIR)/junit.xml
TMP_DIR                   ?= $(TEST_ARTIFACTS_DIR)/tmp
HYPOTHESIS_DB_DIR         ?= $(TEST_ARTIFACTS_DIR)/hypothesis
BENCHMARK_DIR             ?= $(TEST_ARTIFACTS_DIR)/benchmarks
COV_XML                   ?= $(TEST_ARTIFACTS_DIR)/coverage.xml

ENABLE_BENCH              ?= 1
PYTEST_ADDOPTS_EXTRA      ?=
TEST_PRE_TARGETS          ?=
TEST_MAIN_ARGS            ?=
TEST_UNIT_DIR_ARGS        ?= -m "not slow" --maxfail=1 -q
TEST_UNIT_FALLBACK_ARGS   ?= -k "not e2e and not integration and not functional" -m "not slow" --maxfail=1 -q
TEST_E2E_ARGS             ?= -m "e2e" --maxfail=1 -q
TEST_REGRESSION_ARGS      ?= -m "regression" --maxfail=1 -q
TEST_EVALUATION_ARGS      ?= -m "evaluation" --maxfail=1 -q
TEST_REAL_LOCAL_ARGS      ?= -m "real_local" -s -p no:cov
TEST_CI_TARGETS           ?= test-unit
TEST_RESET_PYCACHE        ?= 0
TEST_SYNTAX_PATHS         ?=
TEST_PYCACHE_PREFIX       ?= $(TEST_ARTIFACTS_DIR)/pycache
TEST_COVERAGE_TARGETS     ?=
TEST_COVERAGE_FAIL_UNDER  ?= 90
TEST_SOURCE_PATH          ?= src
TEST_SOURCE_PATHS         ?= $(TEST_SOURCE_PATH)
TEST_COVERAGE_SOURCE      ?= $(TEST_SOURCE_PATHS)
TEST_CLEAN_PATHS          ?=

TEST_PYTHON               ?= $(if $(wildcard $(VENV_PYTHON)),$(abspath $(VENV_PYTHON)),$(if $(wildcard $(VENV)/bin/python),$(abspath $(VENV)/bin/python),$(PYTHON)))
PYTEST                    ?= $(TEST_PYTHON) -m pytest
PYTEST_CONFIG             ?= $(MONOREPO_ROOT)/configs/pytest.ini
COVERAGE_CONFIG           ?= $(MONOREPO_ROOT)/configs/coveragerc.ini
PYTEST_ROOTDIR            ?= $(MONOREPO_ROOT)
TEST_SELF_MAKE            ?= $(SELF_MAKE)

include $(abspath $(dir $(lastword $(MAKEFILE_LIST))))/util.mk

define run_test_syntax
	@if [ -n "$(TEST_SYNTAX_PATHS)" ]; then \
	  echo "→ Running syntax-only checks (compileall $(TEST_SYNTAX_PATHS))"; \
	  if [ -n "$(strip $(TEST_PYCACHE_PREFIX))" ]; then mkdir -p "$(TEST_PYCACHE_PREFIX_ABS)"; fi; \
	  PYTHONDONTWRITEBYTECODE=1 $(TEST_PYCACHE_ENV) \
	    "$(TEST_PYTHON)" -m compileall $(TEST_SYNTAX_PATHS); \
	else \
	  echo "→ No syntax-only checks configured"; \
	fi
endef

empty :=
space := $(empty) $(empty)

PROJECT_DIR_ABS           := $(abspath $(PROJECT_DIR))
MONOREPO_ROOT_ABS         := $(abspath $(MONOREPO_ROOT))
PYTEST_ROOTDIR_ABS        := $(abspath $(PYTEST_ROOTDIR))
PYTEST_INI_ABS            := $(abspath $(PYTEST_CONFIG))
COVCFG_ABS                := $(abspath $(COVERAGE_CONFIG))
COV_HTML_ABS              := $(abspath $(TEST_ARTIFACTS_DIR)/htmlcov)
CACHE_DIR_ABS             := $(abspath $(TEST_ARTIFACTS_DIR)/.pytest_cache)
COV_XML_ABS               := $(abspath $(COV_XML))
COV_DATA_ABS              := $(abspath $(TEST_ARTIFACTS_DIR)/.coverage)
TEST_PATHS_ABS            := $(abspath $(TEST_PATHS))
TEST_PATHS_UNIT_ABS       := $(abspath $(TEST_PATHS_UNIT))
TEST_PATHS_E2E_ABS        := $(abspath $(TEST_PATHS_E2E))
TEST_PATHS_REGRESSION_ABS := $(abspath $(TEST_PATHS_REGRESSION))
TEST_PATHS_EVALUATION_ABS := $(abspath $(TEST_PATHS_EVALUATION))
TEST_REAL_LOCAL_ABS       := $(abspath $(TEST_REAL_LOCAL_PATH))
TEST_SOURCE_PATH_ABS      := $(subst $(space),:,$(foreach path,$(TEST_SOURCE_PATHS),$(abspath $(path))))
TEST_COVERAGE_SOURCE_ABS  := $(foreach path,$(TEST_COVERAGE_SOURCE),$(abspath $(path)))
JUNIT_XML_ABS             := $(abspath $(JUNIT_XML))
TMP_DIR_ABS               := $(abspath $(TMP_DIR))
HYPOTHESIS_DB_ABS         := $(abspath $(HYPOTHESIS_DB_DIR))
BENCHMARK_DIR_ABS         := $(abspath $(BENCHMARK_DIR))
TEST_PYCACHE_PREFIX_ABS   := $(abspath $(TEST_PYCACHE_PREFIX))
TEST_PYCACHE_ENV          := $(if $(strip $(TEST_PYCACHE_PREFIX)),PYTHONPYCACHEPREFIX="$(TEST_PYCACHE_PREFIX_ABS)",)
pytest_abs_path           = $(abspath $(if $(filter /%,$(strip $(1))),$(strip $(1)),$(PROJECT_DIR_ABS)/$(strip $(1))))
to_pytest_path            = $(if $(strip $(1)),$(patsubst $(PYTEST_ROOTDIR_ABS)/%,%,$(call pytest_abs_path,$(1))),)
TEST_PATH_ARGS            := $(foreach path,$(TEST_PATHS),"$(call to_pytest_path,$(path))")
TEST_PATHS_UNIT_ARG       := $(call to_pytest_path,$(TEST_PATHS_UNIT))
TEST_PATHS_E2E_ARG        := $(call to_pytest_path,$(TEST_PATHS_E2E))
TEST_PATHS_REGRESSION_ARG := $(call to_pytest_path,$(TEST_PATHS_REGRESSION))
TEST_PATHS_EVALUATION_ARG := $(call to_pytest_path,$(TEST_PATHS_EVALUATION))
TEST_REAL_LOCAL_ARG       := $(call to_pytest_path,$(TEST_REAL_LOCAL_PATH))
TEST_COVERAGE_TARGET_ARGS := $(foreach path,$(TEST_COVERAGE_TARGETS),"$(call to_pytest_path,$(path))")

PYTEST_COV_FLAGS = $(foreach path,$(TEST_COVERAGE_SOURCE_ABS),--cov="$(path)")

PYTEST_FLAGS = \
  --junitxml "$(JUNIT_XML_ABS)" \
  --basetemp "$(TMP_DIR_ABS)" \
  $(PYTEST_COV_FLAGS) \
  --cov-config "$(COVCFG_ABS)" \
  --cov-report=html:"$(COV_HTML_ABS)" \
  --cov-report=xml:"$(COV_XML_ABS)" \
  -o cache_dir="$(CACHE_DIR_ABS)" \
  $(PYTEST_ADDOPTS_EXTRA)
PYTEST_INFO_FLAGS = -o cache_dir="$(CACHE_DIR_ABS)"

.PHONY: test test-unit test-e2e test-regression test-evaluation test-ci test-clean test-syntax coverage-core real-local

test:
	@echo "→ Running full test suite on $(TEST_PATHS)"
	$(call run_make_targets,$(TEST_PRE_TARGETS),$(TEST_SELF_MAKE))
	$(call run_test_syntax)
	@if [ "$(TEST_RESET_PYCACHE)" = "1" ]; then find . -type d -name '__pycache__' -exec rm -rf {} + >/dev/null 2>&1 || true; fi
	@rm -rf "$(TMP_DIR_ABS)"
	@mkdir -p "$(TEST_ARTIFACTS_DIR)" "$(HYPOTHESIS_DB_DIR)" "$(BENCHMARK_DIR)" "$(TMP_DIR)" "$(COV_HTML_ABS)"
	$(call clean_paths,$(TEST_CLEAN_PATHS))
	@echo "   • JUnit XML → $(JUNIT_XML_ABS)"
	@echo "   • Hypothesis DB → $(HYPOTHESIS_DB_ABS)"
	@echo "   • Using pytest → $(PYTEST)"
	@BENCH_FLAGS=""; \
	if [ "$(ENABLE_BENCH)" = "1" ] && sh -c '$(PYTEST) $(PYTEST_INFO_FLAGS) -q --help' 2>/dev/null | grep -q -- '--benchmark-storage'; then \
	  BENCH_FLAGS="--benchmark-autosave --benchmark-storage=file://$(BENCHMARK_DIR_ABS)"; \
	  echo "   • pytest-benchmark detected → storing in $(BENCHMARK_DIR_ABS)"; \
	else \
	  echo "   • pytest-benchmark disabled or not installed"; \
	fi; \
	( cd "$(PYTEST_ROOTDIR_ABS)" && \
	  PYTHONPATH="$(TEST_SOURCE_PATH_ABS)$${PYTHONPATH:+:$${PYTHONPATH}}" \
	  PYTHONDONTWRITEBYTECODE=1 \
	  COVERAGE_FILE="$(COV_DATA_ABS)" \
	  HYPOTHESIS_DATABASE_DIRECTORY="$(HYPOTHESIS_DB_ABS)" \
	  $(TEST_PYCACHE_ENV) \
	  sh -c '$(PYTEST) --rootdir "$(PYTEST_ROOTDIR_ABS)" -c "$(PYTEST_INI_ABS)" $(TEST_PATH_ARGS) $(TEST_MAIN_ARGS) $(PYTEST_FLAGS) '"$$BENCH_FLAGS" )
	$(call clean_paths,$(TEST_CLEAN_PATHS))

test-unit:
	@echo "→ Running unit tests only"
	$(call run_make_targets,$(TEST_PRE_TARGETS),$(TEST_SELF_MAKE))
	@$(PYTEST) $(PYTEST_INFO_FLAGS) --version
	@echo "pytest cmd: $(PYTEST) -c '$(PYTEST_INI_ABS)' …"
	@rm -rf "$(TMP_DIR_ABS)"
	@mkdir -p "$(TEST_ARTIFACTS_DIR)" "$(HYPOTHESIS_DB_DIR)" "$(BENCHMARK_DIR)" "$(TMP_DIR)" "$(COV_HTML_ABS)"
	$(call clean_paths,$(TEST_CLEAN_PATHS))
	@echo "   • JUnit XML → $(JUNIT_XML_ABS)"
	@echo "   • Hypothesis DB → $(HYPOTHESIS_DB_ABS)"
	@echo "   • Using pytest → $(PYTEST)"
	@BENCH_FLAGS=""; \
	if [ "$(ENABLE_BENCH)" = "1" ] && sh -c '$(PYTEST) $(PYTEST_INFO_FLAGS) -q --help' 2>/dev/null | grep -q -- '--benchmark-storage'; then \
	  BENCH_FLAGS="--benchmark-autosave --benchmark-storage=file://$(BENCHMARK_DIR_ABS)"; \
	  echo "   • pytest-benchmark detected → storing in $(BENCHMARK_DIR_ABS)"; \
	else \
	  echo "   • pytest-benchmark disabled or not installed"; \
	fi; \
	if [ -d "$(TEST_PATHS_UNIT)" ] && find "$(TEST_PATHS_UNIT)" -type f -name 'test_*.py' | grep -q .; then \
	  echo "   • detected $(TEST_PATHS_UNIT) — targeting that directory"; \
	  ( cd "$(PYTEST_ROOTDIR_ABS)" && \
	    PYTHONPATH="$(TEST_SOURCE_PATH_ABS)$${PYTHONPATH:+:$${PYTHONPATH}}" \
	    PYTHONDONTWRITEBYTECODE=1 \
	    COVERAGE_FILE="$(COV_DATA_ABS)" \
	    HYPOTHESIS_DATABASE_DIRECTORY="$(HYPOTHESIS_DB_ABS)" \
	    $(TEST_PYCACHE_ENV) \
	    sh -c '$(PYTEST) --rootdir "$(PYTEST_ROOTDIR_ABS)" -c "$(PYTEST_INI_ABS)" "$(TEST_PATHS_UNIT_ARG)" $(TEST_UNIT_DIR_ARGS) $(PYTEST_FLAGS) '"$$BENCH_FLAGS" ); \
	else \
	  echo "   • no $(TEST_PATHS_UNIT); falling back to filtered suite"; \
	  ( cd "$(PYTEST_ROOTDIR_ABS)" && \
	    PYTHONPATH="$(TEST_SOURCE_PATH_ABS)$${PYTHONPATH:+:$${PYTHONPATH}}" \
	    PYTHONDONTWRITEBYTECODE=1 \
	    COVERAGE_FILE="$(COV_DATA_ABS)" \
	    HYPOTHESIS_DATABASE_DIRECTORY="$(HYPOTHESIS_DB_ABS)" \
	    $(TEST_PYCACHE_ENV) \
	    sh -c '$(PYTEST) --rootdir "$(PYTEST_ROOTDIR_ABS)" -c "$(PYTEST_INI_ABS)" $(TEST_PATH_ARGS) $(TEST_UNIT_FALLBACK_ARGS) $(PYTEST_FLAGS) '"$$BENCH_FLAGS" ); \
	fi
	$(call clean_paths,$(TEST_CLEAN_PATHS))

test-e2e:
	@echo "→ Running end-to-end tests only"
	$(call run_make_targets,$(TEST_PRE_TARGETS),$(TEST_SELF_MAKE))
	@$(PYTEST) $(PYTEST_INFO_FLAGS) --version
	@rm -rf "$(TMP_DIR_ABS)"
	@mkdir -p "$(TEST_ARTIFACTS_DIR)" "$(HYPOTHESIS_DB_DIR)" "$(BENCHMARK_DIR)" "$(TMP_DIR)" "$(COV_HTML_ABS)"
	$(call clean_paths,$(TEST_CLEAN_PATHS))
	@if [ -n "$(TEST_PATHS_E2E)" ] && [ -d "$(TEST_PATHS_E2E)" ] && find "$(TEST_PATHS_E2E)" -type f -name 'test_*.py' | grep -q .; then \
	  ( cd "$(PYTEST_ROOTDIR_ABS)" && \
	    PYTHONPATH="$(TEST_SOURCE_PATH_ABS)$${PYTHONPATH:+:$${PYTHONPATH}}" \
	    PYTHONDONTWRITEBYTECODE=1 \
	    COVERAGE_FILE="$(COV_DATA_ABS)" \
	    HYPOTHESIS_DATABASE_DIRECTORY="$(HYPOTHESIS_DB_ABS)" \
	    $(TEST_PYCACHE_ENV) \
	    sh -c '$(PYTEST) --rootdir "$(PYTEST_ROOTDIR_ABS)" -c "$(PYTEST_INI_ABS)" "$(TEST_PATHS_E2E_ARG)" $(TEST_E2E_ARGS) $(PYTEST_FLAGS)' ); \
	else \
	  echo "   • no $(TEST_PATHS_E2E); skipping"; \
	fi
	$(call clean_paths,$(TEST_CLEAN_PATHS))

test-regression:
	@echo "→ Running regression tests only"
	$(call run_make_targets,$(TEST_PRE_TARGETS),$(TEST_SELF_MAKE))
	@$(PYTEST) $(PYTEST_INFO_FLAGS) --version
	@rm -rf "$(TMP_DIR_ABS)"
	@mkdir -p "$(TEST_ARTIFACTS_DIR)" "$(HYPOTHESIS_DB_DIR)" "$(BENCHMARK_DIR)" "$(TMP_DIR)" "$(COV_HTML_ABS)"
	@rm -rf $(TEST_CLEAN_PATHS) || true
	@if [ -n "$(TEST_PATHS_REGRESSION)" ] && [ -d "$(TEST_PATHS_REGRESSION)" ] && find "$(TEST_PATHS_REGRESSION)" -type f -name 'test_*.py' | grep -q .; then \
	  ( cd "$(PYTEST_ROOTDIR_ABS)" && \
	    PYTHONPATH="$(TEST_SOURCE_PATH_ABS)$${PYTHONPATH:+:$${PYTHONPATH}}" \
	    PYTHONDONTWRITEBYTECODE=1 \
	    COVERAGE_FILE="$(COV_DATA_ABS)" \
	    HYPOTHESIS_DATABASE_DIRECTORY="$(HYPOTHESIS_DB_ABS)" \
	    $(TEST_PYCACHE_ENV) \
	    sh -c '$(PYTEST) --rootdir "$(PYTEST_ROOTDIR_ABS)" -c "$(PYTEST_INI_ABS)" "$(TEST_PATHS_REGRESSION_ARG)" $(TEST_REGRESSION_ARGS) $(PYTEST_FLAGS)' ); \
	else \
	  echo "   • no $(TEST_PATHS_REGRESSION); skipping"; \
	fi
	@rm -rf $(TEST_CLEAN_PATHS) || true

test-evaluation:
	@echo "→ Running evaluation tests only"
	$(call run_make_targets,$(TEST_PRE_TARGETS),$(TEST_SELF_MAKE))
	@$(PYTEST) $(PYTEST_INFO_FLAGS) --version
	@rm -rf "$(TMP_DIR_ABS)"
	@mkdir -p "$(TEST_ARTIFACTS_DIR)" "$(HYPOTHESIS_DB_DIR)" "$(BENCHMARK_DIR)" "$(TMP_DIR)" "$(COV_HTML_ABS)"
	@rm -rf $(TEST_CLEAN_PATHS) || true
	@if [ -n "$(TEST_PATHS_EVALUATION)" ] && [ -d "$(TEST_PATHS_EVALUATION)" ] && find "$(TEST_PATHS_EVALUATION)" -type f -name 'test_*.py' | grep -q .; then \
	  ( cd "$(PYTEST_ROOTDIR_ABS)" && \
	    PYTHONPATH="$(TEST_SOURCE_PATH_ABS)$${PYTHONPATH:+:$${PYTHONPATH}}" \
	    PYTHONDONTWRITEBYTECODE=1 \
	    COVERAGE_FILE="$(COV_DATA_ABS)" \
	    HYPOTHESIS_DATABASE_DIRECTORY="$(HYPOTHESIS_DB_ABS)" \
	    $(TEST_PYCACHE_ENV) \
	    sh -c '$(PYTEST) --rootdir "$(PYTEST_ROOTDIR_ABS)" -c "$(PYTEST_INI_ABS)" "$(TEST_PATHS_EVALUATION_ARG)" $(TEST_EVALUATION_ARGS) $(PYTEST_FLAGS)' ); \
	else \
	  echo "   • no $(TEST_PATHS_EVALUATION); skipping"; \
	fi
	@rm -rf $(TEST_CLEAN_PATHS) || true

test-ci: $(TEST_CI_TARGETS)
	@echo "✔ CI test categories completed"

test-clean:
	@echo "→ Cleaning test artifacts"
	@rm -rf $(TEST_CLEAN_PATHS) || true
	@$(RM) "$(TEST_ARTIFACTS_DIR)" .coverage .coverage.* || true
	@echo "✔ done"

test-syntax:
	$(call run_test_syntax)

coverage-core:
	@if [ -z "$(TEST_COVERAGE_TARGETS)" ]; then \
	  echo "→ coverage-core is not configured for $(PROJECT_SLUG)"; \
	  exit 0; \
	fi
	@echo "→ Coverage focus run (fail-under=$(TEST_COVERAGE_FAIL_UNDER)%)"
	@rm -rf "$(TMP_DIR_ABS)" .coverage .coverage.*
	@mkdir -p "$(TEST_ARTIFACTS_DIR)" "$(HYPOTHESIS_DB_DIR)" "$(BENCHMARK_DIR)" "$(TMP_DIR)"
	@( cd "$(PYTEST_ROOTDIR_ABS)" && \
	  PYTHONPATH="$(TEST_SOURCE_PATH_ABS)$${PYTHONPATH:+:$${PYTHONPATH}}" \
	  PYTHONDONTWRITEBYTECODE=1 \
	  COVERAGE_FILE="$(COV_DATA_ABS)" \
	  HYPOTHESIS_DATABASE_DIRECTORY="$(HYPOTHESIS_DB_ABS)" \
	  $(TEST_PYCACHE_ENV) \
	  $(PYTEST) --rootdir "$(PYTEST_ROOTDIR_ABS)" -c "$(PYTEST_INI_ABS)" $(TEST_COVERAGE_TARGET_ARGS) --cov="$(TEST_SOURCE_PATH_ABS)" --cov-report=term-missing --cov-fail-under=$(TEST_COVERAGE_FAIL_UNDER) )

real-local:
	@if [ -z "$(TEST_REAL_LOCAL_PATH)" ] || [ ! -d "$(TEST_REAL_LOCAL_PATH)" ]; then \
	  echo "→ real-local is not configured for $(PROJECT_SLUG)"; \
	  exit 0; \
	fi
	@echo "→ Running real local model tests (manual only)"
	$(call run_make_targets,$(TEST_PRE_TARGETS),$(TEST_SELF_MAKE))
	@$(PYTEST) $(PYTEST_INFO_FLAGS) --version
	@( cd "$(PYTEST_ROOTDIR_ABS)" && \
	  $(PYTEST) --rootdir "$(PYTEST_ROOTDIR_ABS)" -c "$(PYTEST_INI_ABS)" -o addopts= "$(TEST_REAL_LOCAL_ARG)" $(TEST_REAL_LOCAL_ARGS) )

##@ Test
test:            ## Run the full test suite with artifacts under $(PROJECT_ARTIFACTS_DIR)/test
test-unit:       ## Run the unit-focused suite or fall back to a filtered test run
test-e2e:        ## Run end-to-end tests when the package defines an e2e suite
test-regression: ## Run regression tests when the package defines a regression suite
test-evaluation: ## Run evaluation tests when the package defines an evaluation suite
test-ci:         ## Run the package-defined CI test target set
test-clean:      ## Remove test artifacts and stray benchmark / hypothesis state
test-syntax:     ## Run compileall-based syntax checks when configured
coverage-core:   ## Run the package-defined focused coverage target set
real-local:      ## Run package-defined local-only tests outside CI
