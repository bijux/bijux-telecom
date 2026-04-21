.DEFAULT_GOAL := help

.PHONY: \
	help list list-all install lock lock-check lint quality security test docs docs-check docs-serve api build sbom clean all \
	clean-root-artifacts root-check-env check-make-layout check-bijux-standard

ROOT_CHECK_VENV ?= $(ROOT_ARTIFACTS_DIR)/check-venv
ROOT_CHECK_PYTHON ?= $(ROOT_CHECK_VENV)/bin/python
ROOT_CHECK_STAMP ?= $(ROOT_ARTIFACTS_DIR)/.check-tools.stamp
ROOT_DOCS_ARTIFACTS_DIR ?= $(ROOT_ARTIFACTS_DIR)/docs
ROOT_DOCS_BUILD_SITE_DIR ?= $(ROOT_DOCS_ARTIFACTS_DIR)/build-site
ROOT_DOCS_CHECK_SITE_DIR ?= $(ROOT_DOCS_ARTIFACTS_DIR)/check-site
ROOT_DOCS_SERVE_SITE_DIR ?= $(ROOT_DOCS_ARTIFACTS_DIR)/serve-site
ROOT_DOCS_CACHE_DIR ?= $(ROOT_DOCS_ARTIFACTS_DIR)/cache
ROOT_DOCS_SERVE_CFG ?= $(ROOT_ARTIFACTS_DIR)/mkdocs.serve.yml
ROOT_DOCS_DEV_ADDR ?= 127.0.0.1:8000
UV_SYNC ?= UV_PROJECT_ENVIRONMENT="$(ROOT_CHECK_VENV)" $(UV) sync --frozen --group dev --python "$(PYTHON)"
ROOT_DEV_PYTHONPATH ?=
ROOT_CHECK_STAMP_SYNC_MESSAGE ?= @true

ifneq ($(strip $(ROOT_DEV_PYTHONPATH)),)
export PYTHONPATH := $(ROOT_DEV_PYTHONPATH)$(if $(PYTHONPATH),:$(PYTHONPATH))
endif

ROOT_FORBIDDEN_ARTIFACTS ?= \
	"$(CURDIR)/.hypothesis" \
	"$(CURDIR)/.pytest_cache" \
	"$(CURDIR)/.ruff_cache" \
	"$(CURDIR)/.mypy_cache" \
	"$(CURDIR)/.coverage" \
	"$(CURDIR)/.coverage."* \
	"$(CURDIR)/.benchmarks" \
	"$(CURDIR)/htmlcov" \
	"$(CURDIR)/configs/.pytest_cache" \
	"$(CURDIR)/configs/.ruff_cache" \
	"$(CURDIR)/configs/.mypy_cache" \
	"$(CURDIR)/configs/.hypothesis"

$(ROOT_CHECK_STAMP): pyproject.toml uv.lock
	@mkdir -p "$(ROOT_ARTIFACTS_DIR)"
	@rm -rf "$(ROOT_CHECK_VENV)"
	$(ROOT_CHECK_STAMP_SYNC_MESSAGE)
	@$(UV_SYNC)
	@touch "$(ROOT_CHECK_STAMP)"

list:
	@printf "%s\n" $(PRIMARY_PACKAGES)

list-all:
	@printf "%s\n" $(ALL_PACKAGES)

ROOT_INSTALL_PREREQS ?= root-check-env
ROOT_CHECK_ENV_PREREQS ?= pyproject.toml uv.lock $(ROOT_CHECK_STAMP)
ROOT_CLEAN_ROOT_ARTIFACTS_COMMAND ?= @rm -rf $(ROOT_FORBIDDEN_ARTIFACTS) || true
ROOT_ALL_TARGETS ?= test lint quality security docs api build sbom
ROOT_DEFINE_CLEAN ?= 0

include $(ROOT_MAKEFILE_DIR)/bijux-py/root/lifecycle.mk
