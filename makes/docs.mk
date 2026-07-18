# Documentation commands for the GNSS project site.

UV_BIN ?= $(shell command -v uv 2>/dev/null)
PYTHON_BIN ?= $(shell command -v python3 2>/dev/null)
MKDOCS_BIN = $(shell test -x "$(DOCS_VENV_DIR)/bin/mkdocs" && printf "%s" "$(DOCS_VENV_DIR)/bin/mkdocs" || command -v mkdocs 2>/dev/null)
DOCS_PYTHON_BIN = $(shell test -x "$(DOCS_VENV_DIR)/bin/python" && printf "%s" "$(DOCS_VENV_DIR)/bin/python" || printf "%s" "$(PYTHON_BIN)")
DOCS_REQUIREMENTS ?= configs/docs/requirements-docs.txt
MKDOCS_CFG ?= mkdocs.yml
DOCS_SITE_DIR ?= artifacts/docs/site
DOCS_CACHE_DIR ?= artifacts/docs/.cache
DOCS_VENV_DIR ?= artifacts/docs/venv
DOCS_PYCACHE_DIR ?= artifacts/docs/pycache
DOCS_HOST ?= 127.0.0.1
DOCS_PORT ?= 8000
SITE_URL ?= https://bijux.io/bijux-gnss/
TABLE_GUARD ?= .bijux/shared/bijux-docs/tooling/quality/markdown_table_guard.py
SITE_ICONS_DIR ?= docs/assets/site-icons
ROOT_COMPAT_ICONS ?= favicon.ico apple-touch-icon.png apple-touch-icon-precomposed.png

ifeq ($(strip $(UV_BIN)),)
DOCS_RUN = XDG_CACHE_HOME="$(DOCS_CACHE_DIR)" "$(MKDOCS_BIN)"
DOCS_PYTHON_RUN = PYTHONPYCACHEPREFIX="$(abspath $(DOCS_PYCACHE_DIR))" "$(DOCS_PYTHON_BIN)"
else
DOCS_RUN = XDG_CACHE_HOME="$(DOCS_CACHE_DIR)" UV_PROJECT_ENVIRONMENT="$(DOCS_VENV_DIR)" "$(UV_BIN)" run --with-requirements "$(DOCS_REQUIREMENTS)" mkdocs
DOCS_PYTHON_RUN = XDG_CACHE_HOME="$(DOCS_CACHE_DIR)" PYTHONPYCACHEPREFIX="$(abspath $(DOCS_PYCACHE_DIR))" UV_PROJECT_ENVIRONMENT="$(DOCS_VENV_DIR)" "$(UV_BIN)" run --with-requirements "$(DOCS_REQUIREMENTS)" python
endif

.PHONY: docs docs-check docs-clean docs-hygiene docs-install docs-render-check docs-require docs-sanity docs-serve docs-sync-root-icons gh-docs-build gh-docs-install gh-docs-verify

##@ Documentation
docs-require: ## Verify documentation build inputs and tools
	@test -f "$(MKDOCS_CFG)" || (echo "ERROR: missing $(MKDOCS_CFG)" && exit 1)
	@test -f "$(DOCS_REQUIREMENTS)" || (echo "ERROR: missing $(DOCS_REQUIREMENTS)" && exit 1)
	@test -n "$(PYTHON_BIN)" || (echo "ERROR: install python3 to validate documentation" && exit 1)
	@test -n "$(UV_BIN)$(MKDOCS_BIN)" || (echo "ERROR: install uv or run 'make docs-install'" && exit 1)
	@test -f "$(TABLE_GUARD)" || (echo "ERROR: missing $(TABLE_GUARD)" && exit 1)
	@test -f "$(BIJUX_DOCS_SYNC_SCRIPT)" || (echo "ERROR: missing $(BIJUX_DOCS_SYNC_SCRIPT)" && exit 1)
	@test -f "$(BIJUX_DOCS_SOT_GUARD)" || (echo "ERROR: missing $(BIJUX_DOCS_SOT_GUARD)" && exit 1)
	@test -f "$(BIJUX_DOCS_CONTRACT_GUARD)" || (echo "ERROR: missing $(BIJUX_DOCS_CONTRACT_GUARD)" && exit 1)
	@for icon in $(ROOT_COMPAT_ICONS); do \
		test -f "$(SITE_ICONS_DIR)/$$icon" || (echo "ERROR: missing $(SITE_ICONS_DIR)/$$icon" && exit 1); \
	done

docs-install: ## Install the pinned documentation toolchain under artifacts
	@test -n "$(PYTHON_BIN)" || (echo "ERROR: install python3 to provision documentation tools" && exit 1)
	@mkdir -p "$(dir $(DOCS_VENV_DIR))" "$(DOCS_CACHE_DIR)/pip"
	@"$(PYTHON_BIN)" -m venv "$(DOCS_VENV_DIR)"
	@PIP_CACHE_DIR="$(abspath $(DOCS_CACHE_DIR)/pip)" \
		"$(DOCS_VENV_DIR)/bin/python" -m pip install --disable-pip-version-check -r "$(DOCS_REQUIREMENTS)"

docs-sync-root-icons: ## Copy compatibility icons to the rendered site root
	@for icon in $(ROOT_COMPAT_ICONS); do \
		cp "$(DOCS_SITE_DIR)/assets/site-icons/$$icon" "$(DOCS_SITE_DIR)/$$icon"; \
	done

docs: docs-clean docs-require bijux-docs-sync ## Build documentation into artifacts/docs/site
	@echo "Building GNSS documentation"
	@mkdir -p "$(DOCS_CACHE_DIR)" "$(DOCS_VENV_DIR)" "$(DOCS_PYCACHE_DIR)"
	@SITE_URL="$(SITE_URL)" $(DOCS_RUN) build --strict --config-file "$(MKDOCS_CFG)" --site-dir "$(DOCS_SITE_DIR)"
	@$(MAKE) --no-print-directory docs-sync-root-icons
	@$(MAKE) --no-print-directory docs-hygiene
	@echo "GNSS documentation build complete"

docs-check: docs-require bijux-docs-sync ## Validate source, shell, tables, and the strict site build
	@$(MAKE) --no-print-directory bijux-docs-check
	@mkdir -p "$(DOCS_CACHE_DIR)" "$(DOCS_VENV_DIR)" "$(DOCS_PYCACHE_DIR)"
	@$(DOCS_PYTHON_RUN) "$(TABLE_GUARD)" docs
	@SITE_URL="$(SITE_URL)" $(DOCS_RUN) build --strict --quiet --config-file "$(MKDOCS_CFG)" --site-dir "$(DOCS_SITE_DIR)"
	@$(MAKE) --no-print-directory docs-sync-root-icons
	@$(MAKE) --no-print-directory docs-render-check
	@$(MAKE) --no-print-directory docs-hygiene
	@echo "GNSS documentation checks passed"

docs-render-check: ## Verify rendered shell, navigation, and compatibility assets
	@test -f "$(DOCS_SITE_DIR)/index.html" || (echo "ERROR: rendered documentation index is missing" && exit 1)
	@grep -q 'bijux-hub-strip' "$(DOCS_SITE_DIR)/index.html" || (echo "ERROR: shared Bijux hub strip is missing" && exit 1)
	@grep -q 'bijux-site-tabs' "$(DOCS_SITE_DIR)/index.html" || (echo "ERROR: GNSS site tabs are missing" && exit 1)
	@grep -q 'data-bijux-active-repository="bijux-gnss"' "$(DOCS_SITE_DIR)/index.html" || (echo "ERROR: GNSS repository identity is missing" && exit 1)
	@for label in "GNSS Handbook" "Core Handbook" "Infrastructure Handbook" "Navigation Handbook" "Receiver Handbook" "Signal Handbook" "Maintainer Handbook"; do \
		grep -q "$$label" "$(DOCS_SITE_DIR)/index.html" || (echo "ERROR: rendered navigation is missing $$label" && exit 1); \
	done
	@for icon in $(ROOT_COMPAT_ICONS); do \
		test -f "$(DOCS_SITE_DIR)/$$icon" || (echo "ERROR: rendered root icon $$icon is missing" && exit 1); \
	done
	@if grep -R -i -q 'bijux-telecom' "$(DOCS_SITE_DIR)"; then \
		echo "ERROR: rendered documentation contains obsolete bijux-telecom identity"; \
		exit 1; \
	fi

docs-sanity: docs-check ## Run the complete documentation validation lane

docs-serve: docs-require bijux-docs-sync ## Serve documentation locally with automatic reloads
	@HOST=$${HOST:-$(DOCS_HOST)}; PORT=$${PORT:-$(DOCS_PORT)}; \
		if command -v lsof >/dev/null 2>&1; then \
			while lsof -tiTCP:$$PORT -sTCP:LISTEN >/dev/null 2>&1; do PORT=$$((PORT+1)); done; \
		fi; \
		echo "Serving GNSS documentation on http://$$HOST:$$PORT/"; \
		mkdir -p "$(DOCS_CACHE_DIR)" "$(DOCS_VENV_DIR)" "$(DOCS_PYCACHE_DIR)"; \
		SITE_URL="http://$$HOST:$$PORT/" $(DOCS_RUN) serve --config-file "$(MKDOCS_CFG)" --dev-addr $$HOST:$$PORT

docs-clean: ## Remove generated documentation site and caches
	@$(call safe_rm,$(DOCS_SITE_DIR))
	@$(call safe_rm,$(DOCS_CACHE_DIR))
	@$(call safe_rm,$(DOCS_PYCACHE_DIR))

docs-hygiene: ## Verify documentation output stays under artifacts
	@test ! -e site || (echo "ERROR: root site/ is forbidden" && exit 1)
	@test ! -e .cache || (echo "ERROR: root .cache/ is forbidden" && exit 1)
	@test ! -d docs/artifacts || (echo "ERROR: docs/artifacts/ is forbidden" && exit 1)

gh-docs-install: docs-install ## Install documentation tools for the Pages workflow
gh-docs-build: docs-check ## Build and validate the Pages artifact
gh-docs-verify: docs-render-check ## Verify the Pages artifact
