DOCS_RUN ?= mkdocs
DOCS_CONFIG ?= mkdocs.yml
DOCS_SITE_DIR ?= $(ARTIFACT_ROOT_ABS)/docs/site
DOCS_CHECK_SITE_DIR ?= $(ARTIFACT_ROOT_ABS)/docs/check-site
DOCS_CACHE_DIR ?= $(ARTIFACT_ROOT_ABS)/docs/cache
DOCS_PYCACHE_DIR ?= $(ARTIFACT_ROOT_ABS)/docs/pycache
DOCS_DEV_ADDR ?= 127.0.0.1:8000
DOCS_SITE_URL ?=
DOCS_BUILD_FLAGS ?= --strict
DOCS_CHECK_FLAGS ?= --strict --quiet
DOCS_SERVE_FLAGS ?=
DOCS_PREPARE_TARGETS ?=
DOCS_SOURCE_CHECK_TARGETS ?=
DOCS_HYGIENE_FORBIDDEN_PATHS ?= site .cache

BIJUX_CI_PR_TARGETS += docs-check
BIJUX_CI_DOCS_TARGETS += docs-check
BIJUX_HELP_TARGETS += docs docs-check docs-clean docs-hygiene docs-require docs-serve
BIJUX_HELP_docs := Build strict documentation under artifacts
BIJUX_HELP_docs-check := Validate a strict documentation build
BIJUX_HELP_docs-clean := Remove documentation artifacts
BIJUX_HELP_docs-hygiene := Reject documentation output outside artifacts
BIJUX_HELP_docs-require := Verify documentation inputs and tools
BIJUX_HELP_docs-serve := Serve documentation in the foreground

.PHONY: docs docs-check docs-clean docs-hygiene docs-require docs-serve

docs-require: ## Verify documentation inputs and tools
	@$(call require_file,$(DOCS_CONFIG))
	@$(call require_tool,$(firstword $(DOCS_RUN)))

docs: docs-require $(DOCS_PREPARE_TARGETS) ## Build strict documentation under artifacts
	@$(call safe_remove,$(DOCS_SITE_DIR))
	@mkdir -p "$(DOCS_SITE_DIR)" "$(DOCS_CACHE_DIR)" "$(DOCS_PYCACHE_DIR)"
	@XDG_CACHE_HOME="$(DOCS_CACHE_DIR)" \
		PYTHONPYCACHEPREFIX="$(DOCS_PYCACHE_DIR)" \
		SITE_URL="$(DOCS_SITE_URL)" \
		$(DOCS_RUN) build $(DOCS_BUILD_FLAGS) \
			--config-file "$(DOCS_CONFIG)" \
			--site-dir "$(DOCS_SITE_DIR)"
	@$(BIJUX_MAKE) docs-hygiene

docs-check: docs-require $(DOCS_PREPARE_TARGETS) $(DOCS_SOURCE_CHECK_TARGETS) ## Validate a strict documentation build
	@$(call safe_remove,$(DOCS_CHECK_SITE_DIR))
	@mkdir -p "$(DOCS_CHECK_SITE_DIR)" "$(DOCS_CACHE_DIR)" "$(DOCS_PYCACHE_DIR)"
	@XDG_CACHE_HOME="$(DOCS_CACHE_DIR)" \
		PYTHONPYCACHEPREFIX="$(DOCS_PYCACHE_DIR)" \
		SITE_URL="$(DOCS_SITE_URL)" \
		$(DOCS_RUN) build $(DOCS_CHECK_FLAGS) \
			--config-file "$(DOCS_CONFIG)" \
			--site-dir "$(DOCS_CHECK_SITE_DIR)"
	@$(BIJUX_MAKE) docs-hygiene

docs-serve: docs-require $(DOCS_PREPARE_TARGETS) ## Serve documentation in the foreground
	@mkdir -p "$(DOCS_CACHE_DIR)" "$(DOCS_PYCACHE_DIR)"
	@XDG_CACHE_HOME="$(DOCS_CACHE_DIR)" \
		PYTHONPYCACHEPREFIX="$(DOCS_PYCACHE_DIR)" \
		SITE_URL="http://$(DOCS_DEV_ADDR)/" \
		$(DOCS_RUN) serve $(DOCS_SERVE_FLAGS) \
			--config-file "$(DOCS_CONFIG)" \
			--dev-addr "$(DOCS_DEV_ADDR)"

docs-clean: ## Remove documentation artifacts
	@$(call safe_remove,$(ARTIFACT_ROOT_ABS)/docs)

docs-hygiene: ## Reject documentation output outside artifacts
	@for path in $(DOCS_HYGIENE_FORBIDDEN_PATHS); do \
		test ! -e "$(PROJECT_ROOT)/$$path" || { echo "forbidden documentation output: $$path" >&2; exit 1; }; \
	done
