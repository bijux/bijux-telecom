PRANCE ?= $(ACT)/prance
OPENAPI_SPEC_VALIDATOR ?= $(ACT)/openapi-spec-validator
ALL_API_SCHEMAS := $(shell if [ -d "$(API_DIR)" ]; then find "$(API_DIR)" -type f -path '*/v1/schema.yaml'; fi)
API_LINT_DIR_ABS := $(abspath $(API_LINT_DIR))
API_FREEZE_COMMAND ?=
API_OPENAPI_DRIFT_COMMAND ?=
API_INSTALL_PYTHON_PACKAGES ?= click prance openapi-spec-validator
API_NO_SCHEMA_MESSAGE ?= ✘ No OpenAPI schemas found under $(API_DIR)/*/v1/schema.yaml
API_PYTHON_ENV ?= $(CANON_DEV_PYTHON_ENV)

.PHONY: api api-install api-lint api-freeze openapi-drift api-clean api-test api-serve api-serve-bg api-stop

api: api-install api-lint api-freeze openapi-drift
	@echo "✔ API checks passed"

api-install: install
	@echo "→ API tooling is managed by the package install target"
	@$(UV) pip install --python "$(VENV_PYTHON)" --upgrade $(API_INSTALL_PYTHON_PACKAGES) >/dev/null
	@"$(PRANCE)" --version >/dev/null
	@"$(OPENAPI_SPEC_VALIDATOR)" --help >/dev/null

api-lint:
	@if [ -z "$(ALL_API_SCHEMAS)" ]; then echo "$(API_NO_SCHEMA_MESSAGE)"; exit 1; fi
	@mkdir -p "$(API_LINT_DIR_ABS)"
	@set -e; \
	for schema in $(ALL_API_SCHEMAS); do \
	  name="$$(echo "$$schema" | tr '/' '_')"; \
	  echo "→ Validating $$schema"; \
	  { \
	    $(PRANCE) validate "$$schema"; \
	    $(OPENAPI_SPEC_VALIDATOR) "$$schema"; \
	  } 2>&1 | tee "$(API_LINT_DIR_ABS)/$$name.log"; \
	done
	@echo "✔ API lint complete"

api-freeze:
	@echo "→ Enforcing API schema freeze contracts"
	@cd "$(MONOREPO_ROOT)" && $(API_PYTHON_ENV) $(API_FREEZE_COMMAND)
	@echo "✔ API freeze contracts validated"

openapi-drift:
	@if [ -z "$(strip $(API_OPENAPI_DRIFT_COMMAND))" ]; then \
	  echo "→ No live OpenAPI drift command configured; skipping"; \
	else \
	  echo "→ Checking OpenAPI drift"; \
	  cd "$(MONOREPO_ROOT)" && $(API_PYTHON_ENV) $(API_OPENAPI_DRIFT_COMMAND); \
	  echo "✔ OpenAPI drift check complete"; \
	fi

api-clean:
	@rm -rf "$(API_ARTIFACTS_DIR)" || true

api-test:
	@echo "→ API_MODE=freeze does not run live HTTP tests"

api-serve:
	@echo "→ API_MODE=freeze does not provide api-serve"

api-serve-bg:
	@echo "→ API_MODE=freeze does not provide api-serve-bg"

api-stop:
	@echo "→ API_MODE=freeze does not provide api-stop"

##@ API
api:            ## Validate and enforce frozen API contracts for checked-in schemas
api-install:    ## Validate API lint tooling in the active environment
api-lint:       ## Validate OpenAPI schemas under apis/*/v1
api-freeze:     ## Ensure pinned_openapi.json and schema.hash match schema.yaml
openapi-drift:  ## Detect breaking schema changes without version bumps when configured
api-clean:      ## Remove API artifacts
