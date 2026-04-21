API_SCHEMA                ?= $(API_DIR)/v1/schema.yaml
API_SERVER_LOG            ?= $(API_ARTIFACTS_DIR)/server.log
API_DRIFT_OUT             ?= $(API_ARTIFACTS_DIR)/openapi.generated.json
API_OPENAPI_DRIFT_COMMAND ?=
API_PYTHON_ENV            ?= $(CANON_DEV_PYTHON_ENV)
API_UVICORN               ?= $(ACT)/uvicorn
PRANCE                    ?= $(ACT)/prance
OPENAPI_SPEC_VALIDATOR    ?= $(ACT)/openapi-spec-validator
SCHEMATHESIS              ?= $(ACT)/schemathesis
API_SCHEMATHESIS_ARGS     ?= --workers=1 --max-failures=1 --checks=not_a_server_error,response_schema_conformance,content_type_conformance,response_headers_conformance --hypothesis-max-examples=5 --request-timeout=30000 --max-response-time=500 --hypothesis-suppress-health-check=filter_too_much
API_SERVER_IMPORT         ?=

.PHONY: api api-install api-lint api-test api-clean openapi-drift api-drift

api: api-lint openapi-drift api-test

api-install:
	@echo "→ API tooling is managed by the package install target"

api-lint:
	@if [ ! -f "$(API_SCHEMA)" ]; then echo "✘ Missing $(API_SCHEMA)"; exit 1; fi
	@mkdir -p "$(API_LINT_DIR_ABS)"
	@echo "→ Validating OpenAPI schema $(API_SCHEMA)"
	@$(PRANCE) validate "$(API_SCHEMA)" 2>&1 | tee "$(API_LINT_DIR_ABS)/prance.log"
	@$(OPENAPI_SPEC_VALIDATOR) "$(API_SCHEMA)" 2>&1 | tee "$(API_LINT_DIR_ABS)/spec-validator.log"
	@echo "✔ API lint complete"

api-test:
	@if [ ! -x "$(API_UVICORN)" ]; then echo "uvicorn not found; install dev extras"; exit 1; fi
	@mkdir -p "$(API_ARTIFACTS_DIR_ABS)"
	@printf '%s\n' \
	  'import os, socket' \
	  'host = os.environ.get("API_HOST", "127.0.0.1")' \
	  'preferred = int(os.environ.get("API_PORT", "8000"))' \
	  'busy = False' \
	  'with socket.socket() as sock:' \
	  '    try:' \
	  '        sock.bind((host, preferred))' \
	  '        port = preferred' \
	  '    except OSError:' \
	  '        busy = True' \
	  '        sock.bind((host, 0))' \
	  '        port = sock.getsockname()[1]' \
	  'print(port)' \
	  'print(int(busy))' | API_HOST="$(API_HOST)" API_PORT="$(API_PORT)" "$(VENV_PYTHON)" - >"$(API_ARTIFACTS_DIR_ABS)/port.meta"
	@set -eu; \
	  PORT="$$(sed -n '1p' "$(API_ARTIFACTS_DIR_ABS)/port.meta")"; \
	  FALLBACK="$$(sed -n '2p' "$(API_ARTIFACTS_DIR_ABS)/port.meta")"; \
	  if [ "$$FALLBACK" -eq 1 ]; then echo "→ Port $(API_PORT) busy; using $$PORT"; fi; \
	  echo "$$PORT" >"$(API_ARTIFACTS_DIR_ABS)/port"; \
	  echo "→ Starting API server for schemathesis on $$PORT"; \
	  $(API_UVICORN) "$(API_SERVER_IMPORT)" --host $(API_HOST) --port $$PORT --factory >"$(API_SERVER_LOG)" 2>&1 & echo $$! >"$(API_ARTIFACTS_DIR_ABS)/server.pid"; \
	  sleep 2; \
	  echo "→ Running schemathesis against live server"
	@set -eu; \
	  BASE_FLAG=$$($(SCHEMATHESIS) run -h 2>&1 | grep -q " --url " && echo --url || echo --base-url); \
	  EXTRA_FLAG=""; \
	  HAS_EXPERIMENTAL=$$($(SCHEMATHESIS) run -h 2>&1 | grep -q " --experimental" && echo 1 || echo 0); \
	  if [ "$$HAS_EXPERIMENTAL" -eq 1 ]; then \
	    OPENAPI_VERSION=$$(PYTHONPATH=""; "$(VENV_PYTHON)" -c "import yaml; v=yaml.safe_load(open('$(API_SCHEMA)', 'r', encoding='utf-8')).get('openapi',''); print(v)"); \
	    case "$$OPENAPI_VERSION" in \
	      3.1*) EXTRA_FLAG="--experimental=openapi-3.1" ;; \
	    esac; \
	  fi; \
	  PORT="$$(cat "$(API_ARTIFACTS_DIR_ABS)/port")"; \
	  $(SCHEMATHESIS) run "$(API_SCHEMA)" $$BASE_FLAG "http://$(API_HOST):$$PORT" $$EXTRA_FLAG $(API_SCHEMATHESIS_ARGS) 2>&1 | tee "$(API_ARTIFACTS_DIR_ABS)/schemathesis.log"; \
	  RC=$$?; \
	  kill $$(cat "$(API_ARTIFACTS_DIR_ABS)/server.pid") >/dev/null 2>&1 || true; \
	  wait $$(cat "$(API_ARTIFACTS_DIR_ABS)/server.pid") >/dev/null 2>&1 || true; \
	  exit $$RC

openapi-drift:
	@mkdir -p "$(API_ARTIFACTS_DIR_ABS)"
	@echo "→ Checking OpenAPI drift"
	@$(API_PYTHON_ENV) $(API_OPENAPI_DRIFT_COMMAND)

api-drift: openapi-drift

api-clean:
	@rm -rf "$(API_ARTIFACTS_DIR_ABS)"

##@ API
api:         ## Lint the OpenAPI schema, check drift, and run schemathesis
api-install: ## Report how API tooling is provided for this package
api-lint:    ## Validate the checked-in OpenAPI schema
openapi-drift: ## Compare generated OpenAPI with the checked-in schema
api-test:    ## Start the local API server and run schemathesis
api-clean:   ## Remove API artifacts
