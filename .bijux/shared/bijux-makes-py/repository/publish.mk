PUBLISH_PYTHON ?= $(if $(wildcard $(VENV_PYTHON)),$(VENV_PYTHON),python3.11)
PUBLISH_DIST_DIR ?= $(PROJECT_ARTIFACTS_DIR)/build
PUBLISH_PACKAGE_NAME ?= $(PROJECT_SLUG)
PUBLISH_VERSION_RESOLVER ?=
PUBLISH_VERSION_GUARD ?=
PUBLISH_VERSION ?= $(strip $(if $(PUBLISH_VERSION_RESOLVER),$(shell $(PUBLISH_PYTHON) $(PUBLISH_VERSION_RESOLVER) --pyproject pyproject.toml --package-name "$(PUBLISH_PACKAGE_NAME)" 2>/dev/null || echo 0.0.0),0.0.0))
TWINE ?= $(PUBLISH_PYTHON) -m twine
TWINE_REPOSITORY ?= pypi
TWINE_USERNAME ?= __token__
TWINE_PASSWORD ?= $(PYPI_API_TOKEN)
SKIP_TWINE_CHECK ?= 0
SKIP_EXISTING ?= 1
PUBLISH_UPLOAD_ENABLED ?= 0
PUBLISH_TEST_ENABLED ?= $(PUBLISH_UPLOAD_ENABLED)
PUBLISH_VERIFY_INSTALL_CMD ?=
PUBLISH_ALLOW_PRERELEASE ?= 0
PUBLISH_ALLOW_LOCAL_VERSION ?= 0
PUBLISH_REQUIRE_VERSION_GUARD ?= $(if $(strip $(PUBLISH_VERSION_RESOLVER)$(PUBLISH_VERSION_GUARD)),1,0)

PUBLISH_VERSION_GUARD_FLAGS = $(if $(filter 1,$(PUBLISH_ALLOW_PRERELEASE)),--allow-prerelease,) \
	$(if $(filter 1,$(PUBLISH_ALLOW_LOCAL_VERSION)),--allow-local-version,)
PUBLISH_TEST_INSTALL_SPEC = $(if $(filter 1,$(PUBLISH_REQUIRE_VERSION_GUARD)),$(PUBLISH_PACKAGE_NAME)==$(PUBLISH_VERSION),$(PUBLISH_PACKAGE_NAME))

.PHONY: publish publish-test twine twine-check twine-upload twine-upload-test ensure-dists check-version verify-test-install

twine: publish

publish: check-version build twine-check
	@if [ "$(PUBLISH_UPLOAD_ENABLED)" = "1" ]; then \
	  $(MAKE) twine-upload; \
	  echo "✔ Published $(PUBLISH_PACKAGE_NAME) $(if $(filter 1,$(PUBLISH_REQUIRE_VERSION_GUARD)),$(PUBLISH_VERSION),) to $(TWINE_REPOSITORY)"; \
	else \
	  echo "→ Ready to upload from $(PUBLISH_DIST_DIR); run '$(TWINE) upload $(PUBLISH_DIST_DIR)/*' when credentials are configured."; \
	fi

publish-test: check-version build twine-check
	@if [ "$(PUBLISH_TEST_ENABLED)" = "1" ]; then \
	  $(MAKE) twine-upload-test; \
	  echo "✔ Published $(PUBLISH_PACKAGE_NAME) $(if $(filter 1,$(PUBLISH_REQUIRE_VERSION_GUARD)),$(PUBLISH_VERSION),) to testpypi"; \
	else \
	  echo "→ Ready to upload to TestPyPI from $(PUBLISH_DIST_DIR); run '$(TWINE) upload --repository testpypi $(PUBLISH_DIST_DIR)/*' when credentials are configured."; \
	fi

check-version: build-tools
ifeq ($(PUBLISH_REQUIRE_VERSION_GUARD),1)
	@echo "→ Package version: $(PUBLISH_VERSION)"
	@[ "$(PUBLISH_VERSION)" != "0.0.0" ] || { echo "✘ PUBLISH_VERSION resolved to 0.0.0"; exit 1; }
	@if [ -n "$(strip $(PUBLISH_VERSION_GUARD))" ]; then \
	  $(PUBLISH_PYTHON) $(PUBLISH_VERSION_GUARD) \
	    --pyproject pyproject.toml \
	    --package-name "$(PUBLISH_PACKAGE_NAME)" \
	    $(PUBLISH_VERSION_GUARD_FLAGS) >/dev/null; \
	fi
else
	@echo "→ No publication version guard configured; skipping version policy checks"
endif

ensure-dists:
	@echo "→ Verifying artifacts in '$(PUBLISH_DIST_DIR)'"
	@test -d "$(PUBLISH_DIST_DIR)" || { echo "✘ Dist dir missing: $(PUBLISH_DIST_DIR)"; exit 1; }
	@ls "$(PUBLISH_DIST_DIR)"/*.whl >/dev/null 2>&1 || { echo "✘ Missing wheel in $(PUBLISH_DIST_DIR)"; exit 1; }
	@ls "$(PUBLISH_DIST_DIR)"/*.tar.gz >/dev/null 2>&1 || { echo "✘ Missing sdist in $(PUBLISH_DIST_DIR)"; exit 1; }
	@if [ "$(PUBLISH_REQUIRE_VERSION_GUARD)" = "1" ] && [ -n "$(strip $(PUBLISH_VERSION_GUARD))" ]; then \
	  $(PUBLISH_PYTHON) $(PUBLISH_VERSION_GUARD) \
	    --pyproject pyproject.toml \
	    --package-name "$(PUBLISH_PACKAGE_NAME)" \
	    --dist-dir "$(PUBLISH_DIST_DIR)" \
	    $(PUBLISH_VERSION_GUARD_FLAGS) >/dev/null; \
	fi
	@ls -lh "$(PUBLISH_DIST_DIR)"/*.whl "$(PUBLISH_DIST_DIR)"/*.tar.gz

twine-check: ensure-dists
ifeq ($(SKIP_TWINE_CHECK),1)
	@echo "→ Skipping twine check (SKIP_TWINE_CHECK=$(SKIP_TWINE_CHECK))"
else
	@echo "→ Running twine check"
	@$(TWINE) check "$(PUBLISH_DIST_DIR)"/*
endif

twine-upload: ensure-dists
	@echo "→ Uploading $(PUBLISH_PACKAGE_NAME) $(if $(filter 1,$(PUBLISH_REQUIRE_VERSION_GUARD)),$(PUBLISH_VERSION),) to repository '$(TWINE_REPOSITORY)'"
	@test -n "$(TWINE_PASSWORD)" || { echo "✘ PYPI_API_TOKEN (TWINE_PASSWORD) not set"; exit 1; }
	@SKIP=""; [ "$(SKIP_EXISTING)" = "1" ] && SKIP="--skip-existing"; \
	$(TWINE) upload --non-interactive --disable-progress-bar $$SKIP \
	  --repository "$(TWINE_REPOSITORY)" -u "$(TWINE_USERNAME)" -p "$(TWINE_PASSWORD)" \
	  "$(PUBLISH_DIST_DIR)"/*

twine-upload-test:
	@$(MAKE) twine-upload TWINE_REPOSITORY=testpypi

verify-test-install:
	@if [ -z "$(PUBLISH_VERIFY_INSTALL_CMD)" ]; then \
	  echo "→ verify-test-install is not configured for $(PUBLISH_PACKAGE_NAME)"; \
	  exit 0; \
	fi
	@echo "→ Verifying installation from TestPyPI"
	@tmp=$$(mktemp -d); \
	$(UV) venv --python "$(PUBLISH_PYTHON)" "$$tmp/venv"; \
	$(UV) pip install --python "$$tmp/venv/bin/python" -i https://test.pypi.org/simple --extra-index-url https://pypi.org/simple "$(PUBLISH_TEST_INSTALL_SPEC)"; \
	PATH="$$tmp/venv/bin:$$PATH" VIRTUAL_ENV="$$tmp/venv" sh -c '$(PUBLISH_VERIFY_INSTALL_CMD)'; \
	echo "✔ TestPyPI install OK"; \
	echo "Temp venv at $$tmp (delete when done)"

##@ Publish
publish:             ## Build artifacts, validate with twine, and optionally upload
publish-test:        ## Build artifacts, validate with twine, and optionally upload to TestPyPI
verify-test-install: ## Install from TestPyPI into a temp venv and run the configured verification command
