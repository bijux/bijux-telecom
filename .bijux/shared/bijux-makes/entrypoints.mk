.PHONY: clean doctor
BIJUX_HELP_TARGETS += clean doctor
BIJUX_HELP_clean := Remove repository-generated artifacts
BIJUX_HELP_doctor := Validate the shared Make environment

clean: ## Remove repository-generated artifacts
	@$(call safe_remove,$(ARTIFACT_ROOT_ABS))

doctor: $(BIJUX_DOCTOR_TARGETS) ## Validate the shared Make environment
	@test -d "$(PROJECT_ROOT)" || { echo "project root is unavailable: $(PROJECT_ROOT)" >&2; exit 1; }
	@case "$(ARTIFACT_ROOT_ABS)" in \
		"$(BIJUX_ARTIFACT_BOUNDARY)"|"$(BIJUX_ARTIFACT_BOUNDARY)"/*) ;; \
		*) echo "artifact root must stay under $(BIJUX_ARTIFACT_BOUNDARY): $(ARTIFACT_ROOT_ABS)" >&2; exit 1 ;; \
	esac
	@printf '%s\n' \
		"project-root=$(PROJECT_ROOT)" \
		"artifact-root=$(ARTIFACT_ROOT_ABS)" \
		"components=$(strip $(BIJUX_MAKE_COMPONENTS))"

ifneq ($(strip $(BIJUX_FORMAT_TARGETS)),)
.PHONY: format
BIJUX_HELP_TARGETS += format
BIJUX_HELP_format := Apply configured source formatters
format: $(BIJUX_FORMAT_TARGETS) ## Apply configured source formatters
endif

ifneq ($(strip $(BIJUX_FMT_TARGETS)),)
.PHONY: fmt
BIJUX_HELP_TARGETS += fmt
BIJUX_HELP_fmt := Verify source formatting
fmt: $(BIJUX_FMT_TARGETS) ## Verify source formatting
endif

ifneq ($(strip $(BIJUX_LINT_TARGETS)),)
.PHONY: lint
BIJUX_HELP_TARGETS += lint
BIJUX_HELP_lint := Run configured lint checks
lint: $(BIJUX_LINT_TARGETS) ## Run configured lint checks
endif

ifneq ($(strip $(BIJUX_TEST_TARGETS)),)
.PHONY: test
BIJUX_HELP_TARGETS += test
BIJUX_HELP_test := Run configured fast tests
test: $(BIJUX_TEST_TARGETS) ## Run configured fast tests
endif

ifneq ($(strip $(BIJUX_TEST_SLOW_TARGETS)),)
.PHONY: test-slow
BIJUX_HELP_TARGETS += test-slow
BIJUX_HELP_test-slow := Run configured slow tests
test-slow: $(BIJUX_TEST_SLOW_TARGETS) ## Run configured slow tests
endif

ifneq ($(strip $(BIJUX_TEST_ALL_TARGETS)),)
.PHONY: test-all
BIJUX_HELP_TARGETS += test-all
BIJUX_HELP_test-all := Run the complete configured test suite
test-all: $(BIJUX_TEST_ALL_TARGETS) ## Run the complete configured test suite
endif

ifneq ($(strip $(BIJUX_AUDIT_TARGETS)),)
.PHONY: audit
BIJUX_HELP_TARGETS += audit
BIJUX_HELP_audit := Run dependency policy and advisory checks
audit: $(BIJUX_AUDIT_TARGETS) ## Run dependency policy and advisory checks
endif

ifneq ($(strip $(BIJUX_SECURITY_TARGETS)),)
.PHONY: security
BIJUX_HELP_TARGETS += security
BIJUX_HELP_security := Run configured security checks
security: $(BIJUX_SECURITY_TARGETS) ## Run configured security checks
endif

ifneq ($(strip $(BIJUX_COVERAGE_TARGETS)),)
.PHONY: coverage
BIJUX_HELP_TARGETS += coverage
BIJUX_HELP_coverage := Generate configured coverage reports
coverage: $(BIJUX_COVERAGE_TARGETS) ## Generate configured coverage reports
endif
