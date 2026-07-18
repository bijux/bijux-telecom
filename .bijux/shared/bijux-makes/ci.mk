ifneq ($(strip $(BIJUX_CI_FAST_TARGETS)),)
.PHONY: ci-fast
BIJUX_HELP_TARGETS += ci-fast
BIJUX_HELP_ci-fast := Run the deterministic fast gate lane
ci-fast: $(BIJUX_CI_FAST_TARGETS) ## Run the deterministic fast gate lane
endif

ifneq ($(strip $(BIJUX_CI_PR_TARGETS)),)
.PHONY: ci ci-pr
BIJUX_HELP_TARGETS += ci ci-pr
BIJUX_HELP_ci := Run the canonical pull-request gate lane
BIJUX_HELP_ci-pr := Run all pull-request gates
ci: ci-pr ## Run the canonical pull-request gate lane
ci-pr: $(BIJUX_CI_PR_TARGETS) ## Run all pull-request gates
endif

ifneq ($(strip $(BIJUX_CI_NIGHTLY_TARGETS)),)
.PHONY: ci-nightly
BIJUX_HELP_TARGETS += ci-nightly
BIJUX_HELP_ci-nightly := Run complete and expensive gates
ci-nightly: $(BIJUX_CI_NIGHTLY_TARGETS) ## Run complete and expensive gates
endif

ifneq ($(strip $(BIJUX_CI_DOCS_TARGETS)),)
.PHONY: ci-docs
BIJUX_HELP_TARGETS += ci-docs
BIJUX_HELP_ci-docs := Run documentation and language API documentation gates
ci-docs: $(BIJUX_CI_DOCS_TARGETS) ## Run documentation and language API documentation gates
endif
