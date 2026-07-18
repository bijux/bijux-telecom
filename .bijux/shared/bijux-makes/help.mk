HELP_WIDTH ?= 28
BIJUX_HELP_TARGETS += help
BIJUX_HELP_help := Show available Make targets

.PHONY: help
help: ## Show available Make targets
	@$(foreach target,$(sort $(BIJUX_HELP_TARGETS)),printf '  \033[36m%-$(HELP_WIDTH)s\033[0m %s\n' '$(target)' '$(BIJUX_HELP_$(target))';)
