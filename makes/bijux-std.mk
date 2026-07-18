BIJUX_STD_CHECK_SCRIPT ?= .bijux/shared/bijux-checks/check-bijux-std.sh
BIJUX_STD_UPDATE_SCRIPT ?= .bijux/shared/bijux-checks/update-bijux-std.sh
BIJUX_STD_REF ?= main
BIJUX_STD_REMOTE ?= https://raw.githubusercontent.com/bijux/bijux-std
BIJUX_STD_GIT_URL ?= https://github.com/bijux/bijux-std.git
BIJUX_STD_UPDATE_CHANNEL ?= branch
BIJUX_STD_TAG_PATTERN ?= v*
BIJUX_STD_PYCACHE_DIR ?= artifacts/bijux-std/pycache

.PHONY: bijux-std-checks bijux-std-update bijux-std

bijux-std-checks: ## Verify shared directories match bijux-std
	@mkdir -p "$(BIJUX_STD_PYCACHE_DIR)"
	@PYTHONPYCACHEPREFIX="$(abspath $(BIJUX_STD_PYCACHE_DIR))" BIJUX_STD_REF="$(BIJUX_STD_REF)" BIJUX_STD_REMOTE="$(BIJUX_STD_REMOTE)" bash "$(BIJUX_STD_CHECK_SCRIPT)"

bijux-std-update: ## Update shared directories from bijux-std
	@mkdir -p "$(BIJUX_STD_PYCACHE_DIR)"
	@PYTHONPYCACHEPREFIX="$(abspath $(BIJUX_STD_PYCACHE_DIR))" BIJUX_STD_REF="$(BIJUX_STD_REF)" BIJUX_STD_GIT_URL="$(BIJUX_STD_GIT_URL)" BIJUX_STD_UPDATE_CHANNEL="$(BIJUX_STD_UPDATE_CHANNEL)" BIJUX_STD_TAG_PATTERN="$(BIJUX_STD_TAG_PATTERN)" bash "$(BIJUX_STD_UPDATE_SCRIPT)"

bijux-std: bijux-std-checks ## Backward-compatible alias
