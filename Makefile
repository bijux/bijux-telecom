.PHONY: bijux-std-checks

BIJUX_STD_CHECK_SCRIPT ?= .bijux/shared/bijux-checks/check-bijux-std.sh
BIJUX_STD_REF ?= main
BIJUX_STD_REMOTE ?= https://raw.githubusercontent.com/bijux/bijux-std

bijux-std-checks:
	@BIJUX_STD_REF="$(BIJUX_STD_REF)" BIJUX_STD_REMOTE="$(BIJUX_STD_REMOTE)" bash "$(BIJUX_STD_CHECK_SCRIPT)"
