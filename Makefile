.PHONY: bijux-std-checks ci ci-fast ci-pr ci-nightly ci-docs ci-dependency-lock-refresh

BIJUX_STD_CHECK_SCRIPT ?= .bijux/shared/bijux-checks/check-bijux-std.sh
BIJUX_STD_REF ?= main
BIJUX_STD_REMOTE ?= https://raw.githubusercontent.com/bijux/bijux-std

bijux-std-checks:
	@BIJUX_STD_REF="$(BIJUX_STD_REF)" BIJUX_STD_REMOTE="$(BIJUX_STD_REMOTE)" bash "$(BIJUX_STD_CHECK_SCRIPT)"

ci:
	@cargo make ci

ci-fast:
	@cargo make ci-fast

ci-pr:
	@cargo make ci-pr

ci-nightly:
	@cargo make ci-nightly

ci-docs:
	@cargo make ci-docs

ci-dependency-lock-refresh:
	@cargo make ci-dependency-lock-refresh
