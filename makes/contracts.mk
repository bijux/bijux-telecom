CONTRACT_SHELL_SCRIPTS := \
	makes/bin/run_gnss_rust_gate.sh

.PHONY: contract-tests

contract-tests: release-check bijux-docs-check ## Validate GNSS repository contracts
	@command -v shellcheck >/dev/null 2>&1 || { echo "shellcheck is required" >&2; exit 1; }
	@shellcheck $(CONTRACT_SHELL_SCRIPTS)
