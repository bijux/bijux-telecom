CONFIG_LAYOUT_REQUIRED_FILES ?= coveragerc.ini deptry.toml mypy.ini package-lock.json package.json pytest.ini ruff.toml schemathesis.toml

.PHONY: check-config-layout

root-check-env: check-config-layout

check-config-layout: ## Validate the repository config tree shape and required tool configs
	@set -eu; \
	config_dir="$(CONFIG_DIR)"; \
	test -d "$$config_dir" || { echo "✘ Missing config directory: $$config_dir"; exit 1; }; \
	for file in $(CONFIG_LAYOUT_REQUIRED_FILES); do \
	  test -f "$$config_dir/$$file" || { echo "✘ Missing repository config: $$config_dir/$$file"; exit 1; }; \
	done; \
	echo "✔ config tree layout is complete"
