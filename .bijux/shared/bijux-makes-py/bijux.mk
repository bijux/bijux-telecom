BIJUX_PY_WORKSPACE_DIR ?= $(abspath $(PROJECT_DIR)/..)
BIJUX_PY_REPOS ?= bijux-canon bijux-proteomics bijux-pollenomics
BIJUX_PY_SYSTEM_REL ?= shared/bijux-makes-py
BIJUX_PY_LOCAL_REL ?= makes/bijux-py
BIJUX_STANDARD_SHARED_DIR ?= $(if $(wildcard $(PROJECT_DIR)/.bijux/shared/bijux-gh),.bijux/shared/bijux-gh,shared/bijux-gh)
BIJUX_STANDARD_DEPENDABOT_RENDER ?= scripts/render-dependabot.sh
BIJUX_STANDARD_REQUIRED_FILES ?= \
	automation-identity.md \
	workflows/deploy-docs.yml \
	required-status-checks.md \
	rulesets/main-branch-protection.json
BIJUX_PY_REQUIRED_FILES ?= api-contract.mk api-freeze.mk api-live-contract.mk api.mk bijux.mk package.mk package-catalog.mk ci/build.mk ci/docs.mk ci/help.mk ci/lint.mk ci/quality.mk ci/sbom.mk ci/security.mk ci/test.mk ci/util.mk repository/config-layout.mk repository/env.mk repository/make-layout.mk repository/publish.mk repository/root.mk root/docs.mk root/env.mk root/lifecycle.mk root/package-dispatch.mk
BIJUX_PY_OPTIONAL_FILES ?=

.PHONY: check-bijux-standard bijux-standard-sync bijux-standard-check

check-bijux-standard: ## Verify shared bijux-py make modules match across sibling repositories
	@set -eu; \
	current_repo="$(PROJECT_SLUG)"; \
	workspace_dir="$(BIJUX_PY_WORKSPACE_DIR)"; \
	current_system_dir="$$workspace_dir/$$current_repo/$(BIJUX_PY_SYSTEM_REL)"; \
	current_local_dir="$$workspace_dir/$$current_repo/$(BIJUX_PY_LOCAL_REL)"; \
	compared_repos=0; \
	[ -d "$$current_system_dir" ] || { echo "✘ Missing shared make directory: $$current_system_dir"; exit 2; }; \
	[ -d "$$current_local_dir" ] || { echo "✘ Missing local make directory: $$current_local_dir"; exit 2; }; \
	for file in $(BIJUX_PY_REQUIRED_FILES); do \
	  [ -f "$$current_system_dir/$$file" ] || { echo "✘ Missing $$current_system_dir/$$file"; exit 2; }; \
	  [ -f "$$current_local_dir/$$file" ] || { echo "✘ Missing $$current_local_dir/$$file"; exit 2; }; \
	  cmp -s "$$current_system_dir/$$file" "$$current_local_dir/$$file" || { echo "✘ Local bijux-py drift: $$file differs between $(BIJUX_PY_SYSTEM_REL) and $(BIJUX_PY_LOCAL_REL) in $$current_repo"; exit 1; }; \
	done; \
	for file in $(BIJUX_PY_OPTIONAL_FILES); do \
	  if [ -f "$$current_system_dir/$$file" ] && [ -f "$$current_local_dir/$$file" ]; then \
	    cmp -s "$$current_system_dir/$$file" "$$current_local_dir/$$file" || { echo "✘ Local bijux-py drift: $$file differs between $(BIJUX_PY_SYSTEM_REL) and $(BIJUX_PY_LOCAL_REL) in $$current_repo"; exit 1; }; \
	  fi; \
	done; \
	for repo in $(BIJUX_PY_REPOS); do \
	  [ "$$repo" = "$$current_repo" ] && continue; \
	  other_system_dir="$$workspace_dir/$$repo/$(BIJUX_PY_SYSTEM_REL)"; \
	  if [ ! -d "$$other_system_dir" ]; then \
	    echo "→ Skipping sibling shared make check for $$repo; $$other_system_dir is not present"; \
	    continue; \
	  fi; \
	  compared_repos=$$((compared_repos + 1)); \
	  for file in $(BIJUX_PY_REQUIRED_FILES); do \
	    [ -f "$$other_system_dir/$$file" ] || { echo "✘ Missing $$other_system_dir/$$file"; exit 2; }; \
	    cmp -s "$$current_system_dir/$$file" "$$other_system_dir/$$file" || { echo "✘ Shared make drift: $$file differs between $$current_repo and $$repo"; exit 1; }; \
	  done; \
	  for file in $(BIJUX_PY_OPTIONAL_FILES); do \
	    if [ -f "$$current_system_dir/$$file" ] && [ -f "$$other_system_dir/$$file" ]; then \
	      cmp -s "$$current_system_dir/$$file" "$$other_system_dir/$$file" || { echo "✘ Shared make drift: $$file differs between $$current_repo and $$repo"; exit 1; }; \
	    fi; \
	  done; \
	done; \
	if [ "$$compared_repos" -eq 0 ]; then \
	  echo "✔ bijux-py modules are self-consistent; sibling repositories are not present in $$workspace_dir"; \
	else \
	  echo "✔ bijux-py modules match across $$compared_repos available sibling repositories"; \
	fi

bijux-standard-sync: ## Synchronize shared GitHub governance files into .github/
	@set -eu; \
	shared_dir="$(PROJECT_DIR)/$(BIJUX_STANDARD_SHARED_DIR)"; \
	[ -d "$$shared_dir" ] || { echo "✘ Missing shared governance directory: $$shared_dir"; exit 2; }; \
	for rel in $(BIJUX_STANDARD_REQUIRED_FILES); do \
	  src="$$shared_dir/$$rel"; \
	  dst="$(PROJECT_DIR)/.github/$$rel"; \
	  [ -f "$$src" ] || { echo "✘ Missing governance source file: $$src"; exit 2; }; \
	  mkdir -p "$$(dirname "$$dst")"; \
	  cp "$$src" "$$dst"; \
	  echo "→ synced .github/$$rel"; \
	done; \
	render="$$shared_dir/$(BIJUX_STANDARD_DEPENDABOT_RENDER)"; \
	[ -x "$$render" ] || { echo "✘ Missing executable Dependabot renderer: $$render"; exit 2; }; \
	"$$render" "$(PROJECT_DIR)" > "$(PROJECT_DIR)/.github/dependabot.yml"; \
	echo "→ generated .github/dependabot.yml"; \
	echo "✔ .github governance files synchronized from $(BIJUX_STANDARD_SHARED_DIR)"

bijux-standard-check: ## Verify .github governance files match shared GitHub governance sources
	@set -eu; \
	shared_dir="$(PROJECT_DIR)/$(BIJUX_STANDARD_SHARED_DIR)"; \
	[ -d "$$shared_dir" ] || { echo "✘ Missing shared governance directory: $$shared_dir"; exit 2; }; \
	for rel in $(BIJUX_STANDARD_REQUIRED_FILES); do \
	  src="$$shared_dir/$$rel"; \
	  dst="$(PROJECT_DIR)/.github/$$rel"; \
	  [ -f "$$src" ] || { echo "✘ Missing governance source file: $$src"; exit 2; }; \
	  [ -f "$$dst" ] || { echo "✘ Missing governed file: $$dst"; echo "  Run: make bijux-standard-sync"; exit 2; }; \
	  cmp -s "$$src" "$$dst" || { echo "✘ Governance drift: .github/$$rel differs from $(BIJUX_STANDARD_SHARED_DIR)/$$rel"; echo "  Run: make bijux-standard-sync"; exit 1; }; \
	done; \
	render="$$shared_dir/$(BIJUX_STANDARD_DEPENDABOT_RENDER)"; \
	[ -x "$$render" ] || { echo "✘ Missing executable Dependabot renderer: $$render"; exit 2; }; \
	[ -f "$(PROJECT_DIR)/.github/dependabot.yml" ] || { echo "✘ Missing governed file: $(PROJECT_DIR)/.github/dependabot.yml"; echo "  Run: make bijux-standard-sync"; exit 2; }; \
	tmp="$$(mktemp)"; \
	trap 'rm -f "$$tmp"' EXIT INT TERM; \
	"$$render" "$(PROJECT_DIR)" > "$$tmp"; \
	cmp -s "$$tmp" "$(PROJECT_DIR)/.github/dependabot.yml" || { echo "✘ Governance drift: .github/dependabot.yml is not generated from $(BIJUX_STANDARD_SHARED_DIR)/$(BIJUX_STANDARD_DEPENDABOT_RENDER)"; echo "  Run: make bijux-standard-sync"; exit 1; }; \
	echo "✔ .github governance files match shared $(BIJUX_STANDARD_SHARED_DIR) sources"

.PHONY: check-shared-bijux-py bijux-gh-py-sync bijux-gh-py-check
check-shared-bijux-py: check-bijux-standard ## Backward-compatible alias

bijux-gh-py-sync: bijux-standard-sync ## Backward-compatible alias

bijux-gh-py-check: bijux-standard-check ## Backward-compatible alias
