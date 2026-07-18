RELEASE_SPEC ?= configs/release/crates.toml
RELEASE_VALIDATOR ?= makes/bin/validate_release.py
RELEASE_ARTIFACT_ROOT ?= artifacts/release
RELEASE_PACKAGE_ALLOW_DIRTY ?= 0
RUST_PUBLISH_DRY_RUN ?= 1
RUST_PUBLISH_SKIP_EXISTING ?= 0
CRATES_IO_API_USER_AGENT ?= bijux-telecom-release-check/1
GH_RELEASE_TAG_PATTERN ?= ^v[0-9]+\.[0-9]+\.[0-9]+$$
GH_RELEASE_CI_WORKFLOW_FILE ?= ci.yml
GH_RELEASE_CI_WAIT_TIMEOUT_SECONDS ?= 1800
GH_RELEASE_CI_POLL_INTERVAL_SECONDS ?= 15
GH_RELEASE_CI_LOOKBACK_SECONDS ?= 120
GH_RELEASE_CI_APPEARANCE_GRACE_SECONDS ?= 20

.PHONY: release-check publish-rs install release-crate-install release-crate-build \
	gh-release-plan-github gh-release-plan-crates \
	gh-release-require-cargo-token gh-release-wait-for-ci

release-check: ## Validate public package metadata, ordering, and licenses
	@python3 "$(RELEASE_VALIDATOR)" --repo-root "$(CURDIR)" --spec "$(RELEASE_SPEC)"

publish-rs: release-check ## Publish configured Rust crates; dry-run by default
	@set -euo pipefail; \
	packages="$(RUST_PUBLISH_PACKAGES)"; \
	if [ -z "$${packages}" ]; then \
		packages="$$(python3 "$(RELEASE_VALIDATOR)" --repo-root "$(CURDIR)" --spec "$(RELEASE_SPEC)" --print-publish-order)"; \
	fi; \
	dry_run_flag=""; \
	if [ "$(RUST_PUBLISH_DRY_RUN)" = "1" ]; then \
		dry_run_flag="--dry-run"; \
	fi; \
	for package in $${packages}; do \
		if [ "$(RUST_PUBLISH_SKIP_EXISTING)" = "1" ] && [ "$(RUST_PUBLISH_DRY_RUN)" != "1" ]; then \
			version="$$(cargo metadata --no-deps --format-version 1 | python3 -c 'import json, sys; name = sys.argv[1]; print(next(item["version"] for item in json.load(sys.stdin)["packages"] if item["name"] == name))' "$${package}")"; \
			status="$$(curl -A "$(CRATES_IO_API_USER_AGENT)" -fsS -o /dev/null -w '%{http_code}' "https://crates.io/api/v1/crates/$${package}/$${version}" 2>/dev/null || true)"; \
			if [ "$${status}" = "200" ]; then \
				echo "skipping $${package} $${version}; already present on crates.io"; \
				continue; \
			fi; \
		fi; \
		cargo publish --locked $${dry_run_flag} -p "$${package}"; \
		if [ "$(RUST_PUBLISH_DRY_RUN)" != "1" ]; then \
			version="$$(cargo metadata --no-deps --format-version 1 | python3 -c 'import json, sys; name = sys.argv[1]; print(next(item["version"] for item in json.load(sys.stdin)["packages"] if item["name"] == name))' "$${package}")"; \
			for attempt in $$(seq 1 20); do \
				status="$$(curl -A "$(CRATES_IO_API_USER_AGENT)" -fsS -o /dev/null -w '%{http_code}' "https://crates.io/api/v1/crates/$${package}/$${version}" 2>/dev/null || true)"; \
				if [ "$${status}" = "200" ]; then \
					break; \
				fi; \
				if [ "$${attempt}" = "20" ]; then \
					echo "crates.io did not expose $${package} $${version} after publication" >&2; \
					exit 1; \
				fi; \
				sleep 3; \
			done; \
		fi; \
	done

install: release-crate-install ## Install tools required by the shared release artifact workflow

release-crate-install: ## Confirm the Rust packaging toolchain is available
	@cargo --version

release-crate-build: ## Build a source bundle for the crate in the current directory
	@set -euo pipefail; \
	repo_root="$$(git rev-parse --show-toplevel)"; \
	package="$$(basename "$(CURDIR)")"; \
	output_root="$${repo_root}/$(ARTIFACTS_DIR)"; \
	dist_dir="$${output_root}/release"; \
	version="$$(python3 -c 'import pathlib, tomllib; print(tomllib.loads(pathlib.Path("'"$${repo_root}"'/Cargo.toml").read_text())["workspace"]["package"]["version"])')"; \
	archive="$${dist_dir}/$${package}-$${version}-source.tar.gz"; \
	mkdir -p "$${dist_dir}"; \
	if [ "$(RELEASE_PACKAGE_ALLOW_DIRTY)" = "1" ]; then \
		tar -C "$${repo_root}/crates" \
			--exclude='*/target' \
			--exclude='*/artifacts' \
			-czf "$${archive}" "$${package}"; \
	else \
		git -C "$${repo_root}" diff --quiet; \
		git -C "$${repo_root}" diff --cached --quiet; \
		git -C "$${repo_root}" archive \
			--format=tar.gz \
			--prefix="$${package}-$${version}/" \
			-o "$${archive}" \
			HEAD:"crates/$${package}"; \
	fi; \
	shasum -a 256 "$${archive}" > "$${archive}.sha256"

gh-release-plan-github: ## Decide whether the selected commit owns a stable release tag
	@test -n "$${GITHUB_OUTPUT:-}" || { echo "GITHUB_OUTPUT is required" >&2; exit 1; }
	@test -n "$${TARGET_SHA:-}" || { echo "TARGET_SHA is required" >&2; exit 1; }
	@set -euo pipefail; \
	tags="$$(git tag --points-at "$${TARGET_SHA}" | grep -E "$(GH_RELEASE_TAG_PATTERN)" || true)"; \
	if [ -z "$${tags}" ]; then \
		echo "publish=false" >> "$${GITHUB_OUTPUT}"; \
		exit 0; \
	fi; \
	tag="$$(printf '%s\n' "$${tags}" | head -n 1)"; \
	{ \
		echo "publish=true"; \
		echo "tag=$${tag}"; \
		echo "version=$${tag#v}"; \
	} >> "$${GITHUB_OUTPUT}"

gh-release-plan-crates: ## Select unpublished crates for the stable tag on the selected commit
	@test -n "$${GITHUB_OUTPUT:-}" || { echo "GITHUB_OUTPUT is required" >&2; exit 1; }
	@test -n "$${TARGET_SHA:-}" || { echo "TARGET_SHA is required" >&2; exit 1; }
	@set -euo pipefail; \
	tags="$$(git tag --points-at "$${TARGET_SHA}" | grep -E "$(GH_RELEASE_TAG_PATTERN)" || true)"; \
	if [ -z "$${tags}" ]; then \
		echo "publish=false" >> "$${GITHUB_OUTPUT}"; \
		exit 0; \
	fi; \
	tag="$$(printf '%s\n' "$${tags}" | head -n 1)"; \
	version="$${tag#v}"; \
	workspace_version="$$(python3 -c 'import pathlib, tomllib; print(tomllib.loads(pathlib.Path("Cargo.toml").read_text())["workspace"]["package"]["version"])')"; \
	if [ "$${version}" != "$${workspace_version}" ]; then \
		echo "release tag $${tag} does not match workspace version $${workspace_version}" >&2; \
		exit 1; \
	fi; \
	packages="$$(python3 "$(RELEASE_VALIDATOR)" --repo-root "$(CURDIR)" --spec "$(RELEASE_SPEC)" --print-publish-order)"; \
	unpublished=""; \
	for package in $${packages}; do \
		status="$$(curl -A "$(CRATES_IO_API_USER_AGENT)" -fsS -o /dev/null -w '%{http_code}' "https://crates.io/api/v1/crates/$${package}/$${version}" 2>/dev/null || true)"; \
		if [ "$${status}" != "200" ]; then \
			unpublished="$${unpublished}$${unpublished:+ }$${package}"; \
		fi; \
	done; \
	{ \
		if [ -n "$${unpublished}" ]; then echo "publish=true"; else echo "publish=false"; fi; \
		echo "packages=$${unpublished}"; \
		echo "tag=$${tag}"; \
		echo "version=$${version}"; \
	} >> "$${GITHUB_OUTPUT}"

gh-release-require-cargo-token: ## Require crates.io credentials before publication
	@test -n "$${CARGO_REGISTRY_TOKEN:-}" || { echo "CARGO_REGISTRY_TOKEN is required" >&2; exit 1; }

gh-release-wait-for-ci: ## Wait for CI success on the selected release commit
	@test -n "$${GITHUB_TOKEN:-}" || { echo "GITHUB_TOKEN is required" >&2; exit 1; }
	@test -n "$${GITHUB_REPOSITORY:-}" || { echo "GITHUB_REPOSITORY is required" >&2; exit 1; }
	@test -n "$${TARGET_SHA:-}" || { echo "TARGET_SHA is required" >&2; exit 1; }
	@test -n "$${CI_WAIT_STARTED_AT:-}" || { echo "CI_WAIT_STARTED_AT is required" >&2; exit 1; }
	@GH_RELEASE_CI_WORKFLOW_FILE="$(GH_RELEASE_CI_WORKFLOW_FILE)" \
	GH_RELEASE_CI_WAIT_TIMEOUT_SECONDS="$(GH_RELEASE_CI_WAIT_TIMEOUT_SECONDS)" \
	GH_RELEASE_CI_POLL_INTERVAL_SECONDS="$(GH_RELEASE_CI_POLL_INTERVAL_SECONDS)" \
	GH_RELEASE_CI_LOOKBACK_SECONDS="$(GH_RELEASE_CI_LOOKBACK_SECONDS)" \
	GH_RELEASE_CI_APPEARANCE_GRACE_SECONDS="$(GH_RELEASE_CI_APPEARANCE_GRACE_SECONDS)" \
	python3 .github/scripts/wait_for_ci.py
