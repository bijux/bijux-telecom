RELEASE_SPEC ?= configs/release/crates.toml
RELEASE_VALIDATOR ?= makes/bin/validate_release.py
RELEASE_ARTIFACT_ROOT ?= artifacts/release
RELEASE_PACKAGE_ALLOW_DIRTY ?= 0
RUST_PUBLISH_DRY_RUN ?= 1
RUST_PUBLISH_SKIP_EXISTING ?= 0
CRATES_IO_API_USER_AGENT ?= bijux-telecom-release-check/1

.PHONY: release-check publish-rs release-crate-install release-crate-build

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
	done

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
