test:
	cargo make test

fmt:
	cargo fmt --all

clippy:
	cargo clippy --workspace --all-targets --all-features -- -D warnings

ci:
	cargo make ci

run-cli-example:
	cargo test -p bijux-gnss-receiver --test integration_cli_example -- --nocapture
