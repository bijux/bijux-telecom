# Extending Signal

## Add a new code/modulation
1. Implement in `crates/bijux-gnss-signal/src/codes/`.
2. Expose via `bijux-gnss-signal/src/api.rs`.
3. Add deterministic tests in `crates/bijux-gnss-signal/tests/`.
