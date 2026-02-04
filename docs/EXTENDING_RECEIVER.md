# Extending Receiver

## Add a new stage
1. Implement stage in `crates/bijux-gnss-receiver/src/stages/`.
2. Keep stage inputs/outputs in core contract types.
3. Emit `DiagnosticEvent` for non-trivial issues.
4. Update CLI wiring in `bijux-gnss-cli` if new commands are needed.
