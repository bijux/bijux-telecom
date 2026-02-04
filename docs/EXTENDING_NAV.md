# Extending Nav

## Add a new correction model
1. Implement in `crates/bijux-gnss-nav/src/corrections/`.
2. Wire into `compute_corrections` and ensure it is feature-gated if heavy.
3. Add tests in `crates/bijux-gnss-nav/tests/`.
4. Update `docs/ENGINE_CONTRACTS.md` if the contract changes.
