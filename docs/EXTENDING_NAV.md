# Extending Nav

## Add a new correction model
1. Implement in `crates/bijux-gnss-nav/src/corrections/`.
2. Wire into `compute_corrections` and ensure it is feature-gated if heavy.
3. Add tests in `crates/bijux-gnss-nav/tests/`.
4. Update `docs/ENGINE_CONTRACTS.md` if the contract changes.

## Extend satellite clock handling
1. Keep broadcast clock logic in `GpsSatelliteClockCorrection` and expose new terms there instead
   of scattering standalone bias fields across solvers.
2. Route external clock sources through `ProductsProvider::clock_correction`.
3. Preserve the rule that precise CLK bias replaces broadcast bias terms rather than stacking on
   top of broadcast relativistic or group-delay corrections.
4. Add coverage for both broadcast fallback and precise-clock override behavior in
   `crates/bijux-gnss-nav/tests/`.
