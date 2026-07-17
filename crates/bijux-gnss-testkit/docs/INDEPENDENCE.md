# Independence

`bijux-gnss-testkit` is only useful if it stays meaningfully independent from the product helpers it
validates.

## What independence means here

- reference models should not collapse into thin wrappers over production solvers
- truth generation should prefer explicit formulas, checked-in evidence, or alternative models
- fixture loading should not quietly depend on operator or repository workflow code

## Enforcement surface

`tests/scientific_independence.rs` is the explicit backstop for this rule, but the rule is broader
than that one test: contributors should treat independence as a crate-level design obligation.
