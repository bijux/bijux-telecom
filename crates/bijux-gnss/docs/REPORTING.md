# Reporting

`bijux-gnss` owns operator-facing report rendering and output shape.

## Reporting responsibilities

The CLI boundary owns:

- report-format selection
- human-facing summaries and validation output rendering
- command-level presentation of lower-level results

## Boundary rule

The CLI can decide how information is presented to operators. It should not rewrite the scientific
meaning of lower-level results or duplicate persistence rules that belong to infrastructure.
