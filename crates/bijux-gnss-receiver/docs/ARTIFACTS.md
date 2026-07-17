# Artifacts

`bijux-gnss-receiver` owns the in-memory result of a receiver run before repository persistence.

## Receiver artifact responsibilities

The receiver boundary owns:

- `RunArtifacts` as the top-level collected output of a pipeline run
- acquisition explainability and result collections
- tracking reports and per-channel state reports
- observation decisions, epochs, residuals, and quality reports
- navigation epochs and support-matrix capture

## Why this is distinct from infrastructure

The receiver crate decides what runtime information exists at the end of execution. The
infrastructure crate decides how that information is named, persisted, and indexed in the
repository. Keeping those responsibilities separate prevents runtime code from hard-coding
repository layout policy.

## Boundary rules

- Runtime artifacts may be rich and execution-oriented.
- Persisted manifest/report/file naming does not belong here.
- Reference validation can exist here when it is receiver-boundary logic rather than repository
  inspection logic.
