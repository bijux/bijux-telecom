# Outputs

`bijux-gnss-dev` writes evidence outputs for repository maintenance workflows.
These outputs must be predictable because reviewers need to know whether a file
is a baseline, current-run evidence, or disposable command output.

## Output Flow

```mermaid
flowchart LR
    command["maintainer command"]
    governed["governed input"]
    evidence["evidence output"]
    reviewer["reviewer"]

    governed --> command
    command --> evidence
    evidence --> reviewer
```

## Owned Output Locations

| path | meaning |
| --- | --- |
| `artifacts/benchmarks.txt` | local benchmark comparison evidence |
| `benchmarks/bencher_current.txt` | checked or reviewed current benchmark snapshot |
| `benchmarks/bencher_baseline.txt` | checked or reviewed baseline benchmark snapshot |

## Contract Rules

- Generated run evidence belongs under `artifacts/` unless a reviewed workflow
  explicitly owns a checked-in output path.
- Checked-in benchmark snapshots must be understandable without local logs.
- A command that introduces a new output path must document that path here and
  in the command docs before reviewers are asked to rely on it.
- Maintenance outputs should not mix product runtime artifacts with governance
  evidence unless the command explains the relationship.

## Not Owned Here

- receiver run artifacts belong to `bijux-gnss-receiver` and
  `bijux-gnss-infra`
- operator command reports belong to `bijux-gnss`
- shared artifact envelopes belong to `bijux-gnss-core`

## Proof Surfaces

- `src/main.rs`
- [Benchmark guide](BENCHMARKS.md)
- [Command guide](COMMANDS.md)
- repository Make targets that call `bijux-gnss-dev`
