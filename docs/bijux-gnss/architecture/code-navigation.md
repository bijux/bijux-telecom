---
title: Code Navigation
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-18
---

# Code Navigation

Choose an entrypoint from the operator question. The command package is
organized by stable workflow responsibility: syntax, execution, runtime
context, lower-crate adaptation, and reporting.

## Follow an Operator Request

```mermaid
flowchart LR
    question["operator question"]
    syntax["syntax and defaults"]
    workflow["workflow handler"]
    context["inputs and runtime context"]
    owner["lower-crate owner"]
    output["report and artifacts"]

    question --> syntax
    syntax --> workflow
    workflow --> context
    context --> owner
    owner --> output
```

| question | start here |
| --- | --- |
| Where is a command, flag, default, or argument group defined? | [command catalog](../../../crates/bijux-gnss/src/cli/command_catalog/mod.rs) and [argument parser](../../../crates/bijux-gnss/src/cli/command_line.rs) |
| Which workflow executes the request? | [command handlers](../../../crates/bijux-gnss/src/cli/commands/mod.rs) |
| How are datasets, environment, and report context prepared? | [command runtime](../../../crates/bijux-gnss/src/cli/command_runtime.rs) |
| How are captures, artifacts, and lower-crate outputs adapted? | [command support](../../../crates/bijux-gnss/src/cli/command_support/mod.rs) |
| How is operator or JSON output rendered? | [report renderer](../../../crates/bijux-gnss/src/cli/report.rs) |
| Why is a Rust API available from the facade? | [facade exports](../../../crates/bijux-gnss/src/lib.rs) and the [public API guide](../../../crates/bijux-gnss/docs/PUBLIC_API.md) |

## Find the Workflow Family

| operator intent | handler |
| --- | --- |
| register or inspect input | [ingest workflow](../../../crates/bijux-gnss/src/cli/commands/ingest.rs) |
| export or measure synthetic IQ | [synthetic workflow](../../../crates/bijux-gnss/src/cli/commands/synthetic.rs) |
| execute receiver stages and persist evidence | [pipeline workflow](../../../crates/bijux-gnss/src/cli/commands/run_pipeline.rs) |
| inspect or explain artifacts | [artifact workflow](../../../crates/bijux-gnss/src/cli/commands/artifact.rs) |
| validate capture, schema, observation, or reference evidence | [validation workflows](../../../crates/bijux-gnss/src/cli/commands/validate/mod.rs) |
| run diagnostics, replay, quality, or navigation-decode operations | [diagnostic workflows](../../../crates/bijux-gnss/src/cli/commands/diagnostics/mod.rs) |
| analyze existing evidence | [analysis workflow](../../../crates/bijux-gnss/src/cli/commands/analyze.rs) |

## Trace a Failure

| failure surface | evidence |
| --- | --- |
| configuration or capture validation | [configuration validation integration](../../../crates/bijux-gnss/tests/integration_validate_config.rs) and [capture validation integration](../../../crates/bijux-gnss/tests/integration_validate_capture.rs) |
| raw-IQ export or metadata | [synthetic export integration](../../../crates/bijux-gnss/tests/integration_export_synthetic_iq.rs) and [raw-IQ metadata integration](../../../crates/bijux-gnss/tests/integration_raw_iq_metadata.rs) |
| navigation decoding or RINEX routing | [navigation decode integration](../../../crates/bijux-gnss/tests/integration_nav_decode.rs) and [RINEX integration](../../../crates/bijux-gnss/tests/integration_rinex.rs) |
| command-support adaptation | [command-support coverage](../../../crates/bijux-gnss/src/cli/command_support/tests/mod.rs) |
| pipeline composition and PVT output | [pipeline workflow coverage](../../../crates/bijux-gnss/src/cli/commands/run_pipeline_tests/fixtures.rs) |

Start with the observable failure and owning workflow. Move into lower crates
only when the command has handed off valid inputs and preserved the returned
evidence correctly.
