---
title: Architecture
audience: mixed
type: index
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Architecture

Open this section when the question is how `bijux-gnss-dev` organizes its
binary command surface and repository-scoped effects in code.

## Read These First

- open [Module Map](module-map.md) first when the question is where a maintainer
  workflow lives
- open [Execution Model](execution-model.md) when the issue is what the binary
  is allowed to read, write, and execute
- open [Integration Seams](integration-seams.md) when the question crosses
  command parsing, governed files, and emitted evidence

## Pages In This Section

- [Module Map](module-map.md)
- [Dependency Direction](dependency-direction.md)
- [Execution Model](execution-model.md)
- [State And Persistence](state-and-persistence.md)
- [Integration Seams](integration-seams.md)
- [Error Model](error-model.md)
- [Extensibility Model](extensibility-model.md)
- [Code Navigation](code-navigation.md)
- [Architecture Risks](architecture-risks.md)

## First Code Roots

- `crates/bijux-gnss-dev/src/main.rs`
- `crates/bijux-gnss-dev/tests/integration_guardrails.rs`
- `crates/bijux-gnss-dev/tests/integration_nextest_suite_selection.rs`

## Leave This Section When

- leave for [Interfaces](../interfaces/) when the question is what maintainers
  may rely on rather than how the code is organized
- leave for [Quality](../quality/) when the structure is clear and the next
  question is whether the proof bar is strong enough
