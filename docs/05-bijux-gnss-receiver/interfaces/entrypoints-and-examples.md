---
title: Entrypoints And Examples
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Entrypoints And Examples

Use this page to choose the right public starting point.

## Common Starting Points

- start from `bijux_gnss_receiver::api` when you are a downstream crate
- start from `Receiver` or `ReceiverEngine` when the question is how to launch
  the runtime
- start from port contracts when the question is how samples, clocks, or sinks
  enter the runtime
- start from `AcquisitionEngine`, `TrackingEngine`, or observation helpers when
  the question is stage-local behavior
- start from validation or `sim` surfaces when the question is runtime proof

## Example Reader Routes

- "I need to run the receiver against file-backed samples":
  runtime contracts, then port contracts
- "I need to understand which stage produced this artifact":
  artifact contracts, then stage contracts
- "I need runtime-side validation against reference truth":
  validation and simulation contracts

## Protecting Proof

Inspect `crates/bijux-gnss-receiver/src/api.rs`,
`crates/bijux-gnss-receiver/docs/PUBLIC_API.md`,
`crates/bijux-gnss-receiver/docs/RUNTIME.md`,
`crates/bijux-gnss-receiver/docs/PORTS.md`, and
`crates/bijux-gnss-receiver/docs/REFERENCE_VALIDATION.md` to confirm these
reader routes still match real public runtime entrypoints.
