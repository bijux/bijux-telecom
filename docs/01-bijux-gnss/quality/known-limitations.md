---
title: Known Limitations
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Known Limitations

The command crate is intentionally thin. That is a strength only when the
handbook is honest about what the command can prove. A passing command test
means the operator workflow is wired and reported correctly; it does not prove
every lower-level scientific, runtime, or persistence claim.

## Limits Readers Should Know

| limitation | consequence | honest reading |
| --- | --- | --- |
| Thin command boundary | Many important questions must leave this handbook after the command route is clear. | Use this handbook to find the route, then switch to receiver, signal, nav, core, or infra docs for behavior meaning. |
| Integration-heavy tests | Command tests often exercise multiple crates, so a pass can hide which boundary was actually protected. | Name the command assertion and the lower-owner proof separately. |
| Facade pressure | Convenience exports can make the top crate look like it owns lower behavior. | Treat facade exports as routing convenience unless `FACADE.md` and `PUBLIC_API.md` say otherwise. |
| Reporting compression | Operator reports summarize lower-owned details and may omit internal evidence. | Follow the report to the artifact or lower crate before trusting a strong claim. |
| Synthetic workflows | Synthetic command success is bounded by the selected scenario and lower-owner model assumptions. | Pair command proof with signal or receiver proof when claiming physical behavior. |

## Claim Boundary

This handbook can support claims such as:

- "The CLI exposes the expected validation command and reports failures."
- "The synthetic navigation workflow is wired to the documented lower owners."
- "The command facade exports a deliberate public route."

This handbook cannot support claims such as:

- "The receiver tracks every supported signal under field conditions."
- "Navigation estimators are accurate outside the selected proof fixture."
- "Persisted artifacts are durable unless infra contracts are also checked."

## First Proof Route

Start with `crates/bijux-gnss/docs/TESTS.md`, then pick the narrow command
proof by the claim:

- guardrail and facade claims:
  `crates/bijux-gnss/tests/integration_guardrails.rs`
- configuration validation claims:
  `crates/bijux-gnss/tests/integration_validate_config.rs`
- navigation decode claims:
  `crates/bijux-gnss/tests/integration_nav_decode.rs`
- synthetic navigation claims:
  `crates/bijux-gnss/tests/integration_validate_synthetic_navigation.rs`

If the sentence says more than the command proof can defend, narrow the sentence
or route the reader to the owning lower crate.
