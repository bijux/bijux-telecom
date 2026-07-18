---
title: Operations
audience: mixed
type: index
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Operations

Open this section when the question is how to change `bijux-gnss-dev` safely
without weakening repository governance or evidence discipline.

## Operational Model

```mermaid
flowchart LR
    change["proposed maintainer change"]
    scope["scope and owner check"]
    docs["workflow contract update"]
    tests["narrow verification"]
    review["governance review"]
    release["merge-ready change"]

    change --> scope --> docs --> tests --> review --> release
```

## Read These First

- open [Change Sequence](change-sequence.md) first when the question is how to
  stage a safe maintainer-command change
- open [Verification Commands](verification-commands.md) when the question is
  what proof to run
- open [Governed Input And Evidence Care](governed-input-and-evidence-care.md)
  when a reviewed file or evidence location is touched
- open [Review Scope](review-scope.md) when a change spans both maintainer
  commands and a neighboring support or product crate

## Pages In This Section

- [Common Workflows](common-workflows.md)
- [Local Development](local-development.md)
- [Change Sequence](change-sequence.md)
- [Contribution Guide](contribution-guide.md)
- [Verification Commands](verification-commands.md)
- [Governed Input And Evidence Care](governed-input-and-evidence-care.md)
- [Review Scope](review-scope.md)
- [Release And Versioning](release-and-versioning.md)

## First Operational Surfaces

- the [maintainer test suite](../../../crates/bijux-gnss-dev/tests/)
- the [maintainer test guide](../../../crates/bijux-gnss-dev/docs/TESTS.md)
- the [workflow guide](../../../crates/bijux-gnss-dev/docs/WORKFLOWS.md)
- the [policy crate](../../../crates/bijux-gnss-policies/)

## First Proof Check

Start with the [workflow guide](../../../crates/bijux-gnss-dev/docs/WORKFLOWS.md),
[governed-file guide](../../../crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md),
and [output contract guide](../../../crates/bijux-gnss-dev/docs/OUTPUTS.md).
Then inspect the operational surfaces above to confirm a proposed maintainer
change has been mapped to the right governed input, evidence path, and workflow
owner before execution starts.

## Leave This Section When

- leave for [Quality](../quality/) when the workflow is clear and the next
  question is whether the proof bar itself is strong enough
- leave for [Interfaces](../interfaces/) when the real issue is maintainer
  contract design rather than safe change procedure
- leave for [Foundation](../foundation/) when the operational question is
  actually a boundary dispute
