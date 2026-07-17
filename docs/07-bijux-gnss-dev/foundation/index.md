---
title: Foundation
audience: mixed
type: index
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Foundation

Open this section when the question is whether a maintainer workflow belongs in
`bijux-gnss-dev` at all, what repository-owned language this crate uses, and
how its scope fits inside the GNSS workspace.

## Read These First

- open [Package Overview](package-overview.md) for the shortest accurate
  explanation of the crate
- open [Ownership Boundary](ownership-boundary.md) when a maintainer workflow
  is drifting toward product behavior
- open [Dependencies And Adjacencies](dependencies-and-adjacencies.md) when the
  question is whether a dependency or governed file belongs here

## Pages In This Section

- [Package Overview](package-overview.md)
- [Scope And Non-Goals](scope-and-non-goals.md)
- [Ownership Boundary](ownership-boundary.md)
- [Durable Naming](durable-naming.md)
- [Repository Fit](repository-fit.md)
- [Domain Language](domain-language.md)
- [Dependencies And Adjacencies](dependencies-and-adjacencies.md)
- [Change Principles](change-principles.md)

## First Boundary Check

- this crate owns repository maintenance workflows, not product features
- this crate may read and validate governed inputs, but that does not make it a
  general repository-scripting bucket
- this crate may emit maintenance evidence, but it does not own arbitrary
  output policy

## Leave This Section When

- leave for [Architecture](../architecture/) when ownership is clear and the
  question becomes code layout
- leave for [Interfaces](../interfaces/) when the question is already about
  commands, governed inputs, or output contracts
