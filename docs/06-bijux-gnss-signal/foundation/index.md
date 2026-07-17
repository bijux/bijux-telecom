---
title: Foundation
audience: mixed
type: index
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Foundation

Open this section when the question is whether a signal-layer behavior belongs
in `bijux-gnss-signal` at all, what durable language this crate uses, and how
its scope fits inside the repository.

## Read These First

- open [Package Overview](package-overview.md) for the shortest accurate
  explanation of the crate
- open [Ownership Boundary](ownership-boundary.md) when code or documentation
  is drifting across package lines
- open [Dependencies And Adjacencies](dependencies-and-adjacencies.md) when the
  question is whether a new dependency or exported helper belongs here

## Pages In This Section

- [Package Overview](package-overview.md)
- [Scope And Non-Goals](scope-and-non-goals.md)
- [Ownership Boundary](ownership-boundary.md)
- [Repository Fit](repository-fit.md)
- [Domain Language](domain-language.md)
- [Dependencies And Adjacencies](dependencies-and-adjacencies.md)
- [Change Principles](change-principles.md)

## First Boundary Check

- this crate owns reusable signal meaning and reusable DSP, not receiver runs
- this crate may validate signal compatibility, but it does not own navigation
  trust judgments
- this crate owns raw sample contracts, but not repository storage layout or
  capture-file discovery

## Leave This Section When

- leave for [Architecture](../architecture/) when ownership is clear and the
  question becomes code layout
- leave for [Interfaces](../interfaces/) when the question is already about
  public exports, traits, or metadata contracts
