---
title: Domain Language
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Domain Language

Use this vocabulary consistently when changing `bijux-gnss-dev`.

## Durable Terms

- governed input:
  a repository file whose content is reviewed and validated by this crate
- maintainer command:
  a binary subcommand meant for repository-health work, not product behavior
- maintenance evidence:
  emitted output that records the result of a repository workflow
- deviation:
  a reviewed downstream exception to shared policy
- benchmark baseline:
  the checked-in comparison reference for curated maintainer benchmark results

## Terms To Avoid Blurring

- public command:
  use this for `bijux-gnss`, not for maintainer-only tooling
- artifact:
  use this for governed evidence output, not every console message or temp
  string
- policy:
  use this for reviewed repository rules, not incidental implementation detail
