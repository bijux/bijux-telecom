---
title: Change Sequence
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Change Sequence

Use this sequence when modifying `bijux-gnss-dev`.

1. identify the owning maintainer workflow
2. name the governed inputs, outputs, and external commands that workflow uses
3. read the matching crate-local docs and root-handbook section
4. change the selected command path and only the closely coupled tests or docs
5. run the narrowest honest verification commands for that workflow
6. confirm that the governed input or output contract docs still match the
   changed behavior

## Why This Order Matters

The wrong order usually produces one of two failures: a command changes meaning
without updating its documented governance contract, or a green test run proves
something adjacent instead of the actual maintained workflow.
