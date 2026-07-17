---
title: Common Workflows
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Common Workflows

This page describes the recurring edit patterns in `bijux-gnss`.

## Add Or Change A Command

- confirm the change belongs in the command crate rather than a lower owner
- update command-facing docs if the public shape or workflow meaning moves
- run the narrowest integration tests that prove the new or changed command
- inspect reporting and validation surfaces if operator output changes

## Change Workflow Wiring

- identify which lower owners are being composed
- keep the command layer focused on orchestration rather than deep behavior
- run tests that prove the top-level workflow still calls the intended lower
  surfaces

## Change Reporting

- confirm the change is operator-facing output, not lower-owner policy
- review whether the command is rephrasing or redefining deeper meaning
- run the narrowest tests that exercise the changed report path

## Change The Facade

- confirm the export is a durable package convenience
- avoid growing bespoke helpers in the CLI crate
- check whether the better fix belongs in a lower crate instead
