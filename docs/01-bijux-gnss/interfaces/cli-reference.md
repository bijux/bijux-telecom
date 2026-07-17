---
title: CLI Reference
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# CLI Reference

The durable public command surface of `bijux-gnss` is the `bijux gnss ...`
workflow family. This page exists so readers stop hunting for top-level help in
older root docs.

## Top-Level Shape

The repository exposes GNSS receiver workflows through the `gnss` command tree.
That tree owns operation, validation, diagnostics, replay, and report-facing
behavior. The crate does not promise that every internal subcommand path will
remain verbatim forever, but it does promise that operator workflows are owned
here rather than in lower crates.

The maintained command surface is organized around durable operator families:

- artifact and evidence inspection routes
- ingest and capture-facing workflows
- validate and diagnose routes for checked inputs and emitted evidence
- synthetic and replay-oriented proof workflows
- navigation or reporting routes that still begin from the top-level command
  owner even when lower crates do the scientific work

## What Readers Should Check First

- the live `--help` output for the current binary surface
- `crates/bijux-gnss/docs/PUBLIC_API.md` for the documented command families
- `crates/bijux-gnss/src/cli/command_catalog/` for the maintained catalog of
  command ownership
- `crates/bijux-gnss/src/cli/commands/` for the workflow handler that actually
  owns the route being reviewed

## Durable Reader Rule

If a question starts from an operator typing a command, the first handbook
owner is `01-bijux-gnss`, even when the implementation ultimately hands off to
receiver, infrastructure, signal, or navigation crates.
