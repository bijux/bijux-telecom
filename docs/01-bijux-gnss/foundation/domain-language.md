---
title: Domain Language
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Domain Language

This page fixes the durable vocabulary used across the `bijux-gnss` handbook.

## Command Boundary

The public operator-facing surface: command names, flags, workflows, and output
shape.

## Workflow Composition

The act of wiring lower-level crates into one top-level command flow without
claiming ownership of their deeper behavior.

## Runtime Setup

The command-owned preparation needed before lower-level runtime execution
begins, such as environment handling, input loading, or output routing.

## Reporting

Operator-facing rendering of success, failure, or diagnostic output owned by
the command crate.

## Facade Export

A thin Rust re-export surface that lets callers reach the GNSS stack through
one package without turning the CLI crate into a mixed-responsibility library.
