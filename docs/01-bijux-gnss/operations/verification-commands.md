---
title: Verification Commands
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Verification Commands

Use the narrowest relevant command that still proves the changed command
surface.

## Focused Commands From Repository Root

```sh
cargo test -p bijux-gnss --test integration_validate_config
cargo test -p bijux-gnss --test integration_nav_decode
cargo test -p bijux-gnss --test integration_validate_synthetic_navigation
```

## Additional Targeted Checks Worth Using

```sh
cargo test -p bijux-gnss --test integration_rinex
cargo test -p bijux-gnss --test integration_validate_capture
cargo test -p bijux-gnss --test integration_export_synthetic_iq
```

## Command Choice Rule

- use config and guardrail tests for command-shape changes
- use the narrowest workflow integration tests for one command family
- use validation-focused tests directly when validation command behavior moves
- use facade-adjacent tests only when the Rust surface truly changed
