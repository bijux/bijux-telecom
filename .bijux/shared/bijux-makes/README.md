# Bijux Make Foundations

This directory contains language-neutral Make modules for Bijux repositories.
It owns stable entrypoints, artifact containment, documentation execution, CI
composition, help output, and pinned-source gate execution.

Consumers should include `bijux.mk` from the synchronized
`.bijux/shared/bijux-makes/` directory. Repository-owned Make files configure the
contract and retain domain-specific workflows.

See [CONTRACT.md](CONTRACT.md) for supported capabilities and target semantics.
