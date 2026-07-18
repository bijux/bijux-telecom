---
title: Workflow Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-18
---

# Workflow Contracts

Maintainer workflows are safe only when validation, derivation, execution, and
review happen in the right order. A command returning success does not always
mean a gate was evaluated: missing optional input and advisory modes are
intentional states that callers must distinguish.

## Security Exception Workflow

```mermaid
flowchart TD
    edit["edit current advisory record"]
    validate["validate identifier, why, owner, link, and expiry"]
    valid{"validation succeeds?"}
    derive["derive sorted audit ignore arguments"]
    audit["run cargo audit with derived arguments"]
    review["review advisory and expiration"]
    stop["stop without deriving exceptions"]

    edit --> validate
    validate --> valid
    valid -- no --> stop
    valid -- yes --> derive
    derive --> audit
    audit --> review
```

The derivation command is not a validator. Safe automation must:

1. require the audit exception ledger;
2. run `audit-allowlist`;
3. stop on validation failure;
4. run `audit-ignore-args`;
5. pass the resulting arguments to the audit tool without maintaining a second
   exception list.

Current advisory records use `why`, owner, link, and expiry. Legacy ignore-only
entries are consumed by derivation but do not receive the same governance
validation. Migrate them before treating the workflow as fully reviewed.

The [audit policy](../../../crates/bijux-gnss-dev/docs/AUDIT_POLICY.md) defines
the human review expectation.

## Local Standards Deviation Workflow

```mermaid
flowchart LR
    upstream["upstream standards review"]
    record["local deviation with id, owner, reason, review, expiry"]
    validate["deny-policy-deviations"]
    gate["repository policy gate"]
    remove["remove deviation when upstream resolution lands"]

    upstream --> record
    record --> validate
    validate --> gate
    gate --> remove
```

The review link must use HTTP(S) and reference `bijux-std`. This command checks
that local deviations remain attributable and time-bounded; it does not decide
shared policy or update synchronized standards. Empty deviation lists are valid.

## Benchmark Evidence Workflow

```mermaid
flowchart TD
    change["performance-sensitive product change"]
    run["run curated benchmarks"]
    snapshot["write raw and normalized evidence"]
    baseline{"baseline exists?"}
    compare["compare configured ratio"]
    strict{"strict mode?"}
    advisory["report finding and succeed"]
    gate["fail on regression"]
    absent["record that comparison was skipped"]
    review["review product cause and evidence"]

    change --> run
    run --> snapshot
    snapshot --> baseline
    baseline -- no --> absent
    baseline -- yes --> compare
    compare --> strict
    strict -- no --> advisory
    strict -- yes --> gate
    advisory --> review
    gate --> review
    absent --> review
```

Use non-strict mode for investigation and strict mode for a gate. In either
mode, reviewers need the baseline identity, threshold, toolchain, benchmark
inventory, and raw evidence. A successful run without a baseline is execution
evidence, not regression evidence.

The [benchmark contract](../../../crates/bijux-gnss-dev/docs/BENCHMARKS.md) and
[output contract](../../../crates/bijux-gnss-dev/docs/OUTPUTS.md) define the
reviewed files.

## Slow-Test Lane Governance

The [slow-test roster](../../../configs/rust/nextest-slow-roster.txt) is
validated by the
[lane selection integration test](../../../crates/bijux-gnss-dev/tests/integration_nextest_suite_selection.rs).
That evidence checks sorted uniqueness, resolution to known test functions, and
agreement between fast and slow nextest expressions.

This is a repository test workflow, not one of the binary’s four commands.
Changes to the roster must update lane evidence, but they do not expand the
public command surface.

## Add A Maintainer Workflow

A new workflow belongs in this executable only when it has:

- a repository-maintenance owner rather than product behavior;
- a named governed input or explicit no-input contract;
- documented output, filesystem effects, and missing-input behavior;
- a failure mode that tells maintainers what to correct;
- deterministic behavior suitable for local and CI use;
- process-level evidence for observable command behavior;
- no duplicated rule already owned by shared standards or the policy crate.

Update the [binary boundary](binary-boundary.md),
[command surface](command-surface.md), and crate-local
[workflow guide](../../../crates/bijux-gnss-dev/docs/WORKFLOWS.md) together when
the command inventory changes.
