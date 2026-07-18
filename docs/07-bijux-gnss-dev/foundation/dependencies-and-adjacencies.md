---
title: Dependencies And Adjacencies
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-18
---

# Dependencies And Adjacencies

`bijux-gnss-dev` is a repository maintenance executable. Its Rust dependency
graph is intentionally small, but successful execution also depends on named
repository files, the working directory or an explicit workspace root, and
external processes for selected commands.

## Four Dependency Classes

```mermaid
flowchart LR
    command["maintainer command"]
    rust["Rust libraries"]
    files["governed repository inputs"]
    process["external processes"]
    tests["test-only policy and scripts"]
    report["maintainer evidence or gate result"]

    rust --> command
    files --> command
    process -. "selected commands" .-> command
    tests -. "verification only" .-> command
    command --> report
```

| class | current dependency | purpose | failure ownership |
| --- | --- | --- | --- |
| Rust runtime | `clap` | command syntax and typed arguments | maintainer command contract |
| Rust runtime | `anyhow` | contextual failures for repository workflows | maintainer command contract |
| Rust runtime | `regex` | controlled parsing of benchmark output | benchmark evidence workflow |
| Rust runtime | `toml` | reviewed governance input parsing | governed-file contract |
| process | `cargo` | execute package benchmarks for comparison | local toolchain plus benchmark owner |
| process | `date` | resolve the current review date for expiring deviations | host environment |
| repository input | audit exceptions, deny deviations, benchmark baseline | source-controlled policy and evidence | repository governance owner |
| test-only | policy crate, nextest expression script, slow roster | prove guardrails and lane selection | repository test policy |

The exact compile dependency list is the
[package manifest](../../../crates/bijux-gnss-dev/Cargo.toml). Process spawning
is visible in the
[maintainer executable](../../../crates/bijux-gnss-dev/src/main.rs).

## Runtime Resolution

```mermaid
flowchart TD
    invoke["invoke maintainer command"]
    root{"workspace root supplied?"}
    explicit["use explicit root"]
    current["use current directory"]
    locate["join exact governed location"]
    parse["parse and validate"]
    child{"external command required?"}
    execute["run child in workspace root"]
    result["emit evidence or fail with context"]

    invoke --> root
    root -- yes --> explicit
    root -- no --> current
    explicit --> locate
    current --> locate
    locate --> parse
    parse --> child
    child -- yes --> execute
    child -- no --> result
    execute --> result
```

The executable does not search ancestor directories for a repository root. It
uses an explicitly supplied root where the command offers one; otherwise it
uses the current directory and joins the governed location. Running from an
arbitrary subdirectory can therefore produce a legitimate “file not found”
rather than discovering the repository automatically.

Benchmark comparison starts `cargo bench` in the resolved workspace root and
parses its output. This is a process boundary, not a Rust dependency on product
crates. A benchmark failure belongs first to the invoked package or local
toolchain; a successful process with unparseable output belongs to the
maintainer evidence parser.

## Product Crates Are Adjacent, Not Linked

```mermaid
flowchart LR
    product["product crate and benchmark"]
    cargo["cargo child process"]
    dev["maintainer executable"]
    policy["policy support in tests"]
    command["operator command crate"]

    product --> cargo
    cargo --> dev
    policy -.-> dev
    command -. "separate ownership" .- dev
```

The executable does not compile against receiver, navigation, signal, command,
or testkit crates. It may inspect their benchmark output or repository evidence
without acquiring product behavior. The policy crate is currently a
development-only dependency used by guardrail tests, not production command
logic.

This distinction prevents two common mistakes:

- adding a product crate dependency to call internal behavior from a repository
  gate;
- moving operator commands into maintainer tooling because both happen to use a
  command-line parser.

Product runtime behavior belongs to the
[command boundary](../../01-bijux-gnss/foundation/ownership-boundary.md).
Reusable repository rules belong to the policy support crate; command
orchestration and maintainer-facing evidence remain here.

## Review A New Dependency

Before adding a library, file, or process dependency, answer:

1. Which maintainer workflow owns it, and what evidence becomes more reliable?
2. Can the behavior be implemented through an existing governed input or
   standard-library facility without obscuring intent?
3. Is the dependency deterministic enough for local and CI use?
4. Does its failure message identify the missing tool, file, policy record, or
   product command?
5. Does it preserve the boundary between repository maintenance and GNSS
   product execution?
6. Does the [governance file contract](../../../crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md)
   need a matching update?

The [dependency direction guide](../architecture/dependency-direction.md)
defines compile and process direction. The [integration seams guide](../architecture/integration-seams.md)
explains how new governed inputs and child-process behavior should be tested.
