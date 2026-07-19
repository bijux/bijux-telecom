# Configuration

`bijux-gnss-policies` owns typed configuration for repository guardrails. The
configuration is the reviewable contract between a policy intent and the code
that enforces it against crate source trees.

## Configuration Flow

```mermaid
flowchart LR
    intent["policy intent"]
    config["GuardrailConfig"]
    runner["check(crate_root, config)"]
    finding["GuardrailError or pass"]
    review["repository review"]

    intent --> config
    config --> runner
    runner --> finding
    finding --> review
```

## Configuration Surface

| family | controls | why it matters |
| --- | --- | --- |
| file and tree shape | file-size limits, source-depth limits, directory fan-out limits | Keeps crate structure reviewable before large files or flat directories become hidden architecture. |
| public surface density | public-item and `pub use` density limits | Keeps API growth deliberate and forces wide exports to justify themselves. |
| panic and staged strings | panic/expect and staged-string restrictions | Prevents brittle failure handling and source text that encodes delivery history. |
| purity zones | purity-zone paths and regex restrictions | Keeps product crates from quietly taking dependencies or side effects outside their boundary. |
| allowlists | intentionally narrow exceptions | Makes exceptions explicit, reviewable, and hard to confuse with general policy. |

## Boundary Rules

- Policy configuration governs repository structure, not product runtime
  behavior.
- A new configuration field needs a rule family, a default, and a failure message
  that a maintainer can act on.
- Allowlists are narrow exceptions, not a second policy language.
- Configuration changes that alter serialized defaults need snapshot review.

## Review Checks

- Can a reviewer tell which repository boundary the field protects?
- Does the default match the documented repository standard?
- Is the error message specific enough to fix the failing crate without reading
  guardrail internals?
- Do docs, snapshots, and tests move together when configuration meaning
  changes?
