---
title: Known Limitations
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Known Limitations

`bijux-gnss-dev` turns repository maintenance rules into executable commands.
That is useful only while the commands stay narrow. The binary can validate
shape, provenance, expiry, and benchmark comparison mechanics; it cannot replace
human review or become a hidden product API.

## Limits Readers Should Know

| limitation | consequence | honest reading |
| --- | --- | --- |
| Audit validation checks shape, not risk acceptance. | A passing allowlist check means every exception is explicit, attributable, and time-bounded. It does not mean the exception is scientifically or commercially acceptable. | Pair a pass with review of `why`, `owner`, `link`, and `expiry`. |
| Deny deviation validation checks local exception discipline. | A passing deviation check means the local policy exception is documented and tied to review. It does not change the upstream standard. | Treat durable policy fixes as `bijux-std` work, not local binary work. |
| Benchmark comparison is bounded. | The command runs a curated benchmark set and compares normalized output when a baseline exists. It does not prove every receiver or nav performance path. | Read benchmark output as regression evidence for the owned benchmark set only. |
| One-file binary organization is a current constraint. | `src/main.rs` is still readable for the present command set, but adding unrelated workflow families would make review harder. | Add commands only when the governed input, output, and owner are explicit. |
| Maintainer convenience is not ownership. | A useful script idea can still be wrong for this crate if it hides policy or reaches into product internals. | Route reusable behavior to the owning product or policy crate. |

## Proof Route

Read these before trusting or extending a maintainer workflow:

1. `crates/bijux-gnss-dev/docs/BOUNDARY.md`
2. `crates/bijux-gnss-dev/docs/COMMANDS.md`
3. `crates/bijux-gnss-dev/docs/WORKFLOWS.md`
4. `crates/bijux-gnss-dev/docs/OUTPUTS.md`
5. `crates/bijux-gnss-dev/docs/TESTS.md`

Then inspect `crates/bijux-gnss-dev/src/main.rs` and the matching integration
test. The limit is acceptable only when the command's governed input and output
are named in documentation and the test proves the documented rule.
