# no_std Decision

Decision: `no_std` is **not** a target for this workspace today.

Rationale:
- The infra/CLI layers depend on filesystem, process, and OS facilities.
- Current core algorithms are optimized for ergonomics and clarity rather than embedded constraints.

If this changes in the future, the plan is to isolate OS and allocation-heavy dependencies into `infra` only and introduce a `no_std`-compatible core subset.
