Layered Dependency DAG

This workspace uses a single allowed dependency graph between crates:

cli -> infra -> receiver -> {signal, core, nav} -> core

Notes:
- This is a strict DAG. No other workspace crate edges are permitted.
- If receiver needs to call nav, it must depend on nav explicitly (as shown).
- All other workspace dependencies (including dev/build dependencies) should respect the same directionality and avoid cycles.
