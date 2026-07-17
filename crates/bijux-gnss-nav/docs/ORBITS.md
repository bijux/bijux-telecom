# Orbits

`bijux-gnss-nav` owns the interpretation of broadcast and precise orbital state for the workspace.

## Orbit-owned surfaces

`src/orbits/` currently owns:

- constellation-specific broadcast ephemeris handling
- GLONASS-specific orbit behavior and state handling
- precise satellite-state support and uncertainty helpers
- the typed records downstream estimators consume when they need satellite state

## Boundary rule

Orbit handling here is about navigation-domain state interpretation. Repository file discovery does
not belong here, and receiver-stage scheduling does not belong here. Once navigation products reach
typed parsing and state interpretation, this crate owns them.
