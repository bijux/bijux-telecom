# Time

`bijux-gnss-nav` owns navigation-specific time conversion and rollover interpretation beyond the
foundational time contracts in `bijux-gnss-core`.

## Time responsibilities

The time surface currently owns:

- GNSS-system-specific time records such as Galileo, BeiDou, and GLONASS time
- time-offset evidence and conversion wrappers
- civil-time parsing tied to navigation-product use
- week-rollover and GLONASS day-resolution logic in `src/time/rollover.rs`

## Boundary rule

Foundational time contracts live in `core`. Navigation-specific interpretation of external time
systems and rollover behavior lives here.
