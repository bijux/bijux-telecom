# Code Families

`bijux-gnss-signal` owns the canonical spreading-code behavior for supported GNSS signal families.

## Covered families

`src/codes/` currently owns:

- GPS L1 C/A, L2C, and L5
- Galileo E1 and E5
- BeiDou B1I, B2I, and D1 secondary-code helpers
- GLONASS L1

## Why this boundary matters

Code generation is one of the easiest places for a workspace to drift into "almost the same"
implementations. This crate centralizes those families so acquisition, tracking, validation, and
test synthesis all share one canonical source of signal-code behavior.

## Ownership rules

- Family-specific assignments, constants, primary-code generation, and secondary-code helpers
  belong here.
- Receiver-specific search strategy does not.
- Navigation-message decoding does not.
- Reference fixtures may live in tests, but the production code behavior remains owned here.
