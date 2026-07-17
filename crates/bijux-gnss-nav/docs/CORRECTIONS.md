# Corrections

`bijux-gnss-nav` owns reusable navigation-domain corrections that sit between raw observations and
estimation.

## Correction families

`src/corrections/` currently owns:

- atmosphere and broadcast-ionosphere correction helpers
- code and phase combinations
- bias handling and group-delay conversions
- dual-frequency, geometry-free, narrow-lane, and related diagnostic surfaces
- phase-windup and measured-ionosphere helpers

## Boundary rule

These computations are navigation-domain science. They are not receiver orchestration, and they are
not merely formatting helpers. If a correction needs satellite state, observation semantics, or
navigation-specific physical modeling, it belongs here.
