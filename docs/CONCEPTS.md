# Concepts

This document establishes the vocabulary and data flow used across `bijux-gnss`.

## Pipeline Stages
- **Acquisition**: Detects visible satellites and coarse code phase + Doppler.
- **Tracking**: Maintains carrier/code lock; outputs per-epoch prompt I/Q and NCO states.
- **Observations**: Builds measurement epochs (pseudorange, carrier phase, Doppler, CN₀).
- **Navigation**: Uses ephemeris + observations to estimate receiver state (PVT, PPP, RTK).

## Data Flow

```
CLI → Receiver → Signal + Core → Nav
```

- **CLI**: orchestration and UX only.
- **Receiver**: pipeline staging, scheduling, artifact IO.
- **Signal**: DSP primitives (NCO, correlators, code generation).
- **Core**: time, identity, units, artifacts, diagnostics.
- **Nav**: ephemeris, corrections, estimation (EKF/PPP/RTK).

See `docs/ARCHITECTURE.md` for dependency rules and ownership.
