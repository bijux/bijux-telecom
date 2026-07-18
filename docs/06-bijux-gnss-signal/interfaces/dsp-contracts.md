---
title: DSP Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-18
---

# DSP Contracts

DSP contracts in `bijux-gnss-signal` are runtime-neutral promises. They may be
used by receiver stages, commands, validation helpers, and test fixtures, but
they do not decide receiver scheduling, artifact policy, or navigation solver
meaning.

## Public DSP Map

| module | contract | first proof |
| --- | --- | --- |
| `front_end` | FIR front-end specification and response measurement | `crates/bijux-gnss-signal/src/dsp/front_end.rs` |
| `quality` | I/Q metrics, noise-floor estimation, DC-offset handling | `crates/bijux-gnss-signal/src/dsp/quality.rs` |
| `local_code`, `sample_timing`, `signal` | code phase, samples-per-code, wrapping, carrier wipeoff, sample-index timing | `crates/bijux-gnss-signal/src/dsp/local_code.rs`, `crates/bijux-gnss-signal/src/dsp/sample_timing.rs`, `crates/bijux-gnss-signal/src/dsp/signal.rs` |
| `nco` | oscillator state and phase progression | `crates/bijux-gnss-signal/src/dsp/nco.rs` |
| `replica` | acquisition signal models, code models, carrier trajectories, synthetic modulation | `crates/bijux-gnss-signal/src/dsp/replica/` |
| `spectrum` | PSD estimation, expected spectra, transfer application, null finding | `crates/bijux-gnss-signal/src/dsp/spectrum.rs` |
| `tracking` | correlators, DLL/FLL/PLL helpers, discriminators, lock thresholds, CN0, tracking uncertainty | `crates/bijux-gnss-signal/src/dsp/tracking.rs` |

## Contract Flow

```mermaid
flowchart TD
    samples["samples and raw-IQ metadata"]
    timing["timing and code phase"]
    replica["local replica and carrier"]
    correlation["correlation and loop helpers"]
    quality["quality, CN0, uncertainty"]
    receiver["receiver stages consume results"]

    samples --> timing --> replica --> correlation --> quality --> receiver
```

## What Callers May Rely On

- DSP types and helpers are reusable without a `Receiver` or command runtime.
- Long-duration timing, phase wrapping, and code-phase behavior must remain
  deterministic across chunk boundaries.
- Front-end and spectrum helpers report signal quality without deciding what a
  receiver should do with that quality.
- Tracking helpers expose reusable math; the receiver decides channel
  lifecycle, state names, reacquisition policy, and emitted artifacts.

## What Callers Must Not Assume

- That internal helpers in `dsp/math.rs` are public promises just because
  receiver code happens to call nearby DSP modules.
- A DSP threshold is automatically a receiver lock policy.
- Replica generation owns synthetic scenario truth; receiver simulation owns
  scenario execution and truth artifacts.
- Signal-layer observation validation replaces core record meaning or
  receiver-side observation artifacts.

## First Proof Check

Inspect `crates/bijux-gnss-signal/docs/DSP.md`,
`crates/bijux-gnss-signal/src/dsp/`, and the focused signal integration tests:

- `crates/bijux-gnss-signal/tests/integration_nco_long_duration_phase.rs`
- `crates/bijux-gnss-signal/tests/integration_replica_continuity.rs`
- `crates/bijux-gnss-signal/tests/integration_signal_spectrum_cboc.rs`
- `crates/bijux-gnss-signal/tests/integration_local_code_continuity.rs`

If a downstream receiver test changes because a DSP primitive changed, update
the signal proof first, then update receiver expectations only when the runtime
handoff changed too.
