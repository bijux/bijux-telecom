# DSP

`bijux-gnss-signal` owns reusable DSP primitives that sit below receiver orchestration.

## DSP families

`src/dsp/` currently owns:

- `front_end` for FIR front-end response handling
- `local_code`, `sample_timing`, and `signal` for code-phase and timing math
- `nco` for oscillator state and phase progression
- `replica` for synthetic carrier/code generation and wipeoff helpers
- `quality` and `spectrum` for front-end and PSD analysis
- `tracking` for reusable loop and discriminator primitives

## Boundary rules

- These helpers may transform samples and correlations, but they must stay runtime-neutral.
- Receiver stage sequencing, channel scheduling, and artifact persistence belong in
  `bijux-gnss-receiver`.
- Navigation-state interpretation belongs in `bijux-gnss-nav`.

## Stability expectation

When these helpers change, downstream acquisition, tracking, and validation behavior can all move
at once. Determinism, reference alignment, and long-duration continuity are therefore part of the
contract, not just implementation quality.
