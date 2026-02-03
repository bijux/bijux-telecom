# Performance Playbook

This document describes how to profile and benchmark bijux-gnss, plus baseline expectations for hot paths.

## Benchmarks

Run all benchmarks:

```bash
cargo bench
```

Key benches:
- `ca_code_prn1` — C/A code generation
- `acquisition_fft_prn1` — FFT-based acquisition correlation

## Profiling

### macOS Instruments (Time Profiler)
1. Build a release binary:
   ```bash
   cargo build --release
   ```
2. Run under Instruments → Time Profiler.

### Flamegraph (Linux)
1. Install tools:
   ```bash
   cargo install flamegraph
   ```
2. Run:
   ```bash
   cargo flamegraph --bench acquisition
   ```

### Heap profiling (dhat)
```bash
cargo install dhat
```
Use `Dhat` with a small test case to inspect allocations.

## Baseline Budgets (Target)
- Acquisition FFT (1 ms, 5 MHz): < 3 ms per PRN on a modern laptop.
- C/A code generation: < 0.5 ms per PRN.
- E/P/L correlator per channel per epoch: < 0.2 ms.

These are target budgets and should be refined as the implementation matures.
