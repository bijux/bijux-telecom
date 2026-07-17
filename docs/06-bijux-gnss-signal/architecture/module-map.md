---
title: Module Map
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Module Map

The crate has one public facade and six private ownership regions behind it.

## Top-Level Structure

- `src/lib.rs` and `src/api.rs` own the private-to-public boundary
- `src/catalog.rs` owns signal identity, signal registry entries, wavelength
  helpers, and default-acquisition selection
- `src/codes/` owns constellation-specific code generation
- `src/dsp/` owns runtime-neutral processing primitives
- `src/raw_iq.rs` and `src/samples.rs` own sample representation and
  quantization contracts
- `src/obs_validation.rs` owns signal-layer observation compatibility
- `src/error.rs` owns reusable signal errors

## Code-Family Region

The `codes/` tree is grouped by durable GNSS family responsibility:

- `ca_code.rs` for GPS L1 C/A
- `gps_l2c_cl.rs`, `gps_l2c_cm.rs`, and `gps_l2c.rs` for GPS L2C components
  and time-multiplexed composition
- `gps_l2c_register.rs` for GPS L2C register-state helpers that support the
  public family without becoming a separate public contract
- `gps_l5.rs` for GPS L5 primary and secondary behavior
- `galileo_e1.rs` and `galileo_e5.rs` for Galileo code families
- `beidou_b1i.rs`, `beidou_b2i.rs`, and `beidou_d1.rs` for BeiDou families
- `glonass_l1.rs` for GLONASS L1 code and symbol helpers

Two modules remain intentionally private:

- `galileo_e1_tables.rs`
- `galileo_e5_tables.rs`

They support published code families without becoming public ownership units of
their own.

## DSP Region

The `dsp/` tree is organized by reusable mathematical role, not by runtime
stage:

- `front_end.rs` for FIR response handling
- `local_code.rs` for tracking-oriented local-code modeling
- `sample_timing.rs` and `signal.rs` for code-phase and sampling helpers
- `nco.rs` for numerically controlled oscillator state
- `replica.rs` and `dsp/replica/` for synthetic signal generation and wipeoff
  helpers
- `quality.rs` and `spectrum.rs` for front-end and spectral analysis
- `tracking.rs` and `dsp/tracking/` for loop and discriminator primitives
- `math.rs` for shared numeric helpers used by DSP modules

## First Proof Check

- `crates/bijux-gnss-signal/src/lib.rs`
- `crates/bijux-gnss-signal/src/api.rs`
- `crates/bijux-gnss-signal/src/codes/mod.rs`
- `crates/bijux-gnss-signal/src/dsp/mod.rs`
