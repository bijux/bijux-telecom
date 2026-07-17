# Contracts

`bijux-gnss-signal` owns computational signal contracts rather than pipeline contracts.

## Signal catalog contract

The crate owns canonical signal definitions, wavelength conversions, and registry access used across
acquisition, tracking, and synthetic generation.

## Code-generation contract

The code families own:
- deterministic primary-code generation
- family-specific secondary-code and symbol helpers
- stable assignment lookups and reference constants

These functions define the workspace’s canonical signal-code behavior.

The supported code families and their boundary rules are detailed in
[CODE_FAMILIES.md](CODE_FAMILIES.md).

## DSP contract

The DSP layer owns:
- sample-domain helpers for code phase, carrier phase, and timing
- reusable front-end and spectrum analysis primitives
- replica generation and carrier/code wipeoff primitives
- tracking-loop math and uncertainty helpers

These contracts stay below receiver orchestration so they can be tested independently.

The DSP families and their runtime boundary are detailed in [DSP.md](DSP.md).

## Validation contract

The observation-validation layer owns signal-level compatibility checks such as supported
dual-frequency band pairs and inter-frequency alignment reports. It does not own navigation-quality
decisions.

## Metadata contract

`RawIqMetadata`, quantization enums, and sample-conversion helpers define the stable raw-sample
metadata layer shared by CLI, infrastructure, receiver, and tests.
