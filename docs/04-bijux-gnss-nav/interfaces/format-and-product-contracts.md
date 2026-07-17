---
title: Format And Product Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Format And Product Contracts

These contracts define how external navigation truth enters the crate.

## Owned Contract Families

- GPS LNAV and CNAV decoding
- Galileo FNAV and INAV decoding
- BeiDou B1I and D2 decoding
- GLONASS navigation decoding
- RINEX navigation and observation parsing
- precise-product parsing for SP3, CLK, ANTEX, and bias SINEX

## Stability Expectations

- decoded records and rejection reasons should stay scientifically meaningful
- callers may rely on constellation-specific decoding behavior being owned here
- file discovery and repository placement are explicitly outside the contract

## Boundary Rule

These are domain decoders, not generic file readers. The contract begins once a
caller has acquired the relevant bytes or text and ends when typed navigation
state or typed rejection evidence is produced.

## Closest Proof

- `crates/bijux-gnss-nav/src/formats/`
- `crates/bijux-gnss-nav/docs/FORMATS.md`

## Protecting Proof

Inspect `crates/bijux-gnss-nav/src/formats/`,
`crates/bijux-gnss-nav/docs/FORMATS.md`, and the most relevant decoder and
product tests such as SP3, broadcast orbit, and constellation-specific
navigation decode families to confirm these contracts still start at typed
navigation state rather than repository file handling.
