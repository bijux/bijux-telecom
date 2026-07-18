---
title: Public Imports
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Public Imports

This page summarizes the major import families published from
`bijux_gnss_receiver::api`.

## Runtime And Configuration Imports

- `ReceiverConfig`, `ReceiverPipelineConfig`, `ReceiverRuntimeConfig`, and
  `ReceiverError`
- logger, trace, and metrics sink interfaces plus runtime records
- `Receiver`, `ReceiverEngine`, and `RunArtifacts`

## Port And Source Imports

- `SampleSource`, `ArtifactSink`, `Clock`, and `SystemClock`
- `FileSamples`, `MemorySamples`, and `SampleSourceError`
- `SignalSource` for streaming signal inputs

## Stage Imports

- `AcquisitionEngine` and acquisition-assistance helpers
- `TrackingEngine` and channel or tracking-report types
- observation builders, residual reports, and quality reports
- `Navigation` and `NavigationFilter` when `nav` is enabled

## Validation And Simulation Imports

- reference alignment and comparison helpers under `nav`
- validation report builders and supporting report types
- synthetic receiver execution through `sim` when `nav` is enabled

## Re-Export Families

- `core` for `bijux-gnss-core`
- `signal` for `bijux-gnss-signal`
- `nav` for `bijux-gnss-nav` when enabled

These re-exports are for integration convenience. They do not transfer
ownership of those families into the receiver crate.
