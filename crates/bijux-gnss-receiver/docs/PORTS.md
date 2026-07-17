# Ports

`bijux-gnss-receiver` owns runtime I/O seams through explicit source, sink, and clock boundaries.

## Port responsibilities

The receiver boundary owns:

- sample-source traits and adapters
- artifact-sink traits used during receiver execution
- clock abstractions such as `Clock` and `SystemClock`

## Why this belongs here

These seams are runtime-facing, not repository-facing. They let the receiver pipeline interact with
inputs, outputs, and time sources without hard-coding filesystem or operator workflow policy into
stage logic.

## Boundary rule

Ports here should stay execution-oriented. Repository manifests, persisted directories, and command
flags do not belong in these abstractions.
