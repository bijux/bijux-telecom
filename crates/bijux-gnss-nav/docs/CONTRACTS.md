# Contracts

`bijux-gnss-nav` owns navigation-science contracts across four major families.

## Format contracts

The format layer owns stable decoding and parsing behavior for navigation messages, RINEX products,
and precise-reference products. It defines how external navigation data enters the workspace.

The format families and their boundary rules are detailed in the [format guide](FORMATS.md).

## Correction contracts

The correction layer owns:
- atmosphere and broadcast-ionosphere corrections
- code and phase combinations
- bias and antenna-effect application
- dual-frequency compatibility and correction reports

These are reusable domain computations, not receiver orchestration steps.

The correction families and their boundary are detailed in the [correction guide](CORRECTIONS.md).

## Estimation contracts

The estimation layer owns:
- position solution records and solver behavior
- integrity and RAIM evidence
- PPP filter and measurement contracts
- RTK ambiguity and baseline solution contracts

These contracts describe navigation-domain estimation behavior independent of CLI and repository
workflows.

The estimation-owned families and their boundary are detailed in the
[estimation guide](ESTIMATION.md).

## Orbit and model contracts

The orbit/model layer owns:
- ephemeris and satellite-state records
- physical supporting models needed by correction and estimation flows
- time-system rollover handling specific to navigation formats

The orbit-specific ownership surface is detailed in the [orbit guide](ORBITS.md).
The supporting model and time-interpretation surfaces are detailed in the
[model guide](MODELS.md) and [time guide](TIME.md).
