# Public API

`bijux-gnss-nav` publishes one curated downstream surface through `bijux_gnss_nav::api`.

## Public API families

### Corrections

The API exposes atmospheric, bias, dual-frequency, ionosphere-free, geometry-free, narrow-lane,
wide-lane, and related correction helpers and report types.

### Estimation

The API exposes:
- EKF state and update primitives
- position solver and runtime types
- integrity and RAIM report types
- PPP configuration and measurement helpers
- RTK ambiguity, baseline, execution, and quality types

### Format and product families

The API exposes decoders and parsers for:
- GPS LNAV and CNAV
- Galileo FNAV and INAV
- BeiDou and GLONASS navigation formats
- RINEX navigation and observation
- SP3, CLK, ANTEX, and bias SINEX precise products

### Orbit, model, and time families

The API exposes:
- orbit and ephemeris records
- antenna, atmosphere, tide, and NeQuick model types
- navigation time-system and rollover helpers
- small matrix and geodesy helpers used by downstream consumers

## Public trait surface

- `NavEngine` is the crate’s top-level navigation-engine trait seam.

## Extension rule

If an export is parser-local, solver-internal, or only meaningful to one internal module, keep it
out of `api.rs`. The public surface should stay organized by durable scientific role.
