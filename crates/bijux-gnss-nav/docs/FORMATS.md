# Formats

`bijux-gnss-nav` owns the navigation-domain parsers that turn external products into typed GNSS
state.

## Format families

`src/formats/` currently covers:

- GPS LNAV and CNAV
- Galileo FNAV and INAV
- BeiDou broadcast navigation
- GLONASS navigation decoding
- RINEX navigation and observation parsing
- precise products such as SP3, CLK, ANTEX, and bias SINEX

## Why these parsers belong here

These formats are not generic I/O. They are domain encodings of navigation state, clock products,
antenna calibrations, and signal biases. Keeping them in the navigation crate preserves a single
owner for how external navigation data enters the workspace.

## Boundary rules

- Repository file discovery belongs in infrastructure or CLI layers.
- Decoder and parser semantics belong here once the relevant bytes or text reach the crate.
- Persisted artifact naming and operator-facing command flags are outside this crate’s scope.
