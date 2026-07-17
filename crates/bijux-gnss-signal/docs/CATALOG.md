# Catalog

`bijux-gnss-signal` owns the canonical registry of supported signal definitions and physical lookup
helpers.

## Catalog responsibilities

The catalog surface currently owns:

- signal-spec constructors by constellation and band
- wavelength and cycle/meter conversion helpers
- signal-registry lookup and registered-entry exposure
- default acquisition signal selection helpers

## Boundary rule

Catalog ownership here is about signal meaning and lookup, not about navigation-state inference or
receiver scheduling policy.
