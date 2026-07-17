# Models

`bijux-gnss-nav` owns the supporting physical and environmental models needed by navigation-domain
science.

## Model families

`src/models/` currently owns:

- antenna models
- atmosphere models
- celestial and NeQuick support
- ocean tide loading
- solid earth tide effects

## Boundary rule

These models belong here because they directly support navigation corrections and estimators. They
are not generic utilities, and they are not repository-facing workflow helpers.
