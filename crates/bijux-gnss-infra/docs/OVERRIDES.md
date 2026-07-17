# Overrides

`bijux-gnss-infra` owns typed repository-side receiver-profile override application.

## Override responsibilities

The override surface currently owns:

- `CommonOverrides`
- direct profile override application
- sweep-key application through `apply_sweep_value`

## Why this belongs here

These overrides are part of the infrastructure boundary between repository experiments and receiver
configuration. They should not be reimplemented differently in every command path.

## Boundary rule

Override logic here should stay typed and repository-facing. It should not turn into a second home
for receiver default policy or for ad hoc CLI string munging.
