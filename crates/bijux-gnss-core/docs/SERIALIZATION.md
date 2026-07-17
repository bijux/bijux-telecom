# Serialization

`bijux-gnss-core` is where serialized GNSS meaning becomes durable enough for the rest of the
workspace to exchange, persist, and validate.

## What this crate serializes directly

- artifact envelopes in `src/artifact/`
- observation and differencing records in `src/observation/`
- navigation-solution records in `src/nav_solution.rs`
- diagnostic summaries and validation reports
- foundational identity, time, and unit-bearing value types when downstream crates persist them

## Serialization design rules

1. Contract types must serialize in a way that preserves scientific meaning without requiring the
   caller to know implementation module paths.
2. Validation belongs with the contract family that defines the data shape. Persisted payloads must
   be checked for semantic coherence, not just for structural parse success.
3. Versioned artifact payloads must evolve by adding explicit version boundaries instead of
   retroactively changing the meaning of existing serialized records.
4. Foundational types should remain unsurprising to downstream crates. If serialization needs
   context that only one runtime understands, the type belongs in a higher-level crate.

## Artifact-specific expectations

- `ArtifactHeaderV1` and `ArtifactV1` define the envelope that higher-level crates persist.
- Artifact kind and payload validation traits keep persisted data tied to explicit semantic rules.
- Observation, tracking, and navigation payloads must reject non-finite values or incompatible
  metadata combinations instead of allowing questionable data to pass as valid.

## Fixture responsibility

`tests/data/obs_fixture.jsonl` is the checked-in serialization fixture that anchors one stable path
through observation artifact validation. When serialization contracts change, update fixtures
deliberately and document the semantic reason in the same change set.

## Non-goals

This crate does not own filesystem naming, report-file naming, directory layout, or export-command
behavior. Those concerns belong in `bijux-gnss-infra` and the operator-facing crate.
