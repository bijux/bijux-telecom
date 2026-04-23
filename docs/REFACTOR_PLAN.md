# Refactor Plan

## Mapping
- CLI: `src/parts/*` → `src/cli/*`
- Core: `core_time.rs`, `core_obs.rs`, `core_types.rs` → `time.rs`, `obs.rs`, `ids.rs`
- Nav: `gps/*` → `formats/lnav` + `orbits/` and `corrections/`
- Nav: `ppp_*` → `estimation/ppp/*`
- Nav: `ekf_*` → `estimation/ekf/*`
- Receiver: `core/*` → `runtime/*` and `sim/*` and `rtk/*`
- Receiver: `signal/*` → move reusable to `bijux-gnss-signal`

## Steps
1. Move CLI to `src/cli`, keep API stable.
2. Split core modules into `ids`, `time`, `obs`.
3. Restructure nav modules by domain (formats, orbits, corrections, estimation).
4. Restructure receiver modules and move shared signal primitives to `bijux-gnss-signal`.
5. Flatten depth and remove thin modules.
6. Rename files and update references.

## Done When
- `make ci-fast` passes.
- No numbered files exist.
- Path depth ≤ 4 for all crate files.
- Docs updated and consistent with module layout.
