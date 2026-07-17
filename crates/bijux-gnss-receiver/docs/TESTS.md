# Tests

`bijux-gnss-receiver` has the largest execution-oriented test surface in the workspace.

## Major test families

- acquisition accuracy, interference, explainability, truth-table, and uncertainty tests
- tracking lock, loop dynamics, continuity, handoff, and truth-table tests
- observation quality, smoothing, residual, and covariance tests
- navigation accuracy, protection-level, integrity, multipath, and validation-report tests
- RTK differencing, ambiguity fixing, and downstream artifact tests
- receiver capability, boundary, determinism, and support-matrix tests
- property tests and shared support fixtures under `tests/support/`

## What these tests are protecting

- stage boundaries remain coherent under real receiver flows
- runtime orchestration does not silently drift away from signal and navigation assumptions
- synthetic and reference-backed validation stays available at the receiver boundary
- runtime artifact collection remains meaningful before repository persistence is layered on top

## Verification

Useful commands from the repository root:

```sh
cargo test -p bijux-gnss-receiver --test integration_basic
cargo test -p bijux-gnss-receiver --test integration_receiver_support_matrix_inventory
cargo test -p bijux-gnss-receiver --test integration_navigation_pvt_accuracy_budget
```
