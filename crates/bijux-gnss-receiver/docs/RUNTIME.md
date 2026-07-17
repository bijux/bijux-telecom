# Runtime

`bijux-gnss-receiver` owns top-level receiver runtime composition.

## Runtime responsibilities

`src/engine/` currently owns:

- receiver configuration and validation
- runtime metrics and trace sinks
- logging and diagnostic routing
- top-level receiver engine composition
- support-matrix handling and runtime defaults

## Boundary rule

Runtime composition belongs here because this crate is where stages become a receiver. Lower-level
signal and navigation crates should not absorb runtime policy, and higher-level CLI or
infrastructure crates should not reach inside this layer to reassemble a second runtime.
