#![allow(missing_docs)]
use std::path::Path;

use bijux_guardrails::api::{check, GuardrailConfig};

#[test]
fn guardrails() {
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let config = GuardrailConfig::for_crate("bijux-gnss-signal");
    check(crate_root, &config).unwrap_or_else(|err| panic!("guardrails failed: {err}"));
}
