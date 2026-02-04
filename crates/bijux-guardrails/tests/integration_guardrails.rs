#![allow(missing_docs)]
use std::path::Path;

use bijux_guardrails::api::{check, GuardrailConfig};

#[test]
fn guardrails() {
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut config = GuardrailConfig::for_crate("bijux-guardrails");
    config.max_loc = 520;
    check(crate_root, &config).unwrap_or_else(|err| panic!("guardrails failed: {err}"));
}
