#![allow(missing_docs)]
use std::path::Path;

use bijux_gnss_policies::api::{check, GuardrailConfig};

#[test]
fn guardrails() {
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let config = GuardrailConfig::for_crate("bijux-gnss-policies");
    check(crate_root, &config).unwrap_or_else(|err| panic!("guardrails failed: {err}"));
}
