#![allow(missing_docs)]

use bijux_gnss_policies::api::{check, GuardrailConfig};

#[test]
fn guardrails() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    check(root, &GuardrailConfig::for_crate("bijux-gnss-infra-fuzz")).expect("guardrails failed");
}
