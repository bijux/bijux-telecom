#![allow(missing_docs)]

use bijux_guardrails::api::check;
use bijux_guardrails::api::GuardrailConfig;

#[test]
fn guardrails() {
    let mut config = GuardrailConfig::for_crate("bijux-gnss-infra");
    config.max_modules_per_dir = 15;
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    check(root, &config).expect("guardrails failed");
}
