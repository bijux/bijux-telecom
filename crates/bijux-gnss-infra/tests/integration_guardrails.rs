#![allow(missing_docs)]

use bijux_guardrails::api::run_guardrails_with_config;
use bijux_guardrails::GuardrailConfig;

#[test]
fn guardrails() {
    let config = GuardrailConfig::for_crate("bijux-gnss-infra");
    run_guardrails_with_config(env!("CARGO_MANIFEST_DIR"), &config)
        .expect("guardrails failed");
}
