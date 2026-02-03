use std::path::PathBuf;

use bijux_guardrails::GuardrailConfig;

#[test]
fn guardrail_default_policy_snapshot() {
    let config = GuardrailConfig::default();
    let serialized = serde_json::to_string_pretty(&config).expect("serialize config");
    let snapshot = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("snapshots")
        .join("guardrail_default.json");
    let expected = std::fs::read_to_string(&snapshot).expect("read snapshot");
    assert_eq!(serialized.trim_end(), expected.trim_end());
}
