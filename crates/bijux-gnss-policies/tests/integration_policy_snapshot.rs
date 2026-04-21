#![allow(missing_docs)]
use std::path::PathBuf;

use bijux_gnss_policies::api::GuardrailConfig;

#[test]
fn guardrail_default_policy_snapshot() {
    let config = GuardrailConfig::default();
    let mut serialized: serde_json::Value =
        serde_json::to_value(&config).expect("serialize config to value");
    if let Some(map) = serialized.as_object_mut() {
        map.remove("max_loc");
        map.remove("max_modules_per_dir");
        map.remove("max_rs_files_per_dir");
    }
    let serialized = serde_json::to_string_pretty(&serialized).expect("serialize config");
    let snapshot = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("snapshots")
        .join("guardrail_default.json");
    let expected = std::fs::read_to_string(&snapshot).expect("read snapshot");
    let mut expected: serde_json::Value =
        serde_json::from_str(&expected).expect("parse snapshot value");
    if let Some(map) = expected.as_object_mut() {
        map.remove("max_loc");
        map.remove("max_modules_per_dir");
        map.remove("max_rs_files_per_dir");
    }
    let expected = serde_json::to_string_pretty(&expected).expect("serialize snapshot value");
    assert_eq!(serialized.trim_end(), expected.trim_end());
}
