#![allow(missing_docs)]
use std::path::Path;

use bijux_gnss_policies::api::GuardrailConfig;

#[test]
fn workspace_has_guardrails_tests() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let crates_dir = root.join("crates");
    for entry in std::fs::read_dir(&crates_dir).expect("read crates dir") {
        let entry = entry.expect("crate entry");
        let path = entry.path();
        if !path.is_dir() {
            continue;
        }
        if !path.join("Cargo.toml").exists() {
            continue;
        }
        let guardrails = path.join("tests").join("integration_guardrails.rs");
        let legacy = path.join("tests").join("guardrails.rs");
        let guardrails_path = if guardrails.exists() {
            guardrails
        } else {
            legacy
        };
        assert!(
            guardrails_path.exists(),
            "missing tests/integration_guardrails.rs in {}",
            path.display()
        );
        let content = std::fs::read_to_string(&guardrails_path).expect("read guardrails test");
        assert!(
            content.contains("GuardrailConfig::for_crate"),
            "guardrails test must use GuardrailConfig::for_crate in {}",
            guardrails_path.display()
        );
    }
}

#[test]
fn workspace_guardrail_defaults_not_increased() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let crates_dir = root.join("crates");
    let defaults = GuardrailConfig::default();
    for entry in std::fs::read_dir(&crates_dir).expect("read crates dir") {
        let entry = entry.expect("crate entry");
        let path = entry.path();
        if !path.is_dir() {
            continue;
        }
        let manifest = path.join("Cargo.toml");
        if !manifest.exists() {
            continue;
        }
        let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("");
        let config = GuardrailConfig::for_crate(name);
        let bad = config.max_depth > defaults.max_depth
            || config.max_pub_items_per_file > defaults.max_pub_items_per_file
            || config.max_pub_use_per_file > defaults.max_pub_use_per_file;
        assert!(
            !bad,
            "guardrails defaults increased for {}: {:?}",
            name, config
        );
    }
}
