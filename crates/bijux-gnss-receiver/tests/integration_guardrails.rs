#![allow(missing_docs)]
use std::path::Path;

use bijux_gnss_policies::api::{check, GuardrailConfig};

#[test]
fn guardrails() {
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut config = GuardrailConfig::for_crate("bijux-gnss-receiver");
    config.max_loc = 700;
    config.max_modules_per_dir = 20;
    config.max_rs_files_per_dir = 20;
    config.purity_zones = vec!["/src/pipeline/".to_string()];
    config.purity_forbidden = vec![
        r"\bstd::env::".to_string(),
        r"\benv::var\b".to_string(),
        r"Instant::now".to_string(),
        r"SystemTime::now".to_string(),
        r"\bprintln!".to_string(),
        r"\beprintln!".to_string(),
        r"\bstd::fs::".to_string(),
        r"\bstd::fs\b".to_string(),
        r"\bFile::".to_string(),
        r"\bOpenOptions\b".to_string(),
    ];
    check(crate_root, &config).unwrap_or_else(|err| panic!("guardrails failed: {err}"));
}
