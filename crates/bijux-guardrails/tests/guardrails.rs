use std::path::Path;

use bijux_guardrails::{check, GuardrailConfig};

#[test]
fn guardrails() {
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let config = GuardrailConfig {
        max_loc: 500,
        max_depth: 4,
        max_modules_per_dir: 50,
        max_pub_items_per_file: 80,
        max_pub_use_per_file: 80,
        forbid_pub_use_spam: false,
        forbid_panic_expect: false,
        forbid_stage_id_strings: false,
        allow_panic_expect_paths: Vec::new(),
        allow_stage_id_paths: Vec::new(),
    };
    check(crate_root, &config).unwrap_or_else(|err| panic!("guardrails failed: {err}"));
}
