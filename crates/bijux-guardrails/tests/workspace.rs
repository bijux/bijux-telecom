use std::path::Path;

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
        let guardrails = path.join("tests").join("guardrails.rs");
        assert!(
            guardrails.exists(),
            "missing tests/guardrails.rs in {}",
            path.display()
        );
    }
}
