#![allow(missing_docs)]

use std::fs;
use std::path::Path;

#[test]
fn no_anyhow_eyre_outside_cli() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join("..");
    let crates_dir = root.join("crates");
    let allow = ["bijux-gnss-cli"];
    let mut offenders = Vec::new();
    for entry in fs::read_dir(&crates_dir).expect("read crates dir") {
        let entry = entry.expect("entry");
        let path = entry.path();
        if !path.is_dir() {
            continue;
        }
        let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("");
        if allow.contains(&name) {
            continue;
        }
        let cargo_toml = path.join("Cargo.toml");
        if !cargo_toml.exists() {
            continue;
        }
        let data = fs::read_to_string(&cargo_toml).expect("read Cargo.toml");
        if data.contains("anyhow") || data.contains("eyre") {
            offenders.push(name.to_string());
        }
    }
    assert!(
        offenders.is_empty(),
        "anyhow/eyre found outside CLI: {:?}",
        offenders
    );
}
