#![allow(missing_docs)]
use std::fs;
use std::path::Path;

fn workspace_root() -> &'static Path {
    let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    crate_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("workspace root")
}

fn read_manifest(crate_name: &str) -> String {
    let path = workspace_root()
        .join("crates")
        .join(crate_name)
        .join("Cargo.toml");
    fs::read_to_string(path).expect("read Cargo.toml")
}

#[test]
fn dependency_direction_rules() {
    let crates_dir = workspace_root().join("crates");
    for entry in fs::read_dir(&crates_dir).expect("read crates dir") {
        let entry = entry.expect("crate entry");
        let name = entry.file_name().to_string_lossy().to_string();
        if name == "bijux-gnss-cli" {
            continue;
        }
        let manifest = entry.path().join("Cargo.toml");
        if !manifest.exists() {
            continue;
        }
        let contents = fs::read_to_string(&manifest).expect("read Cargo.toml");
        assert!(
            !contents.contains("bijux-gnss-cli"),
            "crate {} depends on bijux-gnss-cli",
            name
        );
    }

    let nav = read_manifest("bijux-gnss-nav");
    assert!(
        !nav.contains("bijux-gnss-receiver"),
        "bijux-gnss-nav must not depend on receiver"
    );
    assert!(
        !nav.contains("bijux-gnss-cli"),
        "bijux-gnss-nav must not depend on cli"
    );

    let signal = read_manifest("bijux-gnss-signal");
    assert!(
        !signal.contains("bijux-gnss-nav"),
        "bijux-gnss-signal must not depend on nav"
    );

    let core = read_manifest("bijux-gnss-core");
    let mut core_deps = Vec::new();
    for line in core.lines() {
        if line.contains("bijux-gnss-") && !line.contains("bijux-gnss-core") {
            core_deps.push(line.trim().to_string());
        }
    }
    let core_deps: Vec<_> = core_deps
        .into_iter()
        .filter(|line| !line.contains("bijux-guardrails"))
        .collect();
    assert!(
        core_deps.is_empty(),
        "bijux-gnss-core must not depend on other workspace crates (found: {:?})",
        core_deps
    );
}
