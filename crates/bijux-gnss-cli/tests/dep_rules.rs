use std::fs;
use std::path::Path;

fn workspace_root() -> &'static Path {
    let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    crate_dir
        .parent()
        .and_then(|p| p.parent())
        .expect("workspace root")
}

#[test]
fn cli_is_not_depended_on() {
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
}

#[test]
fn nav_does_not_depend_on_receiver() {
    let nav_manifest = workspace_root()
        .join("crates")
        .join("bijux-gnss-nav")
        .join("Cargo.toml");
    let contents = fs::read_to_string(nav_manifest).expect("read nav Cargo.toml");
    assert!(
        !contents.contains("bijux-gnss-receiver"),
        "bijux-gnss-nav must not depend on receiver"
    );
    assert!(
        !contents.contains("bijux-gnss-cli"),
        "bijux-gnss-nav must not depend on cli"
    );
}
