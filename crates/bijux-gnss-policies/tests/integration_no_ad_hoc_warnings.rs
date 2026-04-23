#![allow(missing_docs)]
use std::path::Path;

use walkdir::WalkDir;

#[test]
fn no_ad_hoc_warnings_in_core_nav_receiver() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let crates = ["crates/bijux-gnss-core", "crates/bijux-gnss-nav", "crates/bijux-gnss-receiver"];
    let forbidden = ["warn!(", "error!(", "log::warn", "log::error"];

    for crate_path in crates {
        let src = root.join(crate_path).join("src");
        for entry in WalkDir::new(&src).into_iter().filter_map(Result::ok) {
            if !entry.file_type().is_file() {
                continue;
            }
            if entry.path().extension().and_then(|s| s.to_str()) != Some("rs") {
                continue;
            }
            let contents = match std::fs::read_to_string(entry.path()) {
                Ok(data) => data,
                Err(_) => continue,
            };
            for token in &forbidden {
                if contents.contains(token) {
                    panic!("ad-hoc warning/error found ({}) in {}", token, entry.path().display());
                }
            }
        }
    }
}
