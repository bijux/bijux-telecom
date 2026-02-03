#![allow(missing_docs)]
use std::collections::BTreeSet;
use std::path::Path;

use bijux_gnss_core::DIAGNOSTIC_CODES;
use regex::Regex;
use walkdir::WalkDir;

#[test]
fn diagnostic_codes_are_registered() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let code_re = Regex::new(
        r#""(GNSS_[A-Z0-9_]+|NAV_[A-Z0-9_]+|TRACK_[A-Z0-9_]+|RTK_[A-Z0-9_]+|PPP_[A-Z0-9_]+)""#,
    )
    .unwrap();

    let registered: BTreeSet<String> = DIAGNOSTIC_CODES
        .iter()
        .map(|code| code.code.to_string())
        .collect();

    let mut found = BTreeSet::new();
    for entry in WalkDir::new(&root).into_iter().filter_map(Result::ok) {
        if !entry.file_type().is_file() {
            continue;
        }
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) != Some("rs") {
            continue;
        }
        let contents = match std::fs::read_to_string(path) {
            Ok(data) => data,
            Err(_) => continue,
        };
        for cap in code_re.captures_iter(&contents) {
            if let Some(code) = cap.get(1) {
                found.insert(code.as_str().to_string());
            }
        }
    }

    let missing: Vec<_> = found.difference(&registered).cloned().collect();
    assert!(
        missing.is_empty(),
        "diagnostic codes missing from registry: {missing:?}"
    );
}
