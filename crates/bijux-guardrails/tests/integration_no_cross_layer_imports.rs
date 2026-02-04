#![allow(missing_docs)]

use std::fs;
use std::path::Path;

#[test]
fn no_cross_layer_imports() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join("..");
    let nav_src = root.join("crates/bijux-gnss-nav/src");
    let signal_src = root.join("crates/bijux-gnss-signal/src");
    let receiver_src = root.join("crates/bijux-gnss-receiver/src");

    let nav_bad = find_string(&nav_src, "bijux_gnss_receiver");
    assert!(nav_bad.is_empty(), "nav imports receiver: {:?}", nav_bad);

    let signal_bad = find_string(&signal_src, "bijux_gnss_nav");
    assert!(
        signal_bad.is_empty(),
        "signal imports nav: {:?}",
        signal_bad
    );

    let receiver_bad = find_string(&receiver_src, "bijux_gnss_nav::estimation");
    assert!(
        receiver_bad.is_empty(),
        "receiver should not use nav internals: {:?}",
        receiver_bad
    );
    let receiver_formats = find_string(&receiver_src, "bijux_gnss_nav::formats");
    assert!(
        receiver_formats.is_empty(),
        "receiver should not use nav formats internals: {:?}",
        receiver_formats
    );

    let receiver_obs_writers = find_string(&receiver_src, "ObsEpoch {");
    let receiver_obs_writers = receiver_obs_writers
        .into_iter()
        .filter(|path| !path.ends_with("stages/observations.rs"))
        .collect::<Vec<_>>();
    assert!(
        receiver_obs_writers.is_empty(),
        "only observations builder may construct ObsEpoch: {:?}",
        receiver_obs_writers
    );
}

fn find_string(root: &Path, needle: &str) -> Vec<String> {
    let mut hits = Vec::new();
    for entry in walkdir::WalkDir::new(root) {
        let entry = match entry {
            Ok(entry) => entry,
            Err(_) => continue,
        };
        if !entry.file_type().is_file() {
            continue;
        }
        if entry.path().extension().and_then(|s| s.to_str()) != Some("rs") {
            continue;
        }
        if let Ok(data) = fs::read_to_string(entry.path()) {
            if data.contains(needle) {
                hits.push(entry.path().display().to_string());
            }
        }
    }
    hits
}
