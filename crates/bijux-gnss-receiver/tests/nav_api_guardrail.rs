use std::fs;
use std::path::Path;

fn collect_rs_files(dir: &Path, out: &mut Vec<std::path::PathBuf>) {
    if let Ok(entries) = fs::read_dir(dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                collect_rs_files(&path, out);
            } else if path.extension().and_then(|s| s.to_str()) == Some("rs") {
                out.push(path);
            }
        }
    }
}

#[test]
fn receiver_does_not_use_nav_estimation_module() {
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut files = Vec::new();
    collect_rs_files(&crate_root, &mut files);
    for path in files {
        let content = fs::read_to_string(&path).expect("read source");
        assert!(
            !content.contains("use bijux_gnss_nav::estimation"),
            "found forbidden nav estimation import in {}",
            path.display()
        );
    }
}
