use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

fn collect_rs_files(dir: &Path, out: &mut Vec<PathBuf>) {
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

fn tokenize(content: &str) -> BTreeSet<String> {
    content
        .split(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
        .filter(|token| !token.is_empty())
        .map(|token| token.to_string())
        .collect()
}

fn extract_pub_item_name(line: &str, keyword: &str) -> Option<String> {
    let needle = format!("pub {keyword} ");
    let idx = line.find(&needle)?;
    let rest = &line[idx + needle.len()..];
    let name = rest.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_')).next().unwrap_or("");
    if name.is_empty() {
        None
    } else {
        Some(name.to_string())
    }
}

#[test]
fn public_structs_and_fns_live_in_api() {
    let crate_root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let api_path = crate_root.join("api.rs");
    let api_content = fs::read_to_string(&api_path).expect("read api.rs");
    let allowlist = tokenize(&api_content);

    let mut files = Vec::new();
    collect_rs_files(&crate_root, &mut files);
    for path in files {
        if path.file_name().and_then(|n| n.to_str()) == Some("api.rs") {
            continue;
        }
        let content = fs::read_to_string(&path).expect("read source");
        let mut in_impl = false;
        let mut brace_depth: i32 = 0;
        let mut saw_impl_brace = false;
        for line in content.lines() {
            let trimmed = line.trim_start();
            if trimmed.starts_with("impl ") || trimmed.starts_with("impl<") {
                in_impl = true;
            }
            let opens = line.chars().filter(|c| *c == '{').count() as i32;
            let closes = line.chars().filter(|c| *c == '}').count() as i32;
            if in_impl {
                if opens > 0 {
                    saw_impl_brace = true;
                }
                brace_depth += opens - closes;
                if saw_impl_brace && brace_depth <= 0 {
                    in_impl = false;
                    saw_impl_brace = false;
                    brace_depth = 0;
                }
            }
            if line.contains("pub struct ") {
                if let Some(name) = extract_pub_item_name(line, "struct") {
                    assert!(
                        allowlist.contains(&name),
                        "public struct `{}` must be re-exported in api.rs (found in {})",
                        name,
                        path.display()
                    );
                }
            }
            if !in_impl && line.contains("pub fn ") {
                if let Some(name) = extract_pub_item_name(line, "fn") {
                    assert!(
                        allowlist.contains(&name),
                        "public fn `{}` must be re-exported in api.rs (found in {})",
                        name,
                        path.display()
                    );
                }
            }
        }
    }
}
