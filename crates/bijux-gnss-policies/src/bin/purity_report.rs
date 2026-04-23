#![forbid(unsafe_code)]

use regex::Regex;
use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

fn workspace_root() -> PathBuf {
    let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    crate_dir.parent().and_then(|p| p.parent()).expect("workspace root").to_path_buf()
}

fn read_toml(path: &Path) -> toml::Value {
    let content = fs::read_to_string(path).expect("read Cargo.toml");
    toml::from_str(&content).expect("parse Cargo.toml")
}

fn pub_item_counts(src_dir: &Path) -> (usize, usize) {
    let pub_re =
        Regex::new(r"^\s*pub\s+(struct|enum|fn|type|trait|const|static|use|mod)\b").expect("regex");
    let pub_crate_re = Regex::new(r"^\s*pub\s*\(crate\)\b").expect("regex");
    let mut api_pub = 0usize;
    let mut non_api_pub = 0usize;
    let mut stack = vec![src_dir.to_path_buf()];
    while let Some(dir) = stack.pop() {
        if let Ok(entries) = fs::read_dir(&dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    stack.push(path);
                    continue;
                }
                if path.extension().and_then(|s| s.to_str()) != Some("rs") {
                    continue;
                }
                let content = fs::read_to_string(&path).unwrap_or_default();
                let is_api = path.file_name().and_then(|s| s.to_str()) == Some("api.rs");
                for line in content.lines() {
                    if !pub_re.is_match(line) || pub_crate_re.is_match(line) {
                        continue;
                    }
                    if is_api {
                        api_pub += 1;
                    } else {
                        non_api_pub += 1;
                    }
                }
            }
        }
    }
    (api_pub, non_api_pub)
}

fn main() {
    let root = workspace_root();
    let crates_dir = root.join("crates");
    let heavy_deps: BTreeSet<&str> =
        ["tokio", "reqwest", "hyper", "jsonschema"].into_iter().collect();

    println!("purity-report v1");
    for entry in fs::read_dir(&crates_dir).expect("read crates dir") {
        let entry = entry.expect("crate entry");
        let manifest = entry.path().join("Cargo.toml");
        if !manifest.exists() {
            continue;
        }
        let data = read_toml(&manifest);
        let name = data
            .get("package")
            .and_then(|v| v.get("name"))
            .and_then(|v| v.as_str())
            .unwrap_or("unknown");
        let deps = data
            .get("dependencies")
            .and_then(|v| v.as_table())
            .map(|t| t.keys().cloned().collect::<BTreeSet<String>>())
            .unwrap_or_default();
        let heavy_present: Vec<_> =
            deps.iter().filter(|d| heavy_deps.contains(d.as_str())).cloned().collect();
        let features = data
            .get("features")
            .and_then(|v| v.as_table())
            .map(|t| t.keys().cloned().collect::<BTreeSet<String>>())
            .unwrap_or_default();

        let src_dir = entry.path().join("src");
        let (api_pub, non_api_pub) =
            if src_dir.exists() { pub_item_counts(&src_dir) } else { (0, 0) };

        println!("crate: {}", name);
        println!("deps: {}", deps.len());
        if heavy_present.is_empty() {
            println!("heavy-deps: none");
        } else {
            println!("heavy-deps: {}", heavy_present.join(","));
        }
        println!("pub-items: api={} non-api={}", api_pub, non_api_pub);
        if features.is_empty() {
            println!("features: none");
        } else {
            println!("features: {}", features.into_iter().collect::<Vec<_>>().join(","));
        }
        println!("---");
    }
}
