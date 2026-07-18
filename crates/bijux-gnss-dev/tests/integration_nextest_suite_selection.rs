use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

#[test]
fn slow_roster_is_sorted_unique_and_resolves_to_test_functions() -> anyhow::Result<()> {
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let roster_path = repo_root.join("configs/rust/nextest-slow-roster.txt");
    let roster = load_slow_roster(&roster_path)?;

    let mut sorted = roster.clone();
    sorted.sort();
    sorted.dedup();
    assert_eq!(
        roster,
        sorted,
        "slow roster must stay sorted and unique: {}",
        roster_path.display()
    );

    let known_test_functions = collect_test_function_names(&repo_root.join("crates"))?;
    let missing = roster
        .iter()
        .filter(|entry| !roster_entry_maps_to_known_test(entry, &known_test_functions))
        .cloned()
        .collect::<Vec<_>>();
    assert!(
        missing.is_empty(),
        "slow roster entries must resolve to known test functions: {missing:?}"
    );

    let duplicated_named_slow =
        roster.iter().filter(|entry| entry.contains("slow__")).cloned().collect::<Vec<_>>();
    assert!(
        duplicated_named_slow.is_empty(),
        "slow roster must not duplicate slow__-prefixed tests: {duplicated_named_slow:?}"
    );

    Ok(())
}

#[test]
fn slow_roster_feeds_nextest_lane_expressions() -> anyhow::Result<()> {
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let roster_path = repo_root.join("configs/rust/nextest-slow-roster.txt");
    let roster = load_slow_roster(&roster_path)?;
    let fast_expr = nextest_expr(&repo_root, "fast")?;
    let slow_expr = nextest_expr(&repo_root, "slow")?;

    assert!(
        fast_expr.starts_with("not ("),
        "fast nextest expression must negate the governed slow lane: {fast_expr}"
    );
    assert!(
        slow_expr.contains("test(/::slow__/)"),
        "slow nextest expression must preserve legacy slow__ namespace selection: {slow_expr}"
    );

    for entry in roster {
        let escaped = regex::escape(&entry);
        assert!(
            slow_expr.contains(&escaped),
            "slow nextest expression must include roster entry: {entry}"
        );
        assert!(
            fast_expr.contains(&escaped),
            "fast nextest expression must exclude roster entry through the negated slow lane: {entry}"
        );
    }

    Ok(())
}

fn load_slow_roster(path: &Path) -> anyhow::Result<Vec<String>> {
    let content = fs::read_to_string(path)?;
    Ok(content
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .map(ToOwned::to_owned)
        .collect())
}

fn nextest_expr(repo_root: &Path, mode: &str) -> anyhow::Result<String> {
    let output = Command::new(repo_root.join("makes/bin/nextest_expr.sh")).arg(mode).output()?;
    assert!(
        output.status.success(),
        "nextest expression generator failed for mode {mode}: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    Ok(String::from_utf8(output.stdout)?.trim().to_string())
}

fn collect_test_function_names(root: &Path) -> anyhow::Result<BTreeSet<String>> {
    let mut names = BTreeSet::new();
    collect_from_directory(root, &mut names)?;
    Ok(names)
}

fn collect_from_directory(path: &Path, names: &mut BTreeSet<String>) -> anyhow::Result<()> {
    for entry in fs::read_dir(path)? {
        let entry = entry?;
        let entry_path = entry.path();
        let file_type = entry.file_type()?;
        if file_type.is_dir() {
            collect_from_directory(&entry_path, names)?;
            continue;
        }
        if file_type.is_file() && entry_path.extension().and_then(|ext| ext.to_str()) == Some("rs")
        {
            let content = fs::read_to_string(&entry_path)?;
            record_test_function_names(&content, names);
        }
    }
    Ok(())
}

fn record_test_function_names(source: &str, names: &mut BTreeSet<String>) {
    let mut pending_test_attr = false;
    for line in source.lines() {
        let trimmed = line.trim_start();
        if trimmed.starts_with("#[") {
            if trimmed.contains("test") {
                pending_test_attr = true;
            }
            continue;
        }
        if let Some(rest) = trimmed.strip_prefix("fn ") {
            if pending_test_attr {
                if let Some((name, _)) = rest.split_once('(') {
                    names.insert(name.trim().to_string());
                }
            }
            pending_test_attr = false;
            continue;
        }
        if let Some(rest) = trimmed.strip_prefix("async fn ") {
            if pending_test_attr {
                if let Some((name, _)) = rest.split_once('(') {
                    names.insert(name.trim().to_string());
                }
            }
            pending_test_attr = false;
            continue;
        }
        if !trimmed.is_empty() {
            pending_test_attr = false;
        }
    }
}

fn roster_entry_maps_to_known_test(entry: &str, known_test_functions: &BTreeSet<String>) -> bool {
    if known_test_functions.contains(entry) {
        return true;
    }
    entry
        .rsplit("::")
        .next()
        .is_some_and(|function_name| known_test_functions.contains(function_name))
}

#[test]
fn repo_root_points_at_workspace_root() {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    assert!(repo_root.join("Cargo.toml").exists());
    assert!(repo_root.join("configs/rust/nextest.toml").exists());
}
