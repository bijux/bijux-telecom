#![allow(missing_docs)]
use serde::Deserialize;
use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::Path;
use std::process::Command;

#[derive(Debug, Deserialize)]
struct Metadata {
    packages: Vec<Package>,
    workspace_members: Vec<String>,
}

#[derive(Debug, Deserialize)]
struct Package {
    name: String,
    id: String,
    dependencies: Vec<Dependency>,
}

#[derive(Debug, Deserialize)]
struct Dependency {
    name: String,
    kind: Option<String>,
}

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
    let metadata = load_metadata();
    let workspace = workspace_package_names(&metadata);
    let deps =
        workspace_dependency_edges(&metadata, &workspace, DependencyKindFilter::NormalAndBuild);

    let allowed: HashMap<&str, HashSet<&str>> = [
        ("bijux-gnss-cli", HashSet::from(["bijux-gnss-infra"])),
        ("bijux-gnss-infra", HashSet::from(["bijux-gnss-receiver"])),
        (
            "bijux-gnss-receiver",
            HashSet::from(["bijux-gnss-signal", "bijux-gnss-core", "bijux-gnss-nav"]),
        ),
        (
            "bijux-gnss",
            HashSet::from([
                "bijux-gnss-core",
                "bijux-gnss-signal",
                "bijux-gnss-receiver",
                "bijux-gnss-nav",
            ]),
        ),
        ("bijux-gnss-signal", HashSet::from(["bijux-gnss-core"])),
        ("bijux-gnss-nav", HashSet::from(["bijux-gnss-core"])),
        ("bijux-gnss-core", HashSet::new()),
        (
            "bijux-gnss-testkit",
            HashSet::from(["bijux-gnss-infra", "bijux-gnss-receiver"]),
        ),
        ("bijux-telecom-dev", HashSet::new()),
        ("bijux-gnss-policies", HashSet::new()),
    ]
    .into_iter()
    .collect();

    for (from, tos) in deps {
        let Some(allowed_tos) = allowed.get(from.as_str()) else {
            panic!("unexpected workspace crate in dependency graph: {}", from);
        };
        for to in tos {
            assert!(
                allowed_tos.contains(to.as_str()),
                "dependency not allowed by DAG: {} -> {}",
                from,
                to
            );
        }
    }

    for (from, allowed_tos) in &allowed {
        if !workspace.contains(*from) {
            continue;
        }
        let actual = deps_for(
            &metadata,
            &workspace,
            from,
            DependencyKindFilter::NormalAndBuild,
        );
        for to in actual {
            assert!(
                allowed_tos.contains(to.as_str()),
                "dependency not allowed by DAG: {} -> {}",
                from,
                to
            );
        }
    }
}

#[test]
fn no_cyclic_dev_dependencies() {
    let metadata = load_metadata();
    let workspace = workspace_package_names(&metadata);
    let edges = workspace_dependency_edges(&metadata, &workspace, DependencyKindFilter::All);
    let mut visiting = HashSet::new();
    let mut visited = HashSet::new();
    for node in workspace.iter() {
        if visited.contains(node) {
            continue;
        }
        dfs_detect_cycles(node, &edges, &mut visiting, &mut visited);
    }
}

#[test]
fn forbidden_feature_coupling() {
    let manifest = read_manifest("bijux-gnss-receiver");
    let receiver: toml::Value = toml::from_str(&manifest).expect("parse receiver Cargo.toml");
    let deps = receiver
        .get("dependencies")
        .and_then(|d| d.as_table())
        .expect("receiver dependencies");
    let nav = deps
        .get("bijux-gnss-nav")
        .expect("receiver depends on bijux-gnss-nav");
    if let Some(table) = nav.as_table() {
        let default_features = table
            .get("default-features")
            .and_then(|v| v.as_bool())
            .unwrap_or(true);
        assert!(
            !default_features,
            "receiver must disable default features for bijux-gnss-nav"
        );
        if let Some(features) = table.get("features").and_then(|v| v.as_array()) {
            for feature in features {
                let feature = feature.as_str().unwrap_or_default();
                assert_ne!(
                    feature, "precise-products",
                    "receiver must not enable nav/precise-products"
                );
            }
        }
    } else {
        panic!("bijux-gnss-nav dependency must be table form in receiver");
    }
}

#[derive(Copy, Clone)]
enum DependencyKindFilter {
    NormalAndBuild,
    All,
}

fn load_metadata() -> Metadata {
    let output = Command::new("cargo")
        .args(["metadata", "--format-version", "1", "--no-deps"])
        .current_dir(workspace_root())
        .output()
        .expect("cargo metadata");
    assert!(output.status.success(), "cargo metadata failed");
    serde_json::from_slice(&output.stdout).expect("parse cargo metadata")
}

fn workspace_package_names(metadata: &Metadata) -> HashSet<String> {
    let member_ids: HashSet<_> = metadata.workspace_members.iter().cloned().collect();
    metadata
        .packages
        .iter()
        .filter(|pkg| member_ids.contains(&pkg.id))
        .map(|pkg| pkg.name.clone())
        .collect()
}

fn workspace_dependency_edges(
    metadata: &Metadata,
    workspace: &HashSet<String>,
    kind: DependencyKindFilter,
) -> HashMap<String, HashSet<String>> {
    let mut edges: HashMap<String, HashSet<String>> = HashMap::new();
    for pkg in metadata.packages.iter() {
        if !workspace.contains(&pkg.name) {
            continue;
        }
        let deps = deps_for(metadata, workspace, &pkg.name, kind);
        if !deps.is_empty() {
            edges.insert(pkg.name.clone(), deps);
        }
    }
    edges
}

fn deps_for(
    metadata: &Metadata,
    workspace: &HashSet<String>,
    name: &str,
    kind: DependencyKindFilter,
) -> HashSet<String> {
    let pkg = metadata
        .packages
        .iter()
        .find(|pkg| pkg.name == name)
        .unwrap_or_else(|| panic!("missing package {}", name));
    pkg.dependencies
        .iter()
        .filter(|dep| {
            if !workspace.contains(&dep.name) {
                return false;
            }
            match kind {
                DependencyKindFilter::NormalAndBuild => {
                    dep.kind.as_deref().unwrap_or("normal") != "dev"
                }
                DependencyKindFilter::All => true,
            }
        })
        .map(|dep| dep.name.clone())
        .collect()
}

fn dfs_detect_cycles(
    node: &str,
    edges: &HashMap<String, HashSet<String>>,
    visiting: &mut HashSet<String>,
    visited: &mut HashSet<String>,
) {
    if visiting.contains(node) {
        panic!("dependency cycle detected including {}", node);
    }
    if visited.contains(node) {
        return;
    }
    visiting.insert(node.to_string());
    if let Some(nexts) = edges.get(node) {
        for next in nexts {
            dfs_detect_cycles(next, edges, visiting, visited);
        }
    }
    visiting.remove(node);
    visited.insert(node.to_string());
}
