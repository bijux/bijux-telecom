use std::fs;
use std::path::Path;

use bijux_gnss_receiver::api::{
    apply_downgrade_policy, evaluate_prerequisites, AdvancedMode, AdvancedPrerequisites,
    AdvancedRefusalClass, AdvancedSolutionClaim,
};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct AdvancedFixture {
    id: String,
    mode: String,
    prerequisites: AdvancedPrerequisites,
    claim: String,
    expect: AdvancedExpect,
}

#[derive(Debug, Deserialize)]
struct AdvancedExpect {
    ready: bool,
    refusal_class: Option<String>,
    downgraded: bool,
    status_prefix: String,
    final_claim: String,
}

#[test]
fn advanced_status_fixtures_are_deterministic() {
    for path in load_fixture_paths() {
        let fixture = load_fixture(&path);
        let mode = parse_mode(&fixture.mode);
        let claim = parse_claim(&fixture.claim);

        let first = evaluate_prerequisites(mode, &fixture.prerequisites);
        let second = evaluate_prerequisites(mode, &fixture.prerequisites);
        assert_eq!(first.ready, second.ready, "fixture {} readiness not deterministic", fixture.id);
        assert_eq!(
            first.refusal_class, second.refusal_class,
            "fixture {} refusal class not deterministic",
            fixture.id
        );

        let (status, downgraded, _reason, final_claim) =
            apply_downgrade_policy(mode, &first, claim);
        assert_eq!(first.ready, fixture.expect.ready, "fixture {} ready mismatch", fixture.id);
        assert_eq!(
            first.refusal_class.map(format_refusal),
            fixture.expect.refusal_class,
            "fixture {} refusal mismatch",
            fixture.id
        );
        assert_eq!(
            downgraded, fixture.expect.downgraded,
            "fixture {} downgraded mismatch",
            fixture.id
        );
        assert!(
            status.starts_with(&fixture.expect.status_prefix),
            "fixture {} status {} does not start with {}",
            fixture.id,
            status,
            fixture.expect.status_prefix
        );
        assert_eq!(
            format_claim(final_claim),
            fixture.expect.final_claim,
            "fixture {} final claim mismatch",
            fixture.id
        );
    }
}

fn parse_mode(value: &str) -> AdvancedMode {
    match value {
        "rtk" => AdvancedMode::Rtk,
        "ppp" => AdvancedMode::Ppp,
        _ => panic!("unsupported mode {value}"),
    }
}

fn parse_claim(value: &str) -> AdvancedSolutionClaim {
    match value {
        "fixed" => AdvancedSolutionClaim::Fixed,
        "float" => AdvancedSolutionClaim::Float,
        "scaffolding" => AdvancedSolutionClaim::Scaffolding,
        "fallback_nav" => AdvancedSolutionClaim::FallbackNav,
        _ => panic!("unsupported claim {value}"),
    }
}

fn format_claim(value: AdvancedSolutionClaim) -> String {
    match value {
        AdvancedSolutionClaim::Fixed => "fixed",
        AdvancedSolutionClaim::Float => "float",
        AdvancedSolutionClaim::Scaffolding => "scaffolding",
        AdvancedSolutionClaim::FallbackNav => "fallback_nav",
    }
    .to_string()
}

fn format_refusal(value: AdvancedRefusalClass) -> String {
    match value {
        AdvancedRefusalClass::IncompleteCorrections => "incomplete_corrections",
        AdvancedRefusalClass::MissingBaseline => "missing_baseline",
        AdvancedRefusalClass::UnsupportedModel => "unsupported_model",
        AdvancedRefusalClass::InsufficientGeometry => "insufficient_geometry",
        AdvancedRefusalClass::IncompleteAmbiguityState => "incomplete_ambiguity_state",
    }
    .to_string()
}

fn fixture_root() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/advanced")
}

fn load_fixture_paths() -> Vec<std::path::PathBuf> {
    let mut paths = fs::read_dir(fixture_root())
        .expect("read fixture directory")
        .filter_map(|entry| entry.ok().map(|item| item.path()))
        .filter(|path| path.extension().is_some_and(|ext| ext == "json"))
        .collect::<Vec<_>>();
    paths.sort();
    paths
}

fn load_fixture(path: &Path) -> AdvancedFixture {
    let raw = fs::read_to_string(path).expect("read fixture file");
    serde_json::from_str(&raw).expect("parse fixture")
}
