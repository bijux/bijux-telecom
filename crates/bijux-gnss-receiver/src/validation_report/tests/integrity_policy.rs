use super::*;

#[test]
fn science_fixture_integrity_classes_are_deterministic() {
    let fixture_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../bijux-gnss-receiver/tests/data/validation/science_integrity_classification.json");
    let cases: Vec<ScienceFixtureCase> =
        serde_json::from_str(&std::fs::read_to_string(fixture_path).expect("fixture"))
            .expect("cases");
    for (idx, case) in cases.iter().enumerate() {
        let solution = with_integrity_support(fixture_solution(
            idx as u64,
            case.pdop,
            case.rms_m,
            case.used_sat_count,
        ));
        let track = fixture_track(idx as u64, case.lock_ratio);
        let report = build_validation_report(
            &[track],
            &[],
            &[solution],
            &[],
            1.0,
            false,
            Vec::new(),
            ValidationSciencePolicy::default(),
        )
        .expect("report");
        let class = report.integrity.first().expect("integrity row");
        let expected = match case.expected_class.as_str() {
            "missing_integrity_evidence" => NavIntegrityClass::IntegrityEvidenceMissing,
            "weak_geometry" => NavIntegrityClass::WeakGeometry,
            "suspicious_residuals" => NavIntegrityClass::SuspiciousResiduals,
            "unstable_lock" => NavIntegrityClass::UnstableLock,
            other => panic!("unsupported expected class: {other}"),
        };
        assert_eq!(class.class, expected, "case {}", case.name);
    }
}

#[test]
fn validation_policy_marks_high_gdop_as_weak_geometry() {
    let solution = with_integrity_support(fixture_solution(0, 2.0, 1.0, 4));
    let report = build_validation_report(
        &[fixture_track(0, 1.0)],
        &[],
        &[NavSolutionEpoch { gdop: Some(20.0), ..solution }],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("report");

    assert_eq!(
        report.integrity.first().expect("integrity row").class,
        NavIntegrityClass::WeakGeometry
    );
}

#[test]
fn validation_policy_requires_integrity_evidence_for_nominal_classification() {
    let solution = fixture_solution(0, 2.0, 1.0, 4);
    let report = build_validation_report(
        &[fixture_track(0, 1.0)],
        &[],
        &[solution],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("report");

    let integrity = report.integrity.first().expect("integrity row");
    assert_eq!(integrity.class, NavIntegrityClass::IntegrityEvidenceMissing);
    assert_eq!(integrity.reasons, vec!["missing_integrity_evidence".to_string()]);
}

#[test]
fn validation_policy_allows_nominal_when_integrity_evidence_is_present() {
    let solution = with_integrity_support(fixture_solution(0, 2.0, 1.0, 4));
    let report = build_validation_report(
        &[fixture_track(0, 1.0)],
        &[],
        &[solution],
        &[],
        1.0,
        false,
        Vec::new(),
        ValidationSciencePolicy::default(),
    )
    .expect("report");

    let integrity = report.integrity.first().expect("integrity row");
    assert_eq!(integrity.class, NavIntegrityClass::Nominal);
    assert!(integrity.reasons.is_empty());
}
