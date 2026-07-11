#![allow(missing_docs)]

use std::fs;
use std::path::Path;

#[path = "support/navigation_pvt_truth_table.rs"]
mod navigation_pvt_truth_table;
#[path = "support/navigation_pipeline.rs"]
mod navigation_pipeline;

use bijux_gnss_receiver::api::sim::SyntheticPvtTruthTableReport;
use serde_json::Value;

use navigation_pvt_truth_table::build_pvt_truth_table_fixture;

#[test]
fn navigation_pvt_truth_table_matches_reference_fixture() {
    let fixture = build_pvt_truth_table_fixture("clean_synthetic_navigation_pvt_truth", 0.0);
    let expected = load_truth_table_fixture("pvt_truth_table_reference.json");

    assert_json_close(
        &serde_json::to_value(&fixture.report).expect("report value"),
        &serde_json::to_value(&expected).expect("fixture value"),
        1.0e-8,
        "$",
    );
}

fn load_truth_table_fixture(fixture_file: &str) -> SyntheticPvtTruthTableReport {
    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("tests/data/navigation/{fixture_file}"));
    let contents = fs::read_to_string(path).expect("truth table fixture");
    serde_json::from_str(&contents).expect("valid truth table fixture")
}

fn assert_json_close(actual: &Value, expected: &Value, tolerance: f64, path: &str) {
    match (actual, expected) {
        (Value::Null, Value::Null) => {}
        (Value::Bool(actual), Value::Bool(expected)) => {
            assert_eq!(actual, expected, "boolean mismatch at {path}");
        }
        (Value::String(actual), Value::String(expected)) => {
            assert_eq!(actual, expected, "string mismatch at {path}");
        }
        (Value::Number(actual), Value::Number(expected)) => {
            let actual = actual.as_f64().expect("numeric json");
            let expected = expected.as_f64().expect("numeric json");
            let error = (actual - expected).abs();
            assert!(
                error <= tolerance,
                "numeric mismatch at {path}: actual={actual:.15} expected={expected:.15} error={error:.15} tolerance={tolerance:.15}"
            );
        }
        (Value::Array(actual), Value::Array(expected)) => {
            assert_eq!(actual.len(), expected.len(), "array length mismatch at {path}");
            for (index, (actual_item, expected_item)) in
                actual.iter().zip(expected.iter()).enumerate()
            {
                assert_json_close(
                    actual_item,
                    expected_item,
                    tolerance,
                    &format!("{path}[{index}]"),
                );
            }
        }
        (Value::Object(actual), Value::Object(expected)) => {
            assert_eq!(actual.len(), expected.len(), "object field count mismatch at {path}");
            for (key, expected_value) in expected {
                let actual_value =
                    actual.get(key).unwrap_or_else(|| panic!("missing field at {path}.{key}"));
                assert_json_close(
                    actual_value,
                    expected_value,
                    tolerance,
                    &format!("{path}.{key}"),
                );
            }
        }
        _ => panic!("type mismatch at {path}: actual={actual:?} expected={expected:?}"),
    }
}
