#![allow(missing_docs)]

use std::path::{Path, PathBuf};
use std::process::Command;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("workspace root")
        .to_path_buf()
}

fn run_bijux(args: &[&str], cwd: &Path) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_bijux"))
        .args(args)
        .current_dir(cwd)
        .output()
        .expect("run bijux")
}

#[test]
fn ca_code_prints_requested_chip_prefix() {
    let output = run_bijux(&["gnss", "ca-code", "--prn", "1", "--count", "5"], &repo_root());

    assert!(
        output.status.success(),
        "ca-code failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert_eq!(stdout.trim_end(), "-1 -1 1 1 -1");
}

#[test]
fn ca_code_wraps_across_period_boundary() {
    let output = run_bijux(
        &["gnss", "ca-code", "--prn", "1", "--start-chip", "1022", "--count", "4"],
        &repo_root(),
    );

    assert!(
        output.status.success(),
        "ca-code wrap query failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert_eq!(stdout.trim_end(), "1 -1 -1 1");
}

#[test]
fn ca_code_large_start_chip_wraps_to_same_sequence() {
    let output = run_bijux(
        &["gnss", "ca-code", "--prn", "1", "--start-chip", "2045", "--count", "4"],
        &repo_root(),
    );

    assert!(
        output.status.success(),
        "ca-code wrapped start query failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert_eq!(stdout.trim_end(), "1 -1 -1 1");
}

#[test]
fn ca_code_with_reference_reports_published_assignment() {
    let output = run_bijux(
        &[
            "gnss",
            "ca-code",
            "--prn",
            "1",
            "--start-chip",
            "2045",
            "--count",
            "4",
            "--with-reference",
        ],
        &repo_root(),
    );

    assert!(
        output.status.success(),
        "ca-code with reference failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("prn: 1\n"), "stdout did not report PRN metadata: {stdout}");
    assert!(
        stdout.contains("period_chips: 1023\n"),
        "stdout did not report period length: {stdout}"
    );
    assert!(
        stdout.contains("start_chip: 2045\n"),
        "stdout did not report requested start chip: {stdout}"
    );
    assert!(
        stdout.contains("wrapped_start_chip: 1022\n"),
        "stdout did not report wrapped start chip: {stdout}"
    );
    assert!(stdout.contains("g2_taps: 2 6\n"), "stdout did not report G2 taps: {stdout}");
    assert!(stdout.contains("g2_delay_chips: 5\n"), "stdout did not report G2 delay: {stdout}");
    assert!(
        stdout.contains("first_ten_chips_octal: 1440\n"),
        "stdout did not report first-ten-chip octal reference: {stdout}"
    );
    assert!(
        stdout.ends_with("1 -1 -1 1 \n"),
        "stdout did not end with requested chip sequence: {stdout}"
    );
}
