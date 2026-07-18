use std::fs;
use std::path::{Path, PathBuf};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

#[test]
fn root_make_entrypoint_loads_the_shared_rust_contract() {
    let root = repo_root();
    let root_make =
        fs::read_to_string(root.join("makes/root.mk")).expect("read repository Make entrypoint");
    let rust_make =
        fs::read_to_string(root.join("makes/rust.mk")).expect("read GNSS Rust Make policy");

    assert!(root_make.contains("BIJUX_MAKE_COMPONENTS := rust"));
    assert!(root_make.contains(".bijux/shared/bijux-makes/bijux.mk"));
    assert!(rust_make.contains("RUST_GATE_BIN ?= $(GNSS_RUST_GATE_BIN)"));
    assert!(rust_make.contains("NEXTEST_SLOW_NAME_EXPR ?= test(/::slow__/)"));
    assert!(rust_make.contains("RUST_AUDIT_PREREQUISITES += audit-policy-rs"));
}

#[test]
fn gnss_rust_adapter_delegates_to_the_managed_gate() {
    let root = repo_root();
    let adapter = fs::read_to_string(root.join("makes/bin/run_gnss_rust_gate.sh"))
        .expect("read GNSS Rust gate adapter");

    assert!(adapter.contains(".bijux/shared/bijux-makes-rs/scripts/rust_gate.sh"));
    assert!(adapter.contains("cargo run --locked -q -p bijux-gnss-dev -- audit-ignore-args"));
    assert!(adapter.contains("exec \"${shared_gate}\" \"$@\""));
}

#[test]
fn superseded_local_make_helpers_are_absent() {
    let root = repo_root();

    for relative_path in
        ["makes/_internal.mk", "makes/bin/nextest_expr.sh", "makes/bin/run_pinned_ref_gate.sh"]
    {
        assert!(
            !root.join(relative_path).exists(),
            "shared Make integration must not retain {relative_path}"
        );
    }
}
