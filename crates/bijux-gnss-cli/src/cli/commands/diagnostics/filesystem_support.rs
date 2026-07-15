fn ensure_run_dir_exists(run_dir: &Path) -> Result<()> {
    if run_dir.exists() && run_dir.is_dir() {
        return Ok(());
    }
    Err(classified_error(
        CliErrorClass::OperatorMisconfiguration,
        format!("run directory not found: {}", run_dir.display()),
    ))
}

fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::Digest;
    let mut hasher = sha2::Sha256::new();
    hasher.update(bytes);
    hex::encode(hasher.finalize())
}

