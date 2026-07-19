//! Run timestamp helpers.

pub(crate) fn now_unix_ms(deterministic: bool) -> u128 {
    if deterministic {
        return 0;
    }
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis()
}
