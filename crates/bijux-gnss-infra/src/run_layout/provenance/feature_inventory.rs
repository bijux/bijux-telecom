//! Build feature inventory helpers.

pub(crate) fn enabled_features() -> Vec<String> {
    let mut features = Vec::new();
    if let Ok(value) = std::env::var("BIJUX_GNSS_FEATURES") {
        for item in value.split(',') {
            let trimmed = item.trim();
            if !trimmed.is_empty() {
                features.push(trimmed.to_string());
            }
        }
    }
    features
}
