use std::path::Path;

pub(super) fn resolve_registry_path(base_dir: &Path, value: &str) -> String {
    let path = Path::new(value);
    if path.is_absolute() {
        return value.to_string();
    }

    let anchor = match (base_dir.file_name(), base_dir.parent(), path.components().next()) {
        (Some(dir_name), Some(parent), Some(first_component))
            if first_component.as_os_str() == dir_name =>
        {
            parent
        }
        _ => base_dir,
    };

    anchor.join(path).to_string_lossy().into_owned()
}
