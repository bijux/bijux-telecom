use std::path::{Path, PathBuf};

use walkdir::WalkDir;

use crate::guardrails::error::Result;

pub(crate) fn collect_rs_files(root: &Path) -> Result<Vec<PathBuf>> {
    let mut files = Vec::new();
    for entry in WalkDir::new(root) {
        let entry = entry?;
        if entry.file_type().is_file()
            && entry.path().extension().and_then(|segment| segment.to_str()) == Some("rs")
        {
            files.push(entry.into_path());
        }
    }
    Ok(files)
}
