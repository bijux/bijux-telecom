//! Artifact kind detection helpers.

use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ArtifactKind {
    Acq,
    Track,
    Obs,
    Pvt,
}

impl ArtifactKind {
    pub(crate) fn detect(kind: Option<&str>, path: &Path) -> Option<Self> {
        kind.and_then(Self::from_label).or_else(|| Self::from_path(path))
    }

    pub(crate) fn from_path(path: &Path) -> Option<Self> {
        let name = path.file_name()?.to_string_lossy().to_lowercase();
        for (pattern, kind) in [
            ("obs", Self::Obs),
            ("track", Self::Track),
            ("acq", Self::Acq),
            ("pvt", Self::Pvt),
            ("nav", Self::Pvt),
        ] {
            if name.contains(pattern) {
                return Some(kind);
            }
        }
        None
    }

    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Self::Acq => "acq",
            Self::Track => "track",
            Self::Obs => "obs",
            Self::Pvt => "pvt",
        }
    }

    fn from_label(label: &str) -> Option<Self> {
        match label.to_lowercase().as_str() {
            "acq" => Some(Self::Acq),
            "track" => Some(Self::Track),
            "obs" => Some(Self::Obs),
            "pvt" | "nav" => Some(Self::Pvt),
            _ => None,
        }
    }
}
