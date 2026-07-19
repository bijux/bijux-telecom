use std::path::{Path, PathBuf};

use bijux_gnss_receiver::api::core::InputError;
use bijux_gnss_signal::api::RawIqMetadata;

use crate::datasets::DatasetEntry;

pub(super) fn resolve_raw_iq_metadata(
    dataset: Option<&DatasetEntry>,
    explicit_sidecar: Option<&Path>,
) -> Result<RawIqMetadata, InputError> {
    let dataset_metadata = dataset.and_then(DatasetEntry::raw_iq_metadata);
    let sidecar_path = explicit_sidecar
        .map(Path::to_path_buf)
        .or_else(|| dataset.and_then(|entry| entry.sidecar.as_ref().map(PathBuf::from)));

    match (dataset_metadata, sidecar_path) {
        (Some(dataset_metadata), Some(sidecar_path)) => {
            let sidecar_metadata = super::load_raw_iq_metadata(&sidecar_path)?;
            super::validate_dataset_metadata_match(
                dataset.as_ref().copied(),
                &dataset_metadata,
                &sidecar_metadata,
            )?;
            Ok(sidecar_metadata)
        }
        (None, Some(sidecar_path)) => super::load_raw_iq_metadata(&sidecar_path),
        (Some(dataset_metadata), None) => {
            super::validate_raw_iq_metadata(&dataset_metadata)?;
            Ok(dataset_metadata)
        }
        (None, None) => Err(InputError {
            message: "raw IQ ingest requires explicit metadata via dataset registry or sidecar"
                .to_string(),
        }),
    }
}
