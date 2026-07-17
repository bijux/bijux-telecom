use bijux_gnss_receiver::api::core::{ArtifactReadPolicy, InputError};

pub(super) fn validate_schema_version(schema_version: u32, kind: &str) -> Result<(), InputError> {
    if schema_version != ArtifactReadPolicy::LATEST {
        return Err(InputError {
            message: format!("unsupported {kind} schema_version {schema_version}"),
        });
    }

    Ok(())
}
