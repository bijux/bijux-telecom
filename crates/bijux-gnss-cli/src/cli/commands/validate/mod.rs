use super::*;

mod artifact_validation;
mod capture_validation;
mod evidence_bundle;
mod observation_validation;
mod schema_validation;
mod science_policy;

pub(crate) use artifact_validation::handle_validate_artifacts;
pub(crate) use capture_validation::handle_validate_capture;
pub(crate) use evidence_bundle::validation_evidence_bundle;
pub(crate) use observation_validation::handle_validate;
pub(crate) use schema_validation::{
    CsvType, validate_config_schema, validate_csv_schema, validate_json_schema,
    validate_jsonl_schema, validate_sidecar_schema,
};
pub(crate) use science_policy::validation_science_policy;
#[cfg(test)]
mod tests;

pub(crate) fn handle_validate_reference(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateReference { common, run_dir, reference, align } = command else {
        bail!("invalid command for handler");
    };

    let _ = runtime_config_from_env(&common, None);
    let artifacts = run_dir.join("artifacts");
    let obs_path = artifacts.join("obs.jsonl");
    let nav_path = artifacts.join("pvt.jsonl");
    let obs = read_obs_epochs(&obs_path)?;
    let solutions = read_nav_solutions(&nav_path)?;
    let reference_epochs = read_reference_epochs(&reference)?;
    let align_policy = match align {
        ReferenceAlign::Nearest => bijux_gnss_infra::api::ReferenceAlign::Nearest,
        ReferenceAlign::Linear => bijux_gnss_infra::api::ReferenceAlign::Linear,
    };
    let aligned =
        bijux_gnss_infra::api::validate_reference(&solutions, &reference_epochs, align_policy)?;

    let report = build_validation_report(
        &[],
        &obs,
        &solutions,
        &aligned,
        0.0,
        false,
        vec!["run_dir_only".to_string()],
        bijux_gnss_infra::api::receiver::ValidationSciencePolicy::default(),
    )?;
    let out_dir = artifacts_dir(&common, "validate_reference", None)?;
    write_melbourne_wubbena_diagnostics(&out_dir, &obs)?;
    let out = out_dir.join("validation_report.json");
    fs::write(&out, serde_json::to_string_pretty(&report)?)?;
    let evidence = validation_evidence_bundle(&obs, &solutions, &report);
    let evidence_path = out_dir.join("validation_evidence_bundle.json");
    fs::write(&evidence_path, serde_json::to_string_pretty(&evidence)?)?;
    let summary = serde_json::json!({ "report": out.display().to_string() });
    write_manifest(&common, "validate_reference", &ReceiverConfig::default(), None, &summary)?;
    Ok(())
}
