use super::*;

mod artifact_validation;
mod capture_validation;
mod evidence_bundle;
mod schema_validation;

pub(crate) use artifact_validation::handle_validate_artifacts;
pub(crate) use capture_validation::handle_validate_capture;
pub(crate) use evidence_bundle::validation_evidence_bundle;
pub(crate) use schema_validation::{
    CsvType, validate_config_schema, validate_csv_schema, validate_json_schema,
    validate_jsonl_schema, validate_sidecar_schema,
};
#[cfg(test)]
mod tests;

fn validation_science_policy(
    profile: &ReceiverConfig,
) -> bijux_gnss_infra::api::receiver::ValidationSciencePolicy {
    bijux_gnss_infra::api::receiver::ValidationSciencePolicy {
        min_mean_cn0_dbhz: profile.navigation.science_thresholds.min_mean_cn0_dbhz,
        max_pdop: profile.navigation.science_thresholds.max_pdop,
        max_gdop: profile.navigation.science_thresholds.max_gdop,
        max_residual_rms_m: profile.navigation.science_thresholds.max_residual_rms_m,
        min_used_satellites: profile.navigation.science_thresholds.min_used_satellites,
        min_lock_ratio: profile.navigation.science_thresholds.min_lock_ratio,
    }
}


pub(crate) fn handle_validate(command: GnssCommand) -> Result<()> {
    let GnssCommand::Validate { common, file, eph, reference, prn, sp3, clk, bias_sinex } = command else {
        bail!("invalid command for handler");
    };

    let runtime = runtime_config_from_env(&common, None);
    let file = file.context("--file is required for validation")?;
    let profile = load_config(&common)?;
    let dataset = load_dataset(&common)?;
    if profile.schema_version.0 != SchemaVersion::CURRENT.0 {
        bail!(
            "unsupported schema_version {}, expected {}",
            profile.schema_version.0,
            SchemaVersion::CURRENT.0
        );
    }

    validate_config_schema(&profile)?;

    let report = <ReceiverConfig as ValidateConfig>::validate(&profile);
    if !report.errors.is_empty() {
        bail!(
            "config invalid: {}",
            report.errors.iter().map(|e| e.message.as_str()).collect::<Vec<_>>().join(", ")
        );
    }

    let mut obs = read_obs_epochs(&file)?;
    let broadcast_navigation = read_broadcast_navigation_data(&eph)?;
    let reference_epochs = read_reference_epochs(&reference)?;

    if !prn.is_empty() {
        obs.iter_mut().for_each(|e| {
            e.sats.retain(|sat| prn.contains(&sat.signal_id.sat.prn));
        });
    }

    let receiver =
        bijux_gnss_infra::api::receiver::Receiver::new(profile.to_pipeline_config(), runtime);
    let solutions =
        receiver.solve_observation_epochs_with_gps_broadcast_navigation(&obs, &broadcast_navigation);

    #[cfg(feature = "precise-products")]
    let (products_ok, product_fallbacks, code_biases) = {
        let mut products = bijux_gnss_infra::api::nav::Products::new(
            bijux_gnss_infra::api::nav::BroadcastProductsProvider::new(
                broadcast_navigation.ephemerides.clone(),
            ),
        );
        if let Some(path) = sp3 {
            let data = fs::read_to_string(path)?;
            let sp3 = data
                .parse::<bijux_gnss_infra::api::nav::Sp3Provider>()
                .map_err(|e| eyre!("sp3 parse error: {}", e))?;
            products = products.with_sp3(sp3);
        }
        if let Some(path) = clk {
            let data = fs::read_to_string(path)?;
            let clk = data
                .parse::<bijux_gnss_infra::api::nav::ClkProvider>()
                .map_err(|e| eyre!("clk parse error: {}", e))?;
            products = products.with_clk(clk);
        }
        if let Some(path) = bias_sinex {
            let data = fs::read_to_string(path)?;
            let bias_sinex = data
                .parse::<bijux_gnss_infra::api::nav::BiasSinexProvider>()
                .map_err(|e| eyre!("bias sinex parse error: {}", e))?;
            products = products.with_dcb(bias_sinex);
        }
        let ok = products.sp3.is_some() || products.clk.is_some() || products.dcb.is_some();
        let fallbacks = if ok { Vec::new() } else { vec!["broadcast_only".to_string()] };
        (ok, fallbacks, products.dcb.clone())
    };
    #[cfg(not(feature = "precise-products"))]
    let (products_ok, product_fallbacks, code_biases) = {
        if sp3.is_some() || clk.is_some() || bias_sinex.is_some() {
            bail!(
                "precise-products feature disabled; recompile with feature to use SP3/CLK/Bias-SINEX"
            );
        }
        (false, vec!["precise_products_disabled".to_string()], None)
    };

    let out_dir = artifacts_dir(&common, "validate", dataset.as_ref())?;
    write_iono_free_code_artifact(
        &out_dir,
        &obs,
        code_biases
            .as_ref()
            .map(|provider| provider as &dyn bijux_gnss_infra::api::nav::CodeBiasProvider),
    )?;

    let report = build_validation_report(
        &[],
        &obs,
        &solutions,
        &reference_epochs,
        profile.sample_rate_hz,
        products_ok,
        product_fallbacks,
        validation_science_policy(&profile),
    )?;
    write_melbourne_wubbena_diagnostics(&out_dir, &obs)?;
    let out = out_dir.join("validation_report.json");
    fs::write(&out, serde_json::to_string_pretty(&report)?)?;
    let evidence = validation_evidence_bundle(&obs, &solutions, &report);
    let evidence_path = out_dir.join("validation_evidence_bundle.json");
    fs::write(&evidence_path, serde_json::to_string_pretty(&evidence)?)?;
    println!("wrote {}", out.display());
    println!("wrote {}", evidence_path.display());
    write_manifest(&common, "validate", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

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
