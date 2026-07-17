use super::*;

pub(crate) fn handle_validate(command: GnssCommand) -> Result<()> {
    let GnssCommand::Validate { args } = command else {
        bail!("invalid command for handler");
    };
    let ObservationValidationArgs {
        common,
        input,
        eph,
        reference,
        prn,
        sp3,
        clk,
        bias_sinex,
    } = args;
    let RawCaptureInputArgs { file } = input;

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
            report.errors
                .iter()
                .map(|error| error.message.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        );
    }

    let mut obs = read_obs_epochs(&file)?;
    let broadcast_navigation = read_broadcast_navigation_data(&eph)?;
    let reference_epochs = read_reference_epochs(&reference)?;

    if !prn.is_empty() {
        obs.iter_mut().for_each(|epoch| {
            epoch.sats.retain(|sat| prn.contains(&sat.signal_id.sat.prn));
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
                .map_err(|error| eyre!("sp3 parse error: {}", error))?;
            products = products.with_sp3(sp3);
        }
        if let Some(path) = clk {
            let data = fs::read_to_string(path)?;
            let clk = data
                .parse::<bijux_gnss_infra::api::nav::ClkProvider>()
                .map_err(|error| eyre!("clk parse error: {}", error))?;
            products = products.with_clk(clk);
        }
        if let Some(path) = bias_sinex {
            let data = fs::read_to_string(path)?;
            let bias_sinex = data
                .parse::<bijux_gnss_infra::api::nav::BiasSinexProvider>()
                .map_err(|error| eyre!("bias sinex parse error: {}", error))?;
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
