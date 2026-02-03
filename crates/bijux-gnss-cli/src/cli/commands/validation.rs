fn handle_validateartifacts(command: GnssCommand) -> Result<()> {
    let GnssCommand::ValidateArtifacts {
                common,
                obs,
                eph,
                strict,
            } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    if obs.is_none() && eph.is_none() {
                        bail!("--obs and/or --eph is required");
                    }
                    if let Some(path) = obs {
                        validate_jsonl_schema(&schema_path("obs_epoch.schema.json"), &path, strict)?;
                        println!("obs ok: {}", path.display());
                    }
                    if let Some(path) = eph {
                        validate_json_schema(&schema_path("gps_ephemeris.schema.json"), &path, strict)?;
                        println!("ephemeris ok: {}", path.display());
                    }

    Ok(())
}


fn handle_validate(command: GnssCommand) -> Result<()> {
    let GnssCommand::Validate {
                common,
                file,
                eph,
                reference,
                prn,
                sp3,
                clk,
            } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let dataset = load_dataset(&common)?;
                    let mut profile = load_profile(&common)?;
                    apply_common_overrides(&mut profile, &common);
                    if let Some(entry) = &dataset {
                        profile.sample_rate_hz = entry.sample_rate_hz;
                        profile.intermediate_freq_hz = entry.intermediate_freq_hz;
                    }
                    profile
                        .validate()
                        .map_err(|errs| eyre!("invalid config: {}", errs.join(", ")))?;
                    validate_config_schema(&profile)?;
                    let config = profile.to_receiver_config();
                    let input_file = resolve_input_file(file.as_ref(), dataset.as_ref())?;
                    let sidecar = load_sidecar(common.sidecar.as_ref())?;
                    let frame = load_frame(&input_file, &config, 0, sidecar.as_ref())?;
    
                    let acquisition = Acquisition::new(config.clone());
                    let sats = prns_to_sats(&prn);
                    let acquisitions = acquisition.run_fft(&frame, &sats);
                    let tracking = bijux_gnss_receiver::tracking::Tracking::new(config.clone());
                    let tracks = tracking.track_from_acquisition(
                        &frame,
                        &acquisitions,
                        bijux_gnss_core::SignalBand::L1,
                    );
                    let obs = bijux_gnss_receiver::observations::observations_from_tracking_results(
                        &config,
                        &tracks,
                        profile.navigation.hatch_window,
                    );
                    let ephs = read_ephemeris(&eph)?;
                    let mut products_diag = bijux_gnss_nav::ProductDiagnostics::new();
                    let mut products_ok = false;
                    let mut products = bijux_gnss_nav::Products::new(
                        bijux_gnss_nav::BroadcastProductsProvider::new(ephs.clone()),
                    );
                    if sp3.is_some() || clk.is_some() {
                        if let Some(sp3_path) = sp3 {
                            let data = fs::read_to_string(sp3_path)?;
                            let sp3: bijux_gnss_nav::Sp3Provider = data
                                .parse()
                                .map_err(|err| eyre!("SP3 parse failed: {err}"))?;
                            products = products.with_sp3(sp3);
                        } else {
                            products_diag.fallback("SP3 not provided");
                        }
                        if let Some(clk_path) = clk {
                            let data = fs::read_to_string(clk_path)?;
                            let clk: bijux_gnss_nav::ClkProvider = data
                                .parse()
                                .map_err(|err| eyre!("CLK parse failed: {err}"))?;
                            products = products.with_clk(clk);
                        } else {
                            products_diag.fallback("CLK not provided");
                        }
                        if let Some(first_epoch) = obs.first() {
                            if let Some(first_sat) = first_epoch.sats.first() {
                                let sat = first_sat.signal_id.sat;
                                let t = first_epoch.t_rx_s;
                                let state = products.sat_state(sat, t, &mut products_diag);
                                let clock = products.clock_bias_s(sat, t, &mut products_diag);
                                products_ok = state.is_some()
                                    && clock.is_some()
                                    && products_diag.fallbacks.is_empty();
                            }
                        }
                    } else {
                        products_diag.fallback("products not provided");
                    }
                    let mut nav = bijux_gnss_receiver::navigation::Navigation::new(config.clone());
    
                    let mut solutions = Vec::new();
                    for epoch in &obs {
                        if let Some(solution) = nav.solve_epoch(epoch, &ephs) {
                            solutions.push(solution);
                        }
                    }
    
                    let reference_epochs = read_reference_epochs(&reference)?;
                    let report = build_validation_report(
                        &tracks,
                        &obs,
                        &solutions,
                        &reference_epochs,
                        profile.sample_rate_hz,
                        products_ok,
                        products_diag.fallbacks.clone(),
                    )?;
    
                    let out_dir = common
                        .out
                        .clone()
                        .context("--out is required for validate")?;
                    fs::create_dir_all(&out_dir)?;
                    let path = out_dir.join("validation_report.json");
                    fs::write(&path, serde_json::to_string_pretty(&report)?)?;
                    println!("wrote {}", path.display());
    
                    let (rows, stats) = reference_compare(&solutions, &reference_epochs);
                    let compare_path = out_dir.join("reference_compare.json");
                    fs::write(&compare_path, serde_json::to_string_pretty(&stats)?)?;
                    let compare_csv = out_dir.join("reference_compare.csv");
                    fs::write(&compare_csv, rows.join("\n"))?;
    
                    if profile.navigation.ppp.enabled {
                        let ppp_config = build_ppp_config(&profile);
                        let mut ppp = PppFilter::new(ppp_config);
                        let mut ppp_solutions = Vec::new();
                        let mut checkpoint_path = None;
                        for epoch in &obs {
                            if let Some(sol) = ppp.solve_epoch(epoch, &ephs, &products) {
                                ppp_solutions.push(sol);
                            }
                            if profile.navigation.ppp.checkpoint_interval_epochs > 0
                                && epoch.epoch_idx % profile.navigation.ppp.checkpoint_interval_epochs
                                    == 0
                            {
                                let ck = ppp.checkpoint();
                                let path = out_dir.join("ppp_checkpoint.json");
                                fs::write(&path, serde_json::to_string_pretty(&ck)?)?;
                                checkpoint_path = Some(path.display().to_string());
                            }
                        }
                        let mut ppp_report = ppp_evaluation_report(&ppp_solutions, &reference_epochs);
                        ppp_report.checkpoint_path = checkpoint_path;
                        let ppp_path = out_dir.join("ppp_report.json");
                        fs::write(&ppp_path, serde_json::to_string_pretty(&ppp_report)?)?;
                    }

    Ok(())
}


fn handle_doctor(command: GnssCommand) -> Result<()> {
    let GnssCommand::Doctor { common, file } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    println!("bijux-gnss doctor");
                    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
                    {
                        println!("sse2: {}", std::is_x86_feature_detected!("sse2"));
                        println!("avx2: {}", std::is_x86_feature_detected!("avx2"));
                    }
                    let mut sample_rate_hz = 5_000_000.0;
                    match load_profile(&common) {
                        Ok(config) => {
                            let _ = validate_config_schema(&config);
                            match config.validate() {
                                Ok(()) => println!("config: ok"),
                                Err(errs) => println!("config errors: {}", errs.join(", ")),
                            }
                            sample_rate_hz = config.sample_rate_hz;
                        }
                        Err(err) => println!("config load error: {}", err),
                    }
                    let leap = bijux_gnss_core::LeapSeconds::default_table();
                    match leap.validate() {
                        Ok(()) => println!("leap seconds: ok (offset {})", leap.latest_offset()),
                        Err(err) => println!("leap seconds: {err}"),
                    }
                    if let Some(path) = file {
                        let readable =
                            FileSamples::open(path.to_str().unwrap_or_default(), 0, sample_rate_hz);
                        println!("dataset readable: {}", readable.is_ok());
                    }
                    println!("fft backend: rustfft");

    Ok(())
}
