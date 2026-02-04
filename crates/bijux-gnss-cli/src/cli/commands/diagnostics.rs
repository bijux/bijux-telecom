fn handle_cacode(command: GnssCommand) -> Result<()> {
    let GnssCommand::CaCode { prn, count } = command else {
        bail!("invalid command for handler");
    };

    let code = generate_ca_code(Prn(prn));
                    let count = count.min(code.len());
                    for chip in code.iter().take(count) {
                        print!("{chip} ");
                    }
                    println!();

    Ok(())
}

fn handle_nav(command: GnssCommand) -> Result<()> {
    let GnssCommand::Nav { command } = command else {
        bail!("invalid command for handler");
    };
    let common = match &command {
        NavCommand::Decode { common, .. } => common,
    };
    let profile = load_profile(common)?;
    let dataset = load_dataset(common)?;

    match command {
                    NavCommand::Decode { common, track, prn } => {
                        set_trace_dir(&common);
                        let rows = read_tracking_dump(&track)?;
                        let mut prompt = Vec::new();
                        let target = SatId {
                            constellation: Constellation::Gps,
                            prn,
                        };
                        let mut sorted: Vec<_> = rows.into_iter().filter(|r| r.sat == target).collect();
                        sorted.sort_by_key(|r| r.epoch_idx);
                        for row in sorted {
                            prompt.push(row.prompt_i);
                        }
                        let bits = bijux_gnss_nav::bit_sync_from_prompt(&prompt);
                        let (mut ephs, stats) = bijux_gnss_nav::decode_subframes(&bits.bits);
                        for eph in ephs.iter_mut() {
                            eph.sat = target;
                        }
                        let report = NavDecodeReport {
                            sat: target,
                            preamble_hits: stats.preamble_hits,
                            parity_pass_rate: stats.parity_pass_rate,
                            ephemerides: ephs.clone(),
                        };
                        emit_report(&common, "nav_decode", &report)?;
                        write_ephemeris(&common, &ephs, &profile, dataset.as_ref())?;
                        write_manifest(&common, "nav_decode", &profile, dataset.as_ref(), &report)?;
                    }
                }

    Ok(())
}

fn handle_rtk(command: GnssCommand) -> Result<()> {
    let GnssCommand::Rtk {
                common,
                base_obs,
                rover_obs,
                eph,
                base_ecef,
                tolerance_s,
                ref_policy,
            } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
                    let profile = load_profile(&common)?;
                    let dataset = load_dataset(&common)?;
                    let header = artifact_header(&common, &profile, dataset.as_ref())?;
                    let base_epochs = read_obs_epochs(&base_obs)?;
                    let rover_epochs = read_obs_epochs(&rover_obs)?;
                    let ephs = read_ephemeris(&eph)?;
                    let base_xyz = parse_ecef(&base_ecef)?;
    
                    let mut aligner = bijux_gnss_receiver::rtk::EpochAligner::new(tolerance_s);
                    let aligned = aligner.align(&base_epochs, &rover_epochs);
    
                    let out_dir = artifacts_dir(&common, "rtk", dataset.as_ref())?;
    
                    let mut sd_lines = Vec::new();
                    let mut dd_lines = Vec::new();
                    let mut baseline_lines = Vec::new();
                    let mut baseline_quality_lines = Vec::new();
                    let mut fix_audit_lines = Vec::new();
                    let mut precision_lines = Vec::new();
                    let mut fix_state = bijux_gnss_receiver::rtk::FixState::default();
                    let fixer = bijux_gnss_receiver::rtk::NaiveFixer::new(
                        bijux_gnss_receiver::rtk::FixPolicy::default(),
                    );
                    let mut last_ref: Option<bijux_gnss_core::SigId> = None;
                    let mut ref_selector = bijux_gnss_receiver::rtk::RefSatSelector::new(5);
                    let mut ref_selectors: std::collections::BTreeMap<
                        Constellation,
                        bijux_gnss_receiver::rtk::RefSatSelector,
                    > = std::collections::BTreeMap::new();
    
                    for (base, rover) in &aligned {
                        let sd = bijux_gnss_receiver::rtk::build_sd(base, rover);
                        for item in &sd {
                            let wrapped = bijux_gnss_receiver::rtk::RtkSdEpochV1 {
                                header: header.clone(),
                                payload: item.clone(),
                            };
                            sd_lines.push(serde_json::to_string(&wrapped)?);
                        }
                        let dd = match ref_policy {
                            RefPolicy::Global => {
                                if let Some(ref_sig) = ref_selector.choose(&sd) {
                                    bijux_gnss_receiver::rtk::build_dd(&sd, ref_sig)
                                } else {
                                    Vec::new()
                                }
                            }
                            RefPolicy::PerConstellation => {
                                let refs =
                                    bijux_gnss_receiver::rtk::choose_ref_sat_per_constellation(&sd);
                                let mut chosen = std::collections::BTreeMap::new();
                                for (constellation, sig) in refs {
                                    let selector =
                                        ref_selectors.entry(constellation).or_insert_with(|| {
                                            bijux_gnss_receiver::rtk::RefSatSelector::new(5)
                                        });
                                    let subset: Vec<_> = sd
                                        .iter()
                                        .filter(|s| s.sig.sat.constellation == constellation)
                                        .cloned()
                                        .collect();
                                    if let Some(picked) = selector.choose(&subset) {
                                        chosen.insert(constellation, picked);
                                    } else {
                                        chosen.insert(constellation, sig);
                                    }
                                }
                                bijux_gnss_receiver::rtk::build_dd_per_constellation(&sd, &chosen)
                            }
                        };
                        for item in &dd {
                            let wrapped = bijux_gnss_receiver::rtk::RtkDdEpochV1 {
                                header: header.clone(),
                                payload: item.clone(),
                            };
                            dd_lines.push(serde_json::to_string(&wrapped)?);
                        }
                        let ref_sig = dd.first().map(|d| d.ref_sig);
                        let ref_changed = ref_sig != last_ref;
                        if ref_sig.is_some() {
                            last_ref = ref_sig;
                        }
    
                        let mut baseline = bijux_gnss_receiver::rtk::solve_baseline_dd(
                            &dd,
                            base_xyz,
                            &ephs,
                            rover.t_rx_s.0,
                        );
    
                        let float = bijux_gnss_receiver::rtk::FloatAmbiguitySolution {
                            ids: dd
                                .iter()
                                .map(|d| bijux_gnss_core::AmbiguityId {
                                    sig: d.sig,
                                    signal: format!("{:?}", d.sig.band),
                                })
                                .collect(),
                            float_cycles: dd.iter().map(|d| d.phase_cycles).collect(),
                            covariance: dd.iter().map(|d| vec![d.variance_phase]).collect(),
                        };
                        let (fix_result, audit) =
                            fixer.fix_with_state(rover.epoch_idx, &float, &mut fix_state);
                        let fix_audit = bijux_gnss_receiver::rtk::RtkFixAuditV1 {
                            header: header.clone(),
                            payload: audit.clone(),
                        };
                        fix_audit_lines.push(serde_json::to_string(&fix_audit)?);
    
                        if let Some(baseline_val) = baseline.take() {
                            let before_rms = bijux_gnss_receiver::rtk::dd_residual_metrics(
                                &dd,
                                base_xyz,
                                baseline_val.enu_m,
                                &ephs,
                            rover.t_rx_s.0,
                            )
                            .map(|(rms, _pred, _)| rms);
                            let mut adjusted = bijux_gnss_receiver::rtk::apply_fix_hold(
                                baseline_val,
                                fix_result.accepted,
                            );
                            let after_rms = bijux_gnss_receiver::rtk::dd_residual_metrics(
                                &dd,
                                base_xyz,
                                adjusted.enu_m,
                                &ephs,
                            rover.t_rx_s.0,
                            )
                            .map(|(rms, _pred, _)| rms);
                            if let (Some(before), Some(after)) = (before_rms, after_rms) {
                                if after > before * 1.1 {
                                    adjusted.fixed = false;
                                } else {
                                    adjusted.fixed = fix_result.accepted;
                                }
                            } else {
                                adjusted.fixed = fix_result.accepted;
                            }
                            let (rms_obs, rms_pred, used_sats) =
                                if let Some((rms_obs, rms_pred, count)) =
                                    bijux_gnss_receiver::rtk::dd_residual_metrics(
                                        &dd,
                                        base_xyz,
                                        adjusted.enu_m,
                                        &ephs,
                            rover.t_rx_s.0,
                                    )
                                {
                                    (rms_obs, rms_pred, count)
                                } else {
                                    (0.0, 0.0, 0)
                                };
                            let separation = bijux_gnss_receiver::rtk::solution_separation(
                                &dd,
                                base_xyz,
                                &ephs,
                            rover.t_rx_s.0,
                            );
                            let mut sep_sig = None;
                            let mut sep_max = None;
                            if let Some(seps) = separation {
                                if let Some(max) = seps
                                    .iter()
                                    .max_by(|a, b| a.delta_enu_m.partial_cmp(&b.delta_enu_m).unwrap())
                                {
                                    sep_sig = Some(format!("{:?}", max.sig));
                                    sep_max = Some(max.delta_enu_m);
                                }
                            }
                            if let Some(cov) = adjusted.covariance_m2 {
                                let sigma_e = cov[0][0].abs().sqrt();
                                let sigma_n = cov[1][1].abs().sqrt();
                                let sigma_u = cov[2][2].abs().sqrt();
                                let sigma_h = (sigma_e * sigma_e + sigma_n * sigma_n).sqrt();
                                let hpl = sigma_h * 6.0;
                                let vpl = sigma_u * 6.0;
                                let quality = bijux_gnss_receiver::rtk::RtkBaselineQuality {
                                    epoch_idx: rover.epoch_idx,
                                    fixed: adjusted.fixed,
                                    sigma_e,
                                    sigma_n,
                                    sigma_u,
                                    used_sats,
                                    residual_rms_m: rms_obs,
                                    predicted_rms_m: rms_pred,
                                    hpl_m: hpl,
                                    vpl_m: vpl,
                                    separation_sig: sep_sig,
                                    separation_max_m: sep_max,
                                };
                                let wrapped = bijux_gnss_receiver::rtk::RtkBaselineQualityV1 {
                                    header: header.clone(),
                                    payload: quality,
                                };
                                baseline_quality_lines.push(serde_json::to_string(&wrapped)?);
                            }
                            let wrapped = bijux_gnss_receiver::rtk::RtkBaselineEpochV1 {
                                header: header.clone(),
                                payload: adjusted,
                            };
                            baseline_lines.push(serde_json::to_string(&wrapped)?);
                        }
    
                        let slip_count = base
                            .sats
                            .iter()
                            .chain(rover.sats.iter())
                            .filter(|s| s.lock_flags.cycle_slip)
                            .count();
                        let precision = bijux_gnss_receiver::rtk::RtkPrecision {
                            epoch_idx: rover.epoch_idx,
                            fix_accepted: fix_result.accepted,
                            ratio: fix_result.ratio,
                            fixed_count: audit.fixed_count,
                            ref_changed,
                            slip_count,
                        };
                        let wrapped = bijux_gnss_receiver::rtk::RtkPrecisionV1 {
                            header: header.clone(),
                            payload: precision,
                        };
                        precision_lines.push(serde_json::to_string(&wrapped)?);
                    }
    
                    let sd_path = out_dir.join("rtk_sd.jsonl");
                    fs::write(&sd_path, sd_lines.join("\n"))?;
                    let dd_path = out_dir.join("rtk_dd.jsonl");
                    fs::write(&dd_path, dd_lines.join("\n"))?;
                    let baseline_path = out_dir.join("rtk_baseline.jsonl");
                    fs::write(&baseline_path, baseline_lines.join("\n"))?;
                    let baseline_quality_path = out_dir.join("rtk_baseline_quality.jsonl");
                    fs::write(&baseline_quality_path, baseline_quality_lines.join("\n"))?;
                    let fix_audit_path = out_dir.join("rtk_fix_audit.jsonl");
                    fs::write(&fix_audit_path, fix_audit_lines.join("\n"))?;
                    let precision_path = out_dir.join("rtk_precision.jsonl");
                    fs::write(&precision_path, precision_lines.join("\n"))?;
    
                    let align_report = aligner.report(base_epochs.len(), rover_epochs.len());
                    fs::write(
                        out_dir.join("rtk_align.json"),
                        serde_json::to_string_pretty(&align_report)?,
                    )?;
    
                    validate_json_schema(
                        &schema_path("rtk_baseline_v1.schema.json"),
                        &baseline_path,
                        false,
                    )?;
                    validate_jsonl_schema(
                        &schema_path("rtk_baseline_quality_v1.schema.json"),
                        &baseline_quality_path,
                        false,
                    )?;
                    validate_jsonl_schema(
                        &schema_path("rtk_fix_audit_v1.schema.json"),
                        &fix_audit_path,
                        false,
                    )?;
                    validate_jsonl_schema(
                        &schema_path("rtk_precision_v1.schema.json"),
                        &precision_path,
                        false,
                    )?;

                    let report = serde_json::json!({
                        "aligned_epochs": aligned.len(),
                        "sd_count": sd_lines.len(),
                        "dd_count": dd_lines.len()
                    });
                    write_manifest(&common, "rtk", &profile, dataset.as_ref(), &report)?;

    Ok(())
}

fn handle_diagnostics(command: GnssCommand) -> Result<()> {
    let GnssCommand::Diagnostics { command } = command else {
        bail!("invalid command for handler");
    };

    match command {
        DiagnosticsCommand::Summarize {
            common,
            run_dir,
            top,
        } => {
            set_trace_dir(&common);
            let events = summarize_run_diagnostics(&run_dir)?;
            let summary = bijux_gnss_core::aggregate_diagnostics(&events);
            let mut entries = summary.entries.clone();
            entries.sort_by(|a, b| b.count.cmp(&a.count));
            println!("Diagnostics summary (top {}):", top);
            for entry in entries.into_iter().take(top.max(1)) {
                println!(
                    "{}\t{:?}\tcount={}\tfirst={:?}\tlast={:?}",
                    entry.code, entry.severity, entry.count, entry.first_epoch, entry.last_epoch
                );
            }
            let report = serde_json::json!({
                "run_dir": run_dir.display().to_string(),
                "total": summary.total,
                "top": top,
            });
            write_manifest(&common, "diagnostics_summarize", &ReceiverProfile::default(), None, &report)?;
        }
    }

    Ok(())
}

fn summarize_run_diagnostics(run_dir: &Path) -> Result<Vec<DiagnosticEvent>> {
    let artifacts_dir = run_dir.join("artifacts");
    if !artifacts_dir.exists() {
        bail!("artifacts directory not found: {}", artifacts_dir.display());
    }
    let mut events = Vec::new();
    for entry in fs::read_dir(&artifacts_dir)? {
        let entry = entry?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        let Some(ext) = path.extension().and_then(|s| s.to_str()) else {
            continue;
        };
        if ext != "json" && ext != "jsonl" {
            continue;
        }
        if let Ok(result) = artifact_validate(&path, None, false) {
            events.extend(result.diagnostics);
        }
    }
    Ok(events)
}

fn handle_doctor(command: GnssCommand) -> Result<()> {
    let GnssCommand::Doctor { common, file } = command else {
        bail!("invalid command for handler");
    };

    set_trace_dir(&common);
    println!("bijux gnss doctor");
    println!("build: {}", env!("CARGO_PKG_VERSION"));
    println!("features: tracing={}", cfg!(feature = "tracing"));

    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    {
        println!(
            "cpu: sse2={} avx2={} fma={}",
            std::is_x86_feature_detected!("sse2"),
            std::is_x86_feature_detected!("avx2"),
            std::is_x86_feature_detected!("fma")
        );
    }
    #[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
    {
        println!("cpu: feature detection not available on this arch");
    }

    if let Some(config_path) = &common.config {
        match load_profile_from_path(config_path) {
            Ok(_) => println!("config: ok ({})", config_path.display()),
            Err(err) => println!("config: invalid ({}) - {}", config_path.display(), err),
        }
    } else {
        println!("config: default");
    }

    if let Some(path) = file {
        if path.exists() {
            println!("input: ok ({})", path.display());
        } else {
            println!("input: missing ({})", path.display());
        }
    }

    Ok(())
}
