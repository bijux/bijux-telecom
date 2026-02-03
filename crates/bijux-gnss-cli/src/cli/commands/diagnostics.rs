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
                        write_ephemeris(&common, &ephs)?;
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
                    let base_epochs = read_obs_epochs(&base_obs)?;
                    let rover_epochs = read_obs_epochs(&rover_obs)?;
                    let ephs = read_ephemeris(&eph)?;
                    let base_xyz = parse_ecef(&base_ecef)?;
    
                    let mut aligner = bijux_gnss_receiver::rtk::EpochAligner::new(tolerance_s);
                    let aligned = aligner.align(&base_epochs, &rover_epochs);
    
                    let out_dir = common.out.clone().context("--out is required for rtk")?;
                    fs::create_dir_all(&out_dir)?;
    
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
    
                    for (base, rover) in aligned {
                        let sd = bijux_gnss_receiver::rtk::build_sd(&base, &rover);
                        for item in &sd {
                            sd_lines.push(serde_json::to_string(item)?);
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
                            dd_lines.push(serde_json::to_string(item)?);
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
                            rover.t_rx_s,
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
                        fix_audit_lines.push(serde_json::to_string(&audit)?);
    
                        if let Some(baseline_val) = baseline.take() {
                            let before_rms = bijux_gnss_receiver::rtk::dd_residual_metrics(
                                &dd,
                                base_xyz,
                                baseline_val.enu_m,
                                &ephs,
                                rover.t_rx_s,
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
                                rover.t_rx_s,
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
                                        rover.t_rx_s,
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
                                rover.t_rx_s,
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
                                let mut obj = serde_json::json!({
                                    "epoch_idx": rover.epoch_idx,
                                    "fixed": adjusted.fixed,
                                    "sigma_e": sigma_e,
                                    "sigma_n": sigma_n,
                                    "sigma_u": sigma_u,
                                    "used_sats": used_sats,
                                    "residual_rms_m": rms_obs,
                                    "predicted_rms_m": rms_pred,
                                    "hpl_m": hpl,
                                    "vpl_m": vpl
                                });
                                if let serde_json::Value::Object(map) = &mut obj {
                                    if let Some(sig) = sep_sig {
                                        map.insert(
                                            "separation_sig".to_string(),
                                            serde_json::Value::String(sig),
                                        );
                                    }
                                    if let Some(val) = sep_max {
                                        map.insert(
                                            "separation_max_m".to_string(),
                                            serde_json::Value::Number(
                                                serde_json::Number::from_f64(val)
                                                    .unwrap_or_else(|| serde_json::Number::from(0)),
                                            ),
                                        );
                                    }
                                }
                                baseline_quality_lines.push(serde_json::to_string(&obj)?);
                            }
                            baseline_lines.push(serde_json::to_string(&adjusted)?);
                        }
    
                        let slip_count = base
                            .sats
                            .iter()
                            .chain(rover.sats.iter())
                            .filter(|s| s.lock_flags.cycle_slip)
                            .count();
                        precision_lines.push(serde_json::to_string(&serde_json::json!({
                            "epoch_idx": rover.epoch_idx,
                            "fix_accepted": fix_result.accepted,
                            "ratio": fix_result.ratio,
                            "fixed_count": audit.fixed_count,
                            "ref_changed": ref_changed,
                            "slip_count": slip_count
                        }))?);
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
                        &schema_path("rtk_baseline.schema.json"),
                        &baseline_path,
                        false,
                    )?;
                    validate_jsonl_schema(
                        &schema_path("rtk_baseline_quality.schema.json"),
                        &baseline_quality_path,
                        false,
                    )?;
                    validate_jsonl_schema(
                        &schema_path("rtk_fix_audit.schema.json"),
                        &fix_audit_path,
                        false,
                    )?;
                    validate_jsonl_schema(
                        &schema_path("rtk_precision.schema.json"),
                        &precision_path,
                        false,
                    )?;

    Ok(())
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
