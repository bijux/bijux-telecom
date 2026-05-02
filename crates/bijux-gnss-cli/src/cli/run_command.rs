fn run_command(command: GnssCommand) -> Result<()> {
    match command {
        cmd @ GnssCommand::CaCode { .. } => handle_cacode(cmd),
        cmd @ GnssCommand::Acquire { .. } => handle_acquire(cmd),
        cmd @ GnssCommand::Track { .. } => handle_track(cmd),
        cmd @ GnssCommand::Nav { .. } => handle_nav(cmd),
        cmd @ GnssCommand::Pvt { .. } => handle_pvt(cmd),
        cmd @ GnssCommand::Inspect { .. } => handle_inspect(cmd),
        cmd @ GnssCommand::Rtk { .. } => handle_rtk(cmd),
        cmd @ GnssCommand::Experiment { .. } => handle_experiment(cmd),
        cmd @ GnssCommand::ValidateConfig { .. } => handle_validateconfig(cmd),
        cmd @ GnssCommand::Config { .. } => handle_config(cmd),
        cmd @ GnssCommand::ValidateArtifacts { .. } => handle_validateartifacts(cmd),
        cmd @ GnssCommand::ValidateSidecar { .. } => handle_validatesidecar(cmd),
        cmd @ GnssCommand::Analyze { .. } => handle_analyze(cmd),
        cmd @ GnssCommand::Diff { .. } => handle_diff(cmd),
        cmd @ GnssCommand::Artifact { .. } => handle_artifact(cmd),
        cmd @ GnssCommand::Diagnostics { .. } => handle_diagnostics(cmd),
        cmd @ GnssCommand::ConfigUpgrade { .. } => handle_configupgrade(cmd),
        cmd @ GnssCommand::ConfigSchema { .. } => handle_configschema(cmd),
        cmd @ GnssCommand::Validate { .. } => handle_validate(cmd),
        cmd @ GnssCommand::ValidateReference { .. } => handle_validate_reference(cmd),
        cmd @ GnssCommand::Run { .. } => handle_run(cmd),
        cmd @ GnssCommand::Rinex { .. } => handle_rinex(cmd),
        cmd @ GnssCommand::Doctor { .. } => handle_doctor(cmd),
    }
}



fn inspect_dataset(path: &Path, sample_rate_hz: f64, max_samples: usize) -> Result<InspectReport> {
    let bytes = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    let mut samples = Vec::with_capacity(bytes.len() / 2);
    for chunk in bytes.chunks_exact(2) {
        samples.push(i16::from_le_bytes([chunk[0], chunk[1]]));
    }

    let total_iq = samples.len() / 2;
    let limit = if max_samples == 0 {
        total_iq
    } else {
        total_iq.min(max_samples)
    };

    let mut sum_i = 0.0f64;
    let mut sum_q = 0.0f64;
    let mut clip = 0u64;
    let mut power_hist = vec![0u64; 8];
    let mut power_sum = 0.0f64;

    for idx in 0..limit {
        let i = samples[2 * idx] as f64;
        let q = samples[2 * idx + 1] as f64;
        sum_i += i;
        sum_q += q;
        if samples[2 * idx].abs() == i16::MAX || samples[2 * idx + 1].abs() == i16::MAX {
            clip += 1;
        }
        let power = i * i + q * q;
        power_sum += power;
        let bin = ((power.sqrt() / 10000.0).min(7.0)) as usize;
        power_hist[bin] += 1;
    }

    let dc_i = sum_i / limit.max(1) as f64;
    let dc_q = sum_q / limit.max(1) as f64;
    let mean_power = power_sum / limit.max(1) as f64;
    let noise_floor_db = 10.0 * mean_power.max(1e-9).log10();

    Ok(InspectReport {
        sample_rate_hz,
        total_samples: limit,
        dc_offset_i: dc_i,
        dc_offset_q: dc_q,
        clip_rate: clip as f64 / limit.max(1) as f64,
        noise_floor_db,
        power_histogram: power_hist,
    })
}
fn runtime_config_from_env(
    common: &CommonArgs,
    run_dir: Option<PathBuf>,
) -> bijux_gnss_infra::api::receiver::ReceiverRuntime {
    if common.deterministic {
        std::env::set_var("RAYON_NUM_THREADS", "1");
        std::env::set_var("BIJUX_DETERMINISTIC", "1");
    }
    let config = bijux_gnss_infra::api::receiver::ReceiverRuntimeConfig {
        run_id: std::env::var("BIJUX_RUN_ID").ok(),
        trace_dir: common.dump.clone(),
        run_dir: run_dir.or_else(|| std::env::var("BIJUX_RUN_DIR").ok().map(PathBuf::from)),
        diagnostics_dump: std::env::var("BIJUX_DIAGNOSTICS_DUMP")
            .ok()
            .as_deref()
            == Some("1"),
    };
    bijux_gnss_infra::api::receiver::ReceiverRuntime::new(config)
}
use bijux_gnss_infra::api::core::format_sat;

fn print_acquisition_table(report: &AcquisitionReport) {
    println!("Sat\tCarrier(Hz)\tCodePhase\tPeak\tPeak/Mean\tPeak/2nd");
    for row in &report.results {
        println!(
            "{}\t{:.1}\t{}\t{:.3}\t{:.2}\t{:.2}",
            format_sat(row.sat),
            row.carrier_hz,
            row.code_phase_samples,
            row.peak,
            row.peak_mean_ratio,
            row.peak_second_ratio
        );
    }
}
fn print_inspect_table(report: &InspectReport) {
    println!("SampleRate(Hz)\tSamples\tDC_I\tDC_Q\tClipRate\tNoiseFloor(dB)");
    println!(
        "{:.1}\t{}\t{:.3}\t{:.3}\t{:.6}\t{:.2}",
        report.sample_rate_hz,
        report.total_samples,
        report.dc_offset_i,
        report.dc_offset_q,
        report.clip_rate,
        report.noise_floor_db
    );
    println!("Power histogram bins: {:?}", report.power_histogram);
}
fn solve_epoch_ekf(
    ctx: &mut Option<EkfContext>,
    obs: &ObsEpoch,
    ephs: &[GpsEphemeris],
) -> Result<Option<bijux_gnss_infra::api::core::NavSolutionEpoch>> {
    let Some(ctx) = ctx.as_mut() else {
        return Ok(None);
    };
    let dt_s = if let Some(prev) = ctx.last_t_rx_s {
        (obs.t_rx_s.0 - prev).max(1e-3)
    } else {
        0.001
    };
    ctx.last_t_rx_s = Some(obs.t_rx_s.0);
    ctx.ekf.predict(&ctx.model, dt_s);

    let mut used = 0;
    let mut sats: Vec<&bijux_gnss_infra::api::core::ObsSatellite> = obs.sats.iter().collect();
    sats.sort_by_key(|s| s.signal_id);
    let sat_count = sats.len();
    for sat in sats {
        let eph = match ephs.iter().find(|e| e.sat == sat.signal_id.sat) {
            Some(e) => e,
            None => continue,
        };
        let _corr = bijux_gnss_infra::api::nav::compute_corrections(&ctx.corrections);
        let state = sat_state_gps_l1ca(eph, obs.t_rx_s.0, 0.0);
        let state_next = sat_state_gps_l1ca(eph, obs.t_rx_s.0 + 0.1, 0.0);
        let sat_vel = [
            (state_next.x_m - state.x_m) / 0.1,
            (state_next.y_m - state.y_m) / 0.1,
            (state_next.z_m - state.z_m) / 0.1,
        ];
        let rx_x = ctx.ekf.x[0];
        let rx_y = ctx.ekf.x[1];
        let rx_z = ctx.ekf.x[2];
        let (_az, el) = elevation_azimuth_deg(rx_x, rx_y, rx_z, state.x_m, state.y_m, state.z_m);
        let weight =
            bijux_gnss_infra::api::nav::weight_from_cn0_elev(sat.cn0_dbhz, el, WeightingConfig::default());
        let sigma_m = (5.0 / weight.max(0.1)).max(1.0);
        let isb_index = if sat.signal_id.sat.constellation != Constellation::Gps {
            let key = format!("isb_{:?}", sat.signal_id.sat.constellation);
            Some(ctx.isb.get_or_add(&mut ctx.ekf, &key, 0.0, 1e-6))
        } else {
            None
        };
        let code_bias_m = ctx.code_bias.code_bias_m(sat.signal_id).unwrap_or(0.0);
        let meas = PseudorangeMeasurement {
            sig: sat.signal_id,
            z_m: sat.pseudorange_m.0 - code_bias_m,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_clock_s: state.clock_bias_s,
            tropo_m: 0.0,
            iono_m: 0.0,
            sigma_m,
            elevation_deg: Some(el),
            ztd_index: ctx.ztd_index,
            isb_index,
        };
        if ctx.ekf.update(&meas) {
            used += 1;
        }

        let doppler_meas = bijux_gnss_infra::api::nav::DopplerMeasurement {
            sig: sat.signal_id,
            z_hz: sat.doppler_hz.0,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_vel_mps: sat_vel,
            wavelength_m: 299_792_458.0 / sat.metadata.signal.carrier_hz.value(),
            sigma_hz: 2.0,
        };
        ctx.ekf.update(&doppler_meas);

        let amb_key = format!(
            "{:?}:{}:{:?}",
            sat.metadata.signal.constellation, sat.signal_id.sat.prn, sat.metadata.signal.band
        );
        let amb_idx = ctx.ambiguity.get_or_add(&mut ctx.ekf, &amb_key, 0.0, 100.0);
        let phase_bias_cycles = ctx
            .phase_bias
            .phase_bias_cycles(sat.signal_id)
            .unwrap_or(0.0);
        let carrier_meas = bijux_gnss_infra::api::nav::CarrierPhaseMeasurement {
            sig: sat.signal_id,
            z_cycles: sat.carrier_phase_cycles.0 - phase_bias_cycles,
            sat_pos_m: [state.x_m, state.y_m, state.z_m],
            sat_clock_s: state.clock_bias_s,
            tropo_m: 0.0,
            iono_m: 0.0,
            wavelength_m: 299_792_458.0 / sat.metadata.signal.carrier_hz.value(),
            ambiguity_index: Some(amb_idx),
            sigma_cycles: 0.05,
            elevation_deg: Some(el),
            ztd_index: ctx.ztd_index,
            isb_index,
        };
        ctx.ekf.update(&carrier_meas);
    }

    if let Some(idx) = ctx.ztd_index {
        if idx < ctx.ekf.x.len() {
            let before = ctx.ekf.x[idx];
            let after = bijux_gnss_infra::api::nav::clamp_ztd(before, &ctx.atmosphere);
            if (after - before).abs() > 1e-6 {
                ctx.ekf.x[idx] = after;
                ctx.ekf
                    .health
                    .events
                    .push(bijux_gnss_infra::api::core::NavHealthEvent::ZtdClamped {
                        before_m: before,
                        after_m: after,
                    });
            }
        }
    }

    let (lat, lon, alt) =
        bijux_gnss_infra::api::nav::ecef_to_geodetic(ctx.ekf.x[0], ctx.ekf.x[1], ctx.ekf.x[2]);
    let status = if used < 4 {
        bijux_gnss_infra::api::core::SolutionStatus::Degraded
    } else {
        bijux_gnss_infra::api::core::SolutionStatus::Float
    };
    Ok(Some(bijux_gnss_infra::api::core::NavSolutionEpoch {
        epoch: bijux_gnss_infra::api::core::Epoch {
            index: obs.epoch_idx,
        },
        t_rx_s: obs.t_rx_s,
        ecef_x_m: bijux_gnss_infra::api::core::Meters(ctx.ekf.x[0]),
        ecef_y_m: bijux_gnss_infra::api::core::Meters(ctx.ekf.x[1]),
        ecef_z_m: bijux_gnss_infra::api::core::Meters(ctx.ekf.x[2]),
        latitude_deg: lat,
        longitude_deg: lon,
        altitude_m: bijux_gnss_infra::api::core::Meters(alt),
        clock_bias_s: bijux_gnss_infra::api::core::Seconds(ctx.ekf.x[6]),
        clock_drift_s_per_s: ctx.ekf.x.get(7).copied().unwrap_or(0.0),
        pdop: 0.0,
        rms_m: bijux_gnss_infra::api::core::Meters(ctx.ekf.health.innovation_rms),
        status,
        quality: status.quality_flag(),
        validity: if status == bijux_gnss_infra::api::core::SolutionStatus::Invalid {
            bijux_gnss_infra::api::core::SolutionValidity::Invalid
        } else {
            bijux_gnss_infra::api::core::SolutionValidity::Converging
        },
        valid: bijux_gnss_infra::api::core::is_solution_valid(status),
        processing_ms: None,
        residuals: Vec::new(),
        health: Vec::new(),
        isb: Vec::new(),
        sigma_h_m: None,
        sigma_v_m: None,
        innovation_rms_m: Some(ctx.ekf.health.innovation_rms),
        normalized_innovation_rms: None,
        normalized_innovation_max: None,
        ekf_innovation_rms: Some(ctx.ekf.health.innovation_rms),
        ekf_condition_number: ctx.ekf.health.condition_number,
        ekf_whiteness_ratio: ctx.ekf.health.whiteness_ratio,
        ekf_predicted_variance: ctx.ekf.health.predicted_variance,
        ekf_observed_variance: ctx.ekf.health.observed_variance,
        integrity_hpl_m: None,
        integrity_vpl_m: None,
        model_version: bijux_gnss_infra::api::core::NAV_SOLUTION_MODEL_VERSION,
        lifecycle_state: match status {
            bijux_gnss_infra::api::core::SolutionStatus::Invalid => {
                bijux_gnss_infra::api::core::NavLifecycleState::Invalid
            }
            bijux_gnss_infra::api::core::SolutionStatus::Held => {
                bijux_gnss_infra::api::core::NavLifecycleState::Held
            }
            bijux_gnss_infra::api::core::SolutionStatus::Degraded => {
                bijux_gnss_infra::api::core::NavLifecycleState::Degraded
            }
            bijux_gnss_infra::api::core::SolutionStatus::Coarse => {
                bijux_gnss_infra::api::core::NavLifecycleState::Coarse
            }
            bijux_gnss_infra::api::core::SolutionStatus::Converged => {
                bijux_gnss_infra::api::core::NavLifecycleState::Converged
            }
            bijux_gnss_infra::api::core::SolutionStatus::Float => {
                bijux_gnss_infra::api::core::NavLifecycleState::Float
            }
            bijux_gnss_infra::api::core::SolutionStatus::Fixed => {
                bijux_gnss_infra::api::core::NavLifecycleState::Fixed
            }
        },
        uncertainty_class: if status == bijux_gnss_infra::api::core::SolutionStatus::Invalid {
            bijux_gnss_infra::api::core::NavUncertaintyClass::Unknown
        } else if status == bijux_gnss_infra::api::core::SolutionStatus::Degraded {
            bijux_gnss_infra::api::core::NavUncertaintyClass::High
        } else {
            bijux_gnss_infra::api::core::NavUncertaintyClass::Medium
        },
        assumptions: None,
        refusal_class: None,
        artifact_id: String::new(),
        source_observation_epoch_id: String::new(),
        explain_decision: "cli_navigation_solution".to_string(),
        explain_reasons: vec![format!("usable_satellites={used}")],
        provenance: None,
        sat_count,
        used_sat_count: used,
        rejected_sat_count: sat_count.saturating_sub(used),
        hdop: None,
        vdop: None,
        gdop: None,
        stability_signature: String::new(),
        stability_signature_version:
            bijux_gnss_infra::api::core::NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
    }))
}


#[cfg(feature = "tracing")]
fn init_tracing() {
    let _ = tracing_subscriber::fmt()
        .with_env_filter("info")
        .with_target(false)
        .try_init();
}

#[cfg(not(feature = "tracing"))]
fn init_tracing() {}


fn main() -> Result<()> {
    init_tracing();
    let cli = Cli::parse();
    match cli.command {
        AppCommand::Gnss { command } => run_command(command),
    }
}
