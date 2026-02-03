include!("handlers/gnss_1.rs");
include!("handlers/gnss_2.rs");
include!("handlers/gnss_3.rs");
include!("handlers/gnss_4.rs");

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
        cmd @ GnssCommand::ValidateArtifacts { .. } => handle_validateartifacts(cmd),
        cmd @ GnssCommand::ValidateSidecar { .. } => handle_validatesidecar(cmd),
        cmd @ GnssCommand::ConfigSchema { .. } => handle_configschema(cmd),
        cmd @ GnssCommand::Validate { .. } => handle_validate(cmd),
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
fn set_trace_dir(common: &CommonArgs) {
    if let Some(dir) = &common.dump {
        std::env::set_var("BIJUX_TRACE_DIR", dir);
    }
}
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
) -> Result<Option<bijux_gnss_core::NavSolutionEpoch>> {
    let Some(ctx) = ctx.as_mut() else {
        return Ok(None);
    };
    let dt_s = if let Some(prev) = ctx.last_t_rx_s {
        (obs.t_rx_s - prev).max(1e-3)
    } else {
        0.001
    };
    ctx.last_t_rx_s = Some(obs.t_rx_s);
    ctx.ekf.predict(&ctx.model, dt_s);

    let mut used = 0;
    let mut sats: Vec<&bijux_gnss_core::ObsSatellite> = obs.sats.iter().collect();
    sats.sort_by_key(|s| s.signal_id);
    for sat in sats {
        let eph = match ephs.iter().find(|e| e.sat == sat.signal_id.sat) {
            Some(e) => e,
            None => continue,
        };
        let _corr = bijux_gnss_nav::compute_corrections(&ctx.corrections);
        let state = sat_state_gps_l1ca(eph, obs.t_rx_s, 0.0);
        let state_next = sat_state_gps_l1ca(eph, obs.t_rx_s + 0.1, 0.0);
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
            bijux_gnss_nav::weight_from_cn0_elev(sat.cn0_dbhz, el, WeightingConfig::default());
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
            z_m: sat.pseudorange_m - code_bias_m,
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

        let doppler_meas = bijux_gnss_nav::DopplerMeasurement {
            sig: sat.signal_id,
            z_hz: sat.doppler_hz,
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
        let carrier_meas = bijux_gnss_nav::CarrierPhaseMeasurement {
            sig: sat.signal_id,
            z_cycles: sat.carrier_phase_cycles - phase_bias_cycles,
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
            let after = bijux_gnss_nav::clamp_ztd(before, &ctx.atmosphere);
            if (after - before).abs() > 1e-6 {
                ctx.ekf.x[idx] = after;
                ctx.ekf
                    .health
                    .events
                    .push(bijux_gnss_core::NavHealthEvent::ZtdClamped {
                        before_m: before,
                        after_m: after,
                    });
            }
        }
    }

    if used < 4 {
        return Ok(None);
    }
    let (lat, lon, alt) =
        bijux_gnss_nav::ecef_to_geodetic(ctx.ekf.x[0], ctx.ekf.x[1], ctx.ekf.x[2]);
    Ok(Some(bijux_gnss_core::NavSolutionEpoch {
        epoch: bijux_gnss_core::Epoch {
            index: obs.epoch_idx,
        },
        ecef_x_m: ctx.ekf.x[0],
        ecef_y_m: ctx.ekf.x[1],
        ecef_z_m: ctx.ekf.x[2],
        latitude_deg: lat,
        longitude_deg: lon,
        altitude_m: alt,
        clock_bias_s: ctx.ekf.x[6],
        pdop: 0.0,
        rms_m: ctx.ekf.health.innovation_rms,
        residuals: Vec::new(),
        isb: Vec::new(),
        sigma_h_m: None,
        sigma_v_m: None,
        ekf_innovation_rms: Some(ctx.ekf.health.innovation_rms),
        ekf_condition_number: ctx.ekf.health.condition_number,
        ekf_whiteness_ratio: ctx.ekf.health.whiteness_ratio,
        ekf_predicted_variance: ctx.ekf.health.predicted_variance,
        ekf_observed_variance: ctx.ekf.health.observed_variance,
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
