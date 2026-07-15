fn read_tracking_dump(path: &Path) -> Result<Vec<TrackingRow>> {
    let data = fs::read_to_string(path)?;
    if let Ok(report) = serde_json::from_str::<TrackingReport>(&data) {
        return Ok(report.epochs);
    }
    let mut rows = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        if line.contains("\"header\"") && line.contains("\"payload\"") {
            let wrapped: TrackEpochV1 = serde_json::from_str(line)?;
            if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
                bail!("unsupported track schema_version {}", wrapped.header.schema_version);
            }
            rows.push(tracking_row_from_epoch(wrapped.payload));
        } else {
            let row: TrackingRow = serde_json::from_str(line)?;
            rows.push(row);
        }
    }
    Ok(rows)
}

fn tracking_row_from_epoch(epoch: TrackEpoch) -> TrackingRow {
    TrackingRow {
        epoch_idx: epoch.epoch.index,
        sample_index: epoch.sample_index,
        sat: epoch.sat,
        carrier_hz: epoch.carrier_hz.0,
        carrier_phase_cycles: epoch.carrier_phase_cycles.0,
        code_rate_hz: epoch.code_rate_hz.0,
        code_phase_samples: epoch.code_phase_samples.0,
        prompt_i: epoch.prompt_i,
        prompt_q: epoch.prompt_q,
        early_i: epoch.early_i,
        early_q: epoch.early_q,
        late_i: epoch.late_i,
        late_q: epoch.late_q,
        lock: epoch.lock,
        cn0_dbhz: epoch.cn0_dbhz,
        pll_lock: epoch.pll_lock,
        dll_lock: epoch.dll_lock,
        fll_lock: epoch.fll_lock,
        cycle_slip: epoch.cycle_slip,
        nav_bit_lock: epoch.nav_bit_lock,
        navigation_bit_sign: epoch.navigation_bit_sign,
        dll_err: epoch.dll_err,
        pll_err: epoch.pll_err,
        fll_err: epoch.fll_err,
        anti_false_lock: epoch.anti_false_lock,
        cycle_slip_reason: epoch.cycle_slip_reason,
        lock_state: epoch.lock_state,
        lock_state_reason: epoch.lock_state_reason,
    }
}

fn read_obs_epochs(path: &Path) -> Result<Vec<ObsEpoch>> {
    let data = fs::read_to_string(path)?;
    let mut epochs = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        if line.contains("\"header\"") {
            let wrapped: ObsEpochV1 = serde_json::from_str(line)?;
            if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
                bail!("unsupported obs schema_version {}", wrapped.header.schema_version);
            }
            epochs.push(wrapped.payload);
        } else {
            let epoch: ObsEpoch = serde_json::from_str(line)?;
            epochs.push(epoch);
        }
    }
    validate_obs_epochs(&epochs).map_err(|err| eyre!("obs epoch validation failed: {err}"))?;
    Ok(epochs)
}

#[derive(serde::Deserialize)]
struct NavDecodeEphemerisReportInput {
    sat: SatId,
    #[serde(default)]
    reference_week: Option<u32>,
    #[serde(default)]
    decoded_subframes: Vec<bijux_gnss_infra::api::nav::GpsL1CaLnavDecodedSubframe>,
    #[serde(default)]
    ephemerides: Vec<GpsEphemeris>,
}

fn ephemerides_from_nav_decode_report(
    report: NavDecodeEphemerisReportInput,
) -> Result<Vec<GpsEphemeris>> {
    if !report.decoded_subframes.is_empty() {
        if let Some(reference_week) = report.reference_week {
            let (ephs, _rejections) =
                bijux_gnss_infra::api::nav::ephemerides_from_decoded_gps_l1ca_lnav(
                    report.sat.prn,
                    &report.decoded_subframes,
                    Some(reference_week),
                );
            return Ok(ephs);
        }
        if !report.ephemerides.is_empty() {
            return Ok(report.ephemerides);
        }
        bail!("decoded LNAV report is missing reference_week and embedded ephemerides");
    }

    if !report.ephemerides.is_empty() {
        return Ok(report.ephemerides);
    }

    Ok(Vec::new())
}

fn read_nav_decode_ephemerides(data: &str) -> Result<Option<Vec<GpsEphemeris>>> {
    if !data.contains("\"decoded_subframes\"") || !data.contains("\"sat\"") {
        return Ok(None);
    }

    if data.trim_start().starts_with('[') {
        let reports: Vec<NavDecodeEphemerisReportInput> = serde_json::from_str(data)?;
        let mut ephemerides = Vec::new();
        for report in reports {
            ephemerides.extend(ephemerides_from_nav_decode_report(report)?);
        }
        return Ok(Some(ephemerides));
    }

    let report: NavDecodeEphemerisReportInput = serde_json::from_str(data)?;
    Ok(Some(ephemerides_from_nav_decode_report(report)?))
}

fn read_broadcast_navigation_data(
    path: &Path,
) -> Result<bijux_gnss_infra::api::nav::GpsBroadcastNavigationData> {
    let data = fs::read_to_string(path)?;
    if data.contains("RINEX VERSION / TYPE")
        && (data.contains("NAVIGATION DATA") || data.contains("NAV DATA"))
    {
        return bijux_gnss_infra::api::nav::parse_rinex_broadcast_navigation(&data)
            .map_err(|err| eyre!("RINEX NAV parse failed: {}", err.message));
    }
    if let Some(ephemerides) = read_nav_decode_ephemerides(&data)? {
        return Ok(bijux_gnss_infra::api::nav::GpsBroadcastNavigationData {
            ephemerides,
            klobuchar: None,
        });
    }
    if data.contains("\"header\"") {
        if let Ok(wrapped) = serde_json::from_str::<
            bijux_gnss_infra::api::core::ArtifactV1<
                bijux_gnss_infra::api::nav::GpsBroadcastNavigationData,
            >,
        >(&data)
        {
            if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
                bail!(
                    "unsupported broadcast navigation schema_version {}",
                    wrapped.header.schema_version
                );
            }
            return Ok(wrapped.payload);
        }
        let wrapped: GpsEphemerisV1 = serde_json::from_str(&data)?;
        if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
            bail!("unsupported ephemeris schema_version {}", wrapped.header.schema_version);
        }
        return Ok(bijux_gnss_infra::api::nav::GpsBroadcastNavigationData {
            ephemerides: wrapped.payload,
            klobuchar: None,
        });
    }
    if data.contains("\"ephemerides\"") {
        return serde_json::from_str(&data).map_err(Into::into);
    }
    let ephemerides: Vec<GpsEphemeris> = serde_json::from_str(&data)?;
    Ok(bijux_gnss_infra::api::nav::GpsBroadcastNavigationData { ephemerides, klobuchar: None })
}

fn read_ephemeris(path: &Path) -> Result<Vec<GpsEphemeris>> {
    Ok(read_broadcast_navigation_data(path)?.ephemerides)
}

fn read_reference_epochs(path: &Path) -> Result<Vec<ValidationReferenceEpoch>> {
    let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("");
    let data = fs::read_to_string(path)?;
    let mut epochs = Vec::new();
    if ext.eq_ignore_ascii_case("csv") {
        for (idx, line) in data.lines().enumerate() {
            if idx == 0 && line.contains("epoch_idx") {
                continue;
            }
            if line.trim().is_empty() {
                continue;
            }
            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() < 4 {
                bail!("invalid reference csv row: {line}");
            }
            let epoch_idx = parts[0].trim().parse::<u64>()?;
            let t_rx_s = parts.get(1).and_then(|v| v.trim().parse::<f64>().ok());
            let latitude_deg =
                parts.get(2).and_then(|v| v.trim().parse::<f64>().ok()).unwrap_or(0.0);
            let longitude_deg =
                parts.get(3).and_then(|v| v.trim().parse::<f64>().ok()).unwrap_or(0.0);
            let altitude_m = parts.get(4).and_then(|v| v.trim().parse::<f64>().ok()).unwrap_or(0.0);
            let ecef_x_m = parts.get(5).and_then(|v| v.trim().parse::<f64>().ok());
            let ecef_y_m = parts.get(6).and_then(|v| v.trim().parse::<f64>().ok());
            let ecef_z_m = parts.get(7).and_then(|v| v.trim().parse::<f64>().ok());
            epochs.push(ValidationReferenceEpoch {
                epoch_idx,
                t_rx_s,
                latitude_deg,
                longitude_deg,
                altitude_m,
                ecef_x_m,
                ecef_y_m,
                ecef_z_m,
                vel_x_mps: None,
                vel_y_mps: None,
                vel_z_mps: None,
            });
        }
    } else {
        validate_jsonl_schema(&schema_path("reference_epoch.schema.json"), path, false)?;
        for line in data.lines() {
            if line.trim().is_empty() {
                continue;
            }
            let epoch: ValidationReferenceEpoch = serde_json::from_str(line)?;
            epochs.push(epoch);
        }
    }
    Ok(epochs)
}

fn read_nav_solutions(path: &Path) -> Result<Vec<bijux_gnss_infra::api::core::NavSolutionEpoch>> {
    let data = fs::read_to_string(path)?;
    let mut epochs = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: NavSolutionEpochV1 = serde_json::from_str(line)?;
        if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
            bail!("unsupported nav schema_version {}", wrapped.header.schema_version);
        }
        epochs.push(wrapped.payload);
    }
    Ok(epochs)
}

fn write_ephemeris(
    common: &CommonArgs,
    ephs: &[GpsEphemeris],
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<()> {
    let out_dir = artifacts_dir(common, "nav", dataset)?;
    let header = artifact_header(common, profile, dataset)?;
    let path = out_dir.join("ephemeris.json");
    let wrapped = GpsEphemerisV1 { header, payload: ephs.to_vec() };
    let data = serde_json::to_string_pretty(&wrapped)?;
    fs::write(&path, data)?;
    validate_json_schema(&schema_path("gps_ephemeris_v1.schema.json"), &path, false)?;
    Ok(())
}
