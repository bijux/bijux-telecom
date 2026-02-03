fn apply_overrides(
    profile: &mut ReceiverProfile,
    sampling_hz: Option<f64>,
    if_hz: Option<f64>,
    code_hz: Option<f64>,
    code_length: Option<usize>,
) {
    if let Some(value) = sampling_hz {
        profile.sample_rate_hz = value;
    }
    if let Some(value) = if_hz {
        profile.intermediate_freq_hz = value;
    }
    if let Some(value) = code_hz {
        profile.code_freq_basis_hz = value;
    }
    if let Some(value) = code_length {
        profile.code_length = value;
    }
}

fn apply_common_overrides(profile: &mut ReceiverProfile, common: &CommonArgs) {
    if let Some(seed) = common.seed {
        profile.seed = seed;
    }
}

fn hash_config(path: Option<&PathBuf>, profile: &ReceiverProfile) -> Result<String> {
    let mut hasher = Sha256::new();
    if let Some(path) = path {
        let bytes = fs::read(path)?;
        hasher.update(bytes);
    } else {
        let serialized = toml::to_string(profile)?;
        hasher.update(serialized.as_bytes());
    }
    Ok(hex::encode(hasher.finalize()))
}

fn git_hash() -> Option<String> {
    let output = ProcessCommand::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let hash = String::from_utf8_lossy(&output.stdout).trim().to_string();
    Some(hash)
}

fn cpu_features() -> Vec<String> {
    let mut features = Vec::new();
    #[cfg(target_arch = "x86_64")]
    {
        if std::is_x86_feature_detected!("avx2") {
            features.push("avx2".to_string());
        }
        if std::is_x86_feature_detected!("sse4.2") {
            features.push("sse4.2".to_string());
        }
        if std::is_x86_feature_detected!("fma") {
            features.push("fma".to_string());
        }
    }
    features
}

fn format_sat(sat: SatId) -> String {
    format!("{:?}-{}", sat.constellation, sat.prn)
}

fn prns_to_sats(prns: &[u8]) -> Vec<SatId> {
    prns.iter()
        .map(|&prn| SatId {
            constellation: Constellation::Gps,
            prn,
        })
        .collect()
}

fn parse_sweep(values: &[String]) -> Result<Vec<(String, Vec<String>)>> {
    let mut out = Vec::new();
    for item in values {
        let Some((key, vals)) = item.split_once('=') else {
            bail!("invalid sweep format: {item}");
        };
        let vals: Vec<String> = vals.split(',').map(|v| v.trim().to_string()).collect();
        if vals.is_empty() {
            bail!("sweep values empty for {key}");
        }
        out.push((key.trim().to_string(), vals));
    }
    Ok(out)
}

fn expand_sweep(spec: &[(String, Vec<String>)]) -> Vec<Vec<(String, String)>> {
    fn expand(
        idx: usize,
        spec: &[(String, Vec<String>)],
        current: &mut Vec<(String, String)>,
        out: &mut Vec<Vec<(String, String)>>,
    ) {
        if idx == spec.len() {
            out.push(current.clone());
            return;
        }
        let (key, vals) = &spec[idx];
        for val in vals {
            current.push((key.clone(), val.clone()));
            expand(idx + 1, spec, current, out);
            current.pop();
        }
    }
    let mut out = Vec::new();
    expand(0, spec, &mut Vec::new(), &mut out);
    if out.is_empty() {
        out.push(Vec::new());
    }
    out
}


fn apply_sweep_value(profile: &mut ReceiverProfile, key: &str, value: &str) -> Result<()> {
    match key {
        "tracking.dll_bw_hz" => profile.tracking.dll_bw_hz = value.parse()?,
        "tracking.pll_bw_hz" => profile.tracking.pll_bw_hz = value.parse()?,
        "tracking.fll_bw_hz" => profile.tracking.fll_bw_hz = value.parse()?,
        "tracking.early_late_spacing_chips" => {
            profile.tracking.early_late_spacing_chips = value.parse()?
        }
        "acquisition.integration_ms" => profile.acquisition.integration_ms = value.parse()?,
        "acquisition.doppler_step_hz" => profile.acquisition.doppler_step_hz = value.parse()?,
        "acquisition.doppler_search_hz" => profile.acquisition.doppler_search_hz = value.parse()?,
        "acquisition.peak_mean_threshold" => {
            profile.acquisition.peak_mean_threshold = value.parse()?
        }
        "acquisition.peak_second_threshold" => {
            profile.acquisition.peak_second_threshold = value.parse()?
        }
        "navigation.hatch_window" => profile.navigation.hatch_window = value.parse()?,
        "navigation.weighting.elev_mask_deg" => {
            profile.navigation.weighting.elev_mask_deg = value.parse()?
        }
        "navigation.weighting.elev_exponent" => {
            profile.navigation.weighting.elev_exponent = value.parse()?
        }
        "navigation.weighting.cn0_ref_dbhz" => {
            profile.navigation.weighting.cn0_ref_dbhz = value.parse()?
        }
        "navigation.weighting.min_weight" => {
            profile.navigation.weighting.min_weight = value.parse()?
        }
        "navigation.weighting.tracking_mode_scalar_weight" => {
            profile.navigation.weighting.tracking_mode_scalar_weight = value.parse()?
        }
        "navigation.weighting.tracking_mode_vector_weight" => {
            profile.navigation.weighting.tracking_mode_vector_weight = value.parse()?
        }
        "navigation.robust_solver" => profile.navigation.robust_solver = value.parse()?,
        "navigation.raim" => profile.navigation.raim = value.parse()?,
        _ => bail!("unsupported sweep parameter: {key}"),
    }
    Ok(())
}

fn parse_ecef(text: &str) -> Result<[f64; 3]> {
    let parts: Vec<&str> = text.split(',').collect();
    if parts.len() != 3 {
        bail!("invalid ECEF format, expected x,y,z");
    }
    Ok([
        parts[0].trim().parse()?,
        parts[1].trim().parse()?,
        parts[2].trim().parse()?,
    ])
}

fn stats(values: &[f64]) -> ValidationErrorStats {
    if values.is_empty() {
        return ValidationErrorStats {
            count: 0,
            mean: 0.0,
            median: 0.0,
            rms: 0.0,
            p95: 0.0,
        };
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let count = values.len();
    let mean = values.iter().sum::<f64>() / count as f64;
    let rms = (values.iter().map(|v| v * v).sum::<f64>() / count as f64).sqrt();
    let median = sorted[count / 2];
    let p95 = sorted[(count as f64 * 0.95).floor().min((count - 1) as f64) as usize];
    ValidationErrorStats {
        count,
        mean,
        median,
        rms,
        p95,
    }
}

fn lla_to_ecef(lat_deg: f64, lon_deg: f64, alt_m: f64) -> (f64, f64, f64) {
    let a = 6378137.0;
    let f = 1.0 / 298.257_223_563;
    let e2 = f * (2.0 - f);
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();
    let sin_lat = lat.sin();
    let cos_lat = lat.cos();
    let n = a / (1.0 - e2 * sin_lat * sin_lat).sqrt();
    let x = (n + alt_m) * cos_lat * lon.cos();
    let y = (n + alt_m) * cos_lat * lon.sin();
    let z = (n * (1.0 - e2) + alt_m) * sin_lat;
    (x, y, z)
}
