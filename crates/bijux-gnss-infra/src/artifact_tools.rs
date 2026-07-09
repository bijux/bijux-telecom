//! Artifact validation and explanation helpers.

use std::path::Path;

use bijux_gnss_receiver::api::core::{
    aggregate_diagnostics, AcqResultV1, ArtifactHeaderV1, ArtifactPayloadValidate,
    ArtifactReadPolicy, ArtifactValidate, DiagnosticEvent, DiagnosticSeverity, InputError,
    NavSolutionEpochV1, ObsEpochV1, TrackEpochV1,
};

/// Result of validating an artifact.
#[derive(Debug, Clone)]
pub struct ArtifactValidationResult {
    /// Artifact kind.
    pub kind: String,
    /// Diagnostics produced by validation.
    pub diagnostics: Vec<DiagnosticEvent>,
}

/// Result of explaining an artifact.
#[derive(Debug, Clone)]
pub struct ArtifactExplainResult {
    /// Artifact kind.
    pub kind: String,
    /// Artifact header.
    pub header: ArtifactHeaderV1,
    /// Entry count (if applicable).
    pub entries: usize,
    /// Diagnostics summary.
    pub diagnostics_total: usize,
    /// Diagnostics error count.
    pub diagnostics_error: usize,
    /// Diagnostics warning count.
    pub diagnostics_warn: usize,
}

/// Validate an artifact file and return diagnostics.
pub fn artifact_validate(
    path: &Path,
    kind: Option<&str>,
    strict: bool,
) -> Result<ArtifactValidationResult, InputError> {
    let data = std::fs::read_to_string(path).map_err(map_err)?;
    if strict && data.trim().is_empty() {
        return Err(InputError { message: format!("artifact is empty: {}", path.display()) });
    }
    let kind = kind
        .map(|k| k.to_lowercase())
        .or_else(|| detect_kind_from_path(path))
        .unwrap_or_else(|| "unknown".to_string());

    let diagnostics = match kind.as_str() {
        "acq" => validate_acq_artifact(&data)?,
        "track" => validate_track_artifact(&data)?,
        "obs" => validate_obs_artifact(&data)?,
        "pvt" => validate_nav_artifact(&data)?,
        "unknown" => return Err(InputError { message: "unsupported artifact type".to_string() }),
        _ => return Err(InputError { message: "unsupported artifact type".to_string() }),
    };

    Ok(ArtifactValidationResult { kind, diagnostics })
}

/// Explain an artifact file and return header + stats.
pub fn artifact_explain(path: &Path) -> Result<ArtifactExplainResult, InputError> {
    let data = std::fs::read_to_string(path).map_err(map_err)?;
    let mut header: Option<ArtifactHeaderV1> = None;
    let kind = detect_kind_from_path(path).unwrap_or_else(|| "unknown".to_string());
    let mut entries = 0usize;

    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        if header.is_none() {
            header = Some(match kind.as_str() {
                "acq" => serde_json::from_str::<AcqResultV1>(line).map_err(map_err)?.header,
                "track" => serde_json::from_str::<TrackEpochV1>(line).map_err(map_err)?.header,
                "obs" => serde_json::from_str::<ObsEpochV1>(line).map_err(map_err)?.header,
                "pvt" => serde_json::from_str::<NavSolutionEpochV1>(line).map_err(map_err)?.header,
                _ => {
                    return Err(InputError { message: "unsupported artifact type".to_string() });
                }
            });
        }
        entries += 1;
    }

    let header =
        header.ok_or_else(|| InputError { message: "artifact header not found".to_string() })?;
    let diagnostics = artifact_validate(path, Some(&kind), false)?.diagnostics;
    let summary = aggregate_diagnostics(&diagnostics);
    let mut error_count = 0usize;
    let mut warn_count = 0usize;
    for entry in &summary.entries {
        match entry.severity {
            DiagnosticSeverity::Error => error_count += entry.count,
            DiagnosticSeverity::Warning => warn_count += entry.count,
            DiagnosticSeverity::Info => {}
        }
    }

    Ok(ArtifactExplainResult {
        kind,
        header,
        entries,
        diagnostics_total: summary.total,
        diagnostics_error: error_count,
        diagnostics_warn: warn_count,
    })
}

fn validate_obs_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    let mut epochs = Vec::new();
    let mut events = Vec::new();
    let mut last_t_rx_s: Option<bijux_gnss_receiver::api::core::Seconds> = None;
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: ObsEpochV1 = serde_json::from_str(line).map_err(map_err)?;
        if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
            return Err(InputError {
                message: format!(
                    "unsupported obs schema_version {}",
                    wrapped.header.schema_version
                ),
            });
        }
        if let Some(prev) = last_t_rx_s {
            if wrapped.payload.t_rx_s.0 < prev.0 {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_OBS_TIME_NON_MONOTONIC",
                    "obs t_rx_s is not monotonic",
                ));
            }
        }
        last_t_rx_s = Some(wrapped.payload.t_rx_s);
        events.extend(wrapped.validate());
        epochs.push(wrapped.payload);
    }
    if let Err(err) = bijux_gnss_receiver::api::core::validate_obs_epochs(&epochs) {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_OBS_VALIDATE_FAILED",
            format!("obs epoch validation failed: {err}"),
        ));
    }
    Ok(events)
}

fn validate_acq_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    validate_wrapped_payloads::<AcqResultV1>(data, "acq")
}

fn validate_track_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    let mut last_sample_index: Option<u64> = None;
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: TrackEpochV1 = serde_json::from_str(line).map_err(map_err)?;
        validate_schema_version(wrapped.header.schema_version, "track")?;
        if let Some(prev) = last_sample_index {
            if wrapped.payload.sample_index < prev {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_TRACK_SAMPLE_NON_MONOTONIC",
                    "track sample index is not monotonic",
                ));
            }
        }
        last_sample_index = Some(wrapped.payload.sample_index);
        events.extend(wrapped.payload.validate_payload());
    }
    Ok(events)
}

fn validate_nav_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    let mut last_t_rx_s: Option<bijux_gnss_receiver::api::core::Seconds> = None;
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: NavSolutionEpochV1 = serde_json::from_str(line).map_err(map_err)?;
        validate_schema_version(wrapped.header.schema_version, "pvt")?;
        if let Some(prev) = last_t_rx_s {
            if wrapped.payload.t_rx_s.0 < prev.0 {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_TIME_NON_MONOTONIC",
                    "nav t_rx_s is not monotonic",
                ));
            }
        }
        last_t_rx_s = Some(wrapped.payload.t_rx_s);
        events.extend(wrapped.payload.validate_payload());
    }
    Ok(events)
}

fn validate_wrapped_payloads<T>(data: &str, kind: &str) -> Result<Vec<DiagnosticEvent>, InputError>
where
    T: serde::de::DeserializeOwned + WrappedPayloadValidate,
{
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: T = serde_json::from_str(line).map_err(map_err)?;
        validate_schema_version(wrapped.header().schema_version, kind)?;
        events.extend(wrapped.validate_payload());
    }
    Ok(events)
}

fn validate_schema_version(schema_version: u32, kind: &str) -> Result<(), InputError> {
    if schema_version != ArtifactReadPolicy::LATEST {
        return Err(InputError {
            message: format!("unsupported {kind} schema_version {schema_version}"),
        });
    }
    Ok(())
}

trait WrappedPayloadValidate {
    fn header(&self) -> &ArtifactHeaderV1;
    fn validate_payload(&self) -> Vec<DiagnosticEvent>;
}

impl WrappedPayloadValidate for AcqResultV1 {
    fn header(&self) -> &ArtifactHeaderV1 {
        &self.header
    }

    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        self.payload.validate_payload()
    }
}

impl WrappedPayloadValidate for TrackEpochV1 {
    fn header(&self) -> &ArtifactHeaderV1 {
        &self.header
    }

    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        self.payload.validate_payload()
    }
}

impl WrappedPayloadValidate for NavSolutionEpochV1 {
    fn header(&self) -> &ArtifactHeaderV1 {
        &self.header
    }

    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        self.payload.validate_payload()
    }
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}

fn detect_kind_from_path(path: &Path) -> Option<String> {
    let name = path.file_name()?.to_string_lossy().to_lowercase();
    if name.contains("obs") {
        return Some("obs".to_string());
    }
    if name.contains("track") {
        return Some("track".to_string());
    }
    if name.contains("acq") {
        return Some("acq".to_string());
    }
    if name.contains("eph") {
        return Some("eph".to_string());
    }
    if name.contains("pvt") || name.contains("nav") {
        return Some("pvt".to_string());
    }
    if name.contains("ppp") {
        return Some("ppp".to_string());
    }
    if name.contains("rtk") {
        return Some("rtk".to_string());
    }
    None
}

#[cfg(test)]
mod tests {
    use super::{artifact_explain, artifact_validate};
    use bijux_gnss_receiver::api::core::{
        AcqHypothesis, AcqResult, AcqResultV1, ArtifactHeaderV1, Constellation, Epoch, Hertz,
        Meters, NavLifecycleState, NavSolutionEpoch, NavSolutionEpochV1, NavUncertaintyClass,
        ReceiverSampleTrace, SatId, Seconds, SignalBand, SolutionStatus, SolutionValidity, TrackEpoch,
        TrackEpochV1, NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    };
    use std::fs;
    use tempfile::tempdir;

    fn header() -> ArtifactHeaderV1 {
        ArtifactHeaderV1 {
            schema_version: 1,
            producer: "bijux-gnss-infra-test".to_string(),
            producer_version: "0.1.0".to_string(),
            created_at_unix_ms: 1,
            git_sha: "unknown".to_string(),
            config_hash: "fixture".to_string(),
            dataset_id: None,
            toolchain: "rustc test".to_string(),
            features: Vec::new(),
            deterministic: true,
            git_dirty: false,
        }
    }

    #[test]
    fn artifact_validate_accepts_acquisition_trace() {
        let dir = tempdir().expect("tempdir");
        let path = dir.path().join("acq.jsonl");
        let wrapped = AcqResultV1 {
            header: header(),
            payload: AcqResult {
                sat: SatId { constellation: Constellation::Gps, prn: 3 },
                signal_band: SignalBand::L1,
                source_time: ReceiverSampleTrace::from_sample_index(4_092, 4_092_000.0),
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(500.0),
                carrier_hz: Hertz(500.0),
                code_phase_samples: 17,
                peak: 10.0,
                second_peak: 2.0,
                mean: 1.0,
                peak_mean_ratio: 10.0,
                peak_second_ratio: 5.0,
                cn0_proxy: 42.0,
                score: 0.9,
                hypothesis: AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: Some("accepted_peak".to_string()),
                doppler_refinement: None,
                code_phase_refinement: None,
                uncertainty: None,
            },
        };
        fs::write(&path, serde_json::to_string(&wrapped).expect("serialize")).expect("write");

        let result = artifact_validate(&path, None, true).expect("validate artifact");
        assert_eq!(result.kind, "acq");
        assert!(result.diagnostics.is_empty(), "diagnostics={:?}", result.diagnostics);
    }

    #[test]
    fn artifact_validate_rejects_non_monotonic_track_sample_index() {
        let dir = tempdir().expect("tempdir");
        let path = dir.path().join("track.jsonl");
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
        let rows = [32_u64, 16_u64]
            .into_iter()
            .map(|sample_index| TrackEpochV1 {
                header: header(),
                payload: TrackEpoch {
                    epoch: Epoch { index: sample_index / 16 },
                    sample_index,
                    source_time: ReceiverSampleTrace::from_sample_index(sample_index, 16_000.0),
                    sat,
                    prompt_i: 1.0,
                    prompt_q: 0.0,
                    early_i: 0.0,
                    early_q: 0.0,
                    late_i: 0.0,
                    late_q: 0.0,
                    carrier_hz: Hertz(500.0),
                    code_rate_hz: Hertz(1_023_000.0),
                    code_phase_samples: bijux_gnss_receiver::api::core::Chips(0.0),
                    lock: true,
                    cn0_dbhz: 45.0,
                    pll_lock: true,
                    dll_lock: true,
                    fll_lock: true,
                    cycle_slip: false,
                    nav_bit_lock: false,
                    dll_err: 0.0,
                    pll_err: 0.0,
                    fll_err: 0.0,
                    anti_false_lock: false,
                    cycle_slip_reason: None,
                    lock_state: "tracking".to_string(),
                    lock_state_reason: None,
                    channel_id: None,
                    channel_uid: String::new(),
                    tracking_provenance: String::new(),
                    tracking_assumptions: None,
                    processing_ms: None,
                },
            })
            .map(|wrapped| serde_json::to_string(&wrapped).expect("serialize"))
            .collect::<Vec<_>>();
        fs::write(&path, rows.join("\n")).expect("write");

        let result = artifact_validate(&path, None, true).expect("validate artifact");
        assert!(result
            .diagnostics
            .iter()
            .any(|event| event.code == "GNSS_TRACK_SAMPLE_NON_MONOTONIC"));
    }

    #[test]
    fn artifact_explain_reads_nav_trace_artifact() {
        let dir = tempdir().expect("tempdir");
        let path = dir.path().join("pvt.jsonl");
        let wrapped = NavSolutionEpochV1 {
            header: header(),
            payload: NavSolutionEpoch {
                epoch: Epoch { index: 9 },
                t_rx_s: Seconds(0.25),
                source_time: ReceiverSampleTrace::from_sample_index(1_023_000, 4_092_000.0),
                ecef_x_m: Meters(1.0),
                ecef_y_m: Meters(2.0),
                ecef_z_m: Meters(3.0),
                latitude_deg: 60.0,
                longitude_deg: 18.0,
                altitude_m: Meters(4.0),
                clock_bias_s: Seconds(0.0),
                clock_drift_s_per_s: 0.0,
                pdop: 1.0,
                rms_m: Meters(2.0),
                status: SolutionStatus::Converged,
                quality: SolutionStatus::Converged.quality_flag(),
                validity: SolutionValidity::Stable,
                valid: true,
                processing_ms: None,
                residuals: Vec::new(),
                health: Vec::new(),
                isb: Vec::new(),
                sigma_h_m: None,
                sigma_v_m: None,
                innovation_rms_m: None,
                normalized_innovation_rms: None,
                normalized_innovation_max: None,
                ekf_innovation_rms: None,
                ekf_condition_number: None,
                ekf_whiteness_ratio: None,
                ekf_predicted_variance: None,
                ekf_observed_variance: None,
                integrity_hpl_m: None,
                integrity_vpl_m: None,
                model_version: NAV_SOLUTION_MODEL_VERSION,
                lifecycle_state: NavLifecycleState::Converged,
                uncertainty_class: NavUncertaintyClass::Low,
                assumptions: None,
                refusal_class: None,
                artifact_id: "nav-epoch-0000000009-test".to_string(),
                source_observation_epoch_id: "epoch-0000000009-sample-000001023000".to_string(),
                explain_decision: "accepted".to_string(),
                explain_reasons: vec!["navigation_solution_usable".to_string()],
                provenance: None,
                sat_count: 4,
                used_sat_count: 4,
                rejected_sat_count: 0,
                hdop: None,
                vdop: None,
                gdop: None,
                stability_signature: "navsig:v1:test".to_string(),
                stability_signature_version: NAV_OUTPUT_STABILITY_SIGNATURE_VERSION,
            },
        };
        fs::write(&path, serde_json::to_string(&wrapped).expect("serialize")).expect("write");

        let result = artifact_explain(&path).expect("explain artifact");
        assert_eq!(result.kind, "pvt");
        assert_eq!(result.entries, 1);
        assert_eq!(result.diagnostics_error, 0);
    }
}
