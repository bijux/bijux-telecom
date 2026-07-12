/// Borrowed inputs for one synthetic PVT C/N0 profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtCn0ProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Observation epochs that fed the synthetic PVT solver path.
    pub observations: &'a [ObsEpoch],
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one synthetic PVT geometry profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtGeometryProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Truth-guided PVT accuracy report for the geometry case.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one synthetic mixed-constellation geometry profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtConstellationGeometryProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Constellations intentionally enabled for this validation point.
    pub constellations: &'a [Constellation],
    /// Number of visible satellites in the controlled scenario.
    pub visible_satellite_count: usize,
    /// Truth-guided PVT truth table for the same scenario.
    pub truth_table: &'a SyntheticPvtTruthTableReport,
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one synthetic PVT multipath profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtMultipathProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Number of satellites carrying injected multipath bias.
    pub affected_satellite_count: usize,
    /// Mean absolute injected pseudorange bias across affected satellites, in meters.
    pub mean_abs_pseudorange_bias_m: f64,
    /// Maximum absolute injected pseudorange bias across affected satellites, in meters.
    pub max_abs_pseudorange_bias_m: f64,
    /// Truth-guided PVT truth table for the same scenario.
    pub truth_table: &'a SyntheticPvtTruthTableReport,
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one synthetic PVT receiver-motion profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtMotionProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Truth-guided PVT truth table for the same scenario.
    pub truth_table: &'a SyntheticPvtTruthTableReport,
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one receiver clock-drift PVT profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtClockProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Injected receiver clock drift carried by the scenario truth, in seconds per second.
    pub injected_clock_drift_s_per_s: f64,
    /// Expected observation-Doppler offset induced by the injected clock drift, in hertz.
    pub expected_observation_doppler_offset_hz: f64,
    /// Observation epochs produced by the drifting or stable receiver-clock scenario.
    pub observations: &'a [ObsEpoch],
    /// Stable-reference observation epochs used to measure the clock-induced Doppler offset.
    pub reference_observations: Option<&'a [ObsEpoch]>,
    /// Navigation solutions that should estimate the receiver clock drift.
    pub solutions: &'a [NavSolutionEpoch],
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}

/// Borrowed inputs for one long-run synthetic PVT time-profile point.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticPvtTimeProfileCase<'a> {
    /// Stable scenario identifier for this validation run.
    pub scenario_id: &'a str,
    /// Truth-guided PVT truth table for the same scenario.
    pub truth_table: &'a SyntheticPvtTruthTableReport,
    /// Truth-guided PVT accuracy report for the same scenario.
    pub accuracy: &'a SyntheticPvtAccuracyReport,
}
