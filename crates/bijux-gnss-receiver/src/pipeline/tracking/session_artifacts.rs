pub type CorrelatorOutput = bijux_gnss_signal::api::EarlyPromptLateCorrelation;

#[derive(Debug, Clone)]
pub struct TrackingResult {
    pub sat: SatId,
    pub carrier_hz: f64,
    pub code_phase_samples: f64,
    pub acquisition_hypothesis: String,
    pub acquisition_score: f32,
    pub acquisition_code_phase_samples: usize,
    pub acquisition_carrier_hz: f64,
    pub acq_to_track_state: String,
    pub epochs: Vec<TrackEpoch>,
    pub transitions: Vec<TrackTransition>,
}

#[derive(Debug, Default, Clone)]
pub struct TrackingArtifacts {
    pub processed_input_samples: u64,
    pub processed_input_epochs: u64,
    pub track_transitions: Vec<TrackTransition>,
    pub channel_state_reports: Vec<TrackingChannelStateReport>,
    pub tracking: Vec<TrackingResult>,
    pub common_frequency: Option<CommonTrackingFrequencyEstimate>,
}

#[derive(Debug, Clone)]
pub struct TrackingSession {
    tracking: IncrementalTrackingState,
    processed_input_samples: u64,
    processed_input_epochs: u64,
}
