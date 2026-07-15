#[derive(Debug, Clone)]
pub(crate) struct IncrementalTrackingState {
    channels: Vec<IncrementalTrackingChannel>,
    vector_state: VectorTrackingState,
}

#[derive(Debug, Clone)]
struct IncrementalTrackingChannel {
    sat: SatId,
    channel_id: u8,
    start_source_time: ReceiverSampleTrace,
    signal_band: SignalBand,
    signal_model: TrackingSignalModel,
    acquisition_uncertainty: Option<AcqUncertainty>,
    acquisition_hypothesis: String,
    acquisition_score: f32,
    acquisition_code_phase_samples: usize,
    acquisition_doppler_hz: f64,
    acquisition_resolved_code_phase_samples: f64,
    acquisition_carrier_hz: f64,
    acq_to_track_state: String,
    tracking_params: TrackingParams,
    state: LoopState,
    epochs: Vec<TrackEpoch>,
    transitions: Vec<TrackTransition>,
}
