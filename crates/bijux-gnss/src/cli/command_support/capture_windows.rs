use super::*;

pub(crate) const TRACKING_HISTORY_CODE_PERIODS: usize = 80;

pub(crate) struct SampleWindowSource<S> {
    inner: S,
    remaining_samples: usize,
}

impl<S> SampleWindowSource<S> {
    fn new(inner: S, sample_limit: usize) -> Self {
        Self { inner, remaining_samples: sample_limit }
    }
}

impl<S> SignalSource for SampleWindowSource<S>
where
    S: SignalSource + 'static,
{
    type Error = S::Error;

    fn sample_rate_hz(&self) -> f64 {
        self.inner.sample_rate_hz()
    }

    fn next_frame(&mut self, frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error> {
        if self.remaining_samples == 0 {
            return Ok(None);
        }
        let requested_samples = self.remaining_samples.min(frame_len.max(1));
        let Some(frame) = self.inner.next_frame(requested_samples)? else {
            self.remaining_samples = 0;
            return Ok(None);
        };
        self.remaining_samples = self.remaining_samples.saturating_sub(frame.len());
        Ok(Some(frame))
    }

    fn is_done(&self) -> bool {
        self.remaining_samples == 0 || self.inner.is_done()
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

pub(crate) fn tracking_window_sample_count(
    config: &ReceiverPipelineConfig,
    metadata: &RawIqMetadata,
) -> usize {
    samples_per_code(metadata.sample_rate_hz, config.code_freq_basis_hz, config.code_length)
        .saturating_mul(TRACKING_HISTORY_CODE_PERIODS)
}

pub(crate) fn load_acquisition_frame(
    path: &Path,
    config: &ReceiverPipelineConfig,
    metadata: &RawIqMetadata,
) -> Result<SamplesFrame> {
    let code_periods =
        acquisition_code_periods(config.acquisition_integration_ms, config.acquisition_noncoherent);
    load_frame_window(path, config, metadata, code_periods)
}

pub(crate) fn load_tracking_frame(
    path: &Path,
    config: &ReceiverPipelineConfig,
    metadata: &RawIqMetadata,
) -> Result<SamplesFrame> {
    let samples_per_code =
        samples_per_code(metadata.sample_rate_hz, config.code_freq_basis_hz, config.code_length);
    let desired_samples = tracking_window_sample_count(config, metadata);
    let mut source = FileSamples::open_raw_iq(path, metadata.clone())
        .with_context(|| format!("failed to open {}", path.display()))?;

    let mut frame = match source.next_frame(desired_samples)? {
        Some(frame) => frame,
        None => bail!("no samples available in {}", path.display()),
    };
    if frame.len() < samples_per_code {
        bail!("not enough samples: need {samples_per_code}, got {}", frame.len());
    }
    if config.remove_dc_offset {
        bijux_gnss_infra::api::signal::remove_dc_offset_in_place(&mut frame.iq);
    }
    Ok(frame)
}

pub(crate) fn open_tracking_window_source(
    path: &Path,
    config: &ReceiverPipelineConfig,
    metadata: &RawIqMetadata,
) -> Result<SampleWindowSource<FileSamples>> {
    let source = FileSamples::open_raw_iq(path, metadata.clone())
        .with_context(|| format!("failed to open {}", path.display()))?;
    Ok(SampleWindowSource::new(source, tracking_window_sample_count(config, metadata)))
}

pub(crate) fn acquisition_code_periods(coherent_ms: u32, noncoherent: u32) -> usize {
    coherent_ms.saturating_mul(noncoherent).max(1) as usize
}

pub(crate) fn load_frame_window(
    path: &Path,
    config: &ReceiverPipelineConfig,
    metadata: &RawIqMetadata,
    code_periods: usize,
) -> Result<SamplesFrame> {
    let samples_per_code =
        samples_per_code(metadata.sample_rate_hz, config.code_freq_basis_hz, config.code_length);
    let required_samples = samples_per_code.saturating_mul(code_periods.max(1));
    let mut source = FileSamples::open_raw_iq(path, metadata.clone())
        .with_context(|| format!("failed to open {}", path.display()))?;

    let mut frame = match source.next_frame(required_samples)? {
        Some(frame) => frame,
        None => bail!("no samples available in {}", path.display()),
    };
    if frame.len() < required_samples {
        bail!("not enough samples: need {required_samples}, got {}", frame.len());
    }
    if config.remove_dc_offset {
        bijux_gnss_infra::api::signal::remove_dc_offset_in_place(&mut frame.iq);
    }
    Ok(frame)
}
