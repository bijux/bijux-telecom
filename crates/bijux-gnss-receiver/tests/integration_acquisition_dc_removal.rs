#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SampleTime, SamplesFrame, SatId, Seconds};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    Receiver, ReceiverConfig, ReceiverRuntime, SignalSource,
};

#[derive(Clone)]
struct SingleFrameSource {
    frame: Option<SamplesFrame>,
}

impl SignalSource for SingleFrameSource {
    type Error = bijux_gnss_receiver::api::SampleSourceError;

    fn sample_rate_hz(&self) -> f64 {
        self.frame
            .as_ref()
            .map(|frame| frame.t0.sample_rate_hz)
            .unwrap_or(4_092_000.0)
    }

    fn next_frame(
        &mut self,
        _frame_len: usize,
    ) -> Result<Option<SamplesFrame>, Self::Error> {
        Ok(self.frame.take())
    }

    fn is_done(&self) -> bool {
        self.frame.is_none()
    }
}

#[test]
fn receiver_dc_removal_preserves_or_improves_biased_acquisition_margin() {
    let mut profile = ReceiverConfig::default();
    profile.sample_rate_hz = 4_092_000.0;
    profile.intermediate_freq_hz = 0.0;
    profile.acquisition.doppler_search_hz = 1_000;
    profile.acquisition.doppler_step_hz = 250;
    profile.acquisition.integration_ms = 1;
    profile.acquisition.noncoherent_integration = 1;
    profile.acquisition.peak_mean_threshold = 1.5;
    profile.acquisition.peak_second_threshold = 1.1;

    let pipeline = profile.to_pipeline_config();
    let duration_s = 1_023.0 / profile.code_freq_basis_hz;
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let clean = generate_l1_ca(
        &pipeline,
        SyntheticSignalParams {
            sat,
            doppler_hz: 500.0,
            code_phase_chips: 200.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 35.0,
            data_bit_flip: false,
        },
        0xC0FF_EE11,
        duration_s,
    );

    let mut biased = SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz: clean.t0.sample_rate_hz },
        Seconds(clean.dt_s.0),
        clean.iq.clone(),
    );
    for sample in &mut biased.iq {
        sample.re += 4.0;
        sample.im -= 2.0;
    }

    let mut disabled_profile = profile.clone();
    disabled_profile.front_end.remove_dc_offset = false;
    let mut disabled_source = SingleFrameSource { frame: Some(biased.clone()) };
    let disabled_artifacts = Receiver::new(disabled_profile.to_pipeline_config(), ReceiverRuntime::default())
        .run(&mut disabled_source)
        .expect("run without dc removal");

    let mut enabled_profile = profile;
    enabled_profile.front_end.remove_dc_offset = true;
    let mut enabled_source = SingleFrameSource { frame: Some(biased) };
    let enabled_artifacts = Receiver::new(enabled_profile.to_pipeline_config(), ReceiverRuntime::default())
        .run(&mut enabled_source)
        .expect("run with dc removal");

    let disabled = disabled_artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("uncorrected acquisition for target sat");
    let enabled = enabled_artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("corrected acquisition for target sat");

    assert!(
        enabled.peak_mean_ratio + f32::EPSILON >= disabled.peak_mean_ratio,
        "dc removal reduced peak quality: before={} after={}",
        disabled.peak_mean_ratio,
        enabled.peak_mean_ratio
    );
}
