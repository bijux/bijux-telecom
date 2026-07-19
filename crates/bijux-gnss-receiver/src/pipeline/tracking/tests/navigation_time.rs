use super::*;

#[cfg(feature = "nav")]
fn gps_l1ca_tracking_signal_model() -> super::TrackingSignalModel {
    let config = ReceiverPipelineConfig::default();
    super::TrackingSignalModel::for_sat(
        &config,
        SatId { constellation: Constellation::Gps, prn: 1 },
    )
}

#[cfg(feature = "nav")]
fn prompt_history_epochs(prompt_i: &[f32]) -> Vec<TrackEpoch> {
    prompt_i
        .iter()
        .enumerate()
        .map(|(index, prompt_i)| TrackEpoch {
            epoch: Epoch { index: index as u64 },
            sample_index: index as u64,
            source_time: ReceiverSampleTrace::from_sample_index(index as u64, 1000.0),
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            prompt_i: *prompt_i,
            tracking_provenance: "tracking".to_string(),
            ..TrackEpoch::default()
        })
        .collect()
}

#[cfg(feature = "nav")]
fn set_lnav_bits(data: &mut u32, start: usize, len: usize, value: u32) {
    let shift = 24 - (start - 1) - len;
    let mask = ((1_u32 << len) - 1) << shift;
    *data &= !mask;
    *data |= (value << shift) & mask;
}

#[cfg(feature = "nav")]
fn lnav_word_parity(data_bits: &[u8], d29_star: u8, d30_star: u8) -> (u8, u8, u8, u8, u8, u8) {
    let d = |i: usize| -> u8 { data_bits[i - 1] };
    let p1 = d(1)
        ^ d(2)
        ^ d(3)
        ^ d(5)
        ^ d(6)
        ^ d(10)
        ^ d(11)
        ^ d(12)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(17)
        ^ d(18)
        ^ d(20)
        ^ d(23)
        ^ d(24)
        ^ d29_star;
    let p2 = d(2)
        ^ d(3)
        ^ d(4)
        ^ d(6)
        ^ d(7)
        ^ d(11)
        ^ d(12)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(18)
        ^ d(19)
        ^ d(21)
        ^ d(24)
        ^ d29_star
        ^ d30_star;
    let p3 = d(1)
        ^ d(3)
        ^ d(4)
        ^ d(5)
        ^ d(7)
        ^ d(8)
        ^ d(12)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(19)
        ^ d(20)
        ^ d(22)
        ^ d29_star
        ^ d30_star;
    let p4 = d(1)
        ^ d(2)
        ^ d(4)
        ^ d(5)
        ^ d(6)
        ^ d(8)
        ^ d(9)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(18)
        ^ d(20)
        ^ d(21)
        ^ d(23)
        ^ d30_star;
    let p5 = d(1)
        ^ d(2)
        ^ d(3)
        ^ d(5)
        ^ d(6)
        ^ d(7)
        ^ d(9)
        ^ d(10)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(18)
        ^ d(19)
        ^ d(21)
        ^ d(22)
        ^ d(24)
        ^ d29_star;
    let p6 = d(2)
        ^ d(3)
        ^ d(4)
        ^ d(6)
        ^ d(7)
        ^ d(8)
        ^ d(10)
        ^ d(11)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(18)
        ^ d(19)
        ^ d(20)
        ^ d(22)
        ^ d(23)
        ^ d(24)
        ^ d30_star;
    (p1, p2, p3, p4, p5, p6)
}

#[cfg(feature = "nav")]
fn encode_lnav_word(data: u32, prev_d29: u8, prev_d30: u8) -> [u8; 30] {
    let mut bits = [0_u8; 30];
    for (index, bit) in bits.iter_mut().enumerate().take(24) {
        let shift = 23 - index;
        *bit = ((data >> shift) & 1) as u8;
    }
    let (p1, p2, p3, p4, p5, p6) = lnav_word_parity(&bits[..24], prev_d29, prev_d30);
    bits[24] = p1;
    bits[25] = p2;
    bits[26] = p3;
    bits[27] = p4;
    bits[28] = p5;
    bits[29] = p6;
    if prev_d30 == 1 {
        for bit in &mut bits {
            *bit = 1 - *bit;
        }
    }
    bits
}

#[cfg(feature = "nav")]
fn encode_lnav_subframe(subframe_id: u8, tow_count: u32) -> Vec<i8> {
    let mut tlm = 0_u32;
    set_lnav_bits(&mut tlm, 1, 8, 0x8B);

    let mut how = 0_u32;
    set_lnav_bits(&mut how, 1, 17, tow_count);
    set_lnav_bits(&mut how, 20, 3, subframe_id as u32);

    let mut words = vec![tlm, how];
    for offset in 0..8_u32 {
        words.push(0x012345 + offset * 0x010101);
    }

    let mut prev_d29 = 0_u8;
    let mut prev_d30 = 0_u8;
    let mut bits = Vec::with_capacity(300);
    for data in words {
        let encoded = encode_lnav_word(data, prev_d29, prev_d30);
        prev_d29 = encoded[28];
        prev_d30 = encoded[29];
        bits.extend(encoded.into_iter().map(|bit| if bit == 1 { 1 } else { -1 }));
    }
    bits
}

#[cfg(feature = "nav")]
fn lnav_prompt_history_with_offset(offset_ms: usize, subframe_id: u8, tow_count: u32) -> Vec<f32> {
    let mut prompt = vec![0.25_f32; offset_ms];
    for bit in encode_lnav_subframe(subframe_id, tow_count) {
        prompt.extend(std::iter::repeat_n(bit as f32, 20));
    }
    prompt
}

#[cfg(feature = "nav")]
#[test]
fn annotate_navigation_bit_signs_waits_for_boundary_confidence() {
    let signal_model = gps_l1ca_tracking_signal_model();
    let prompt = vec![1.0_f32; 60];
    let mut epochs = prompt_history_epochs(&prompt);

    super::annotate_navigation_bit_signs(&signal_model, &mut epochs);

    assert!(
        epochs.iter().all(|epoch| epoch.navigation_bit_sign.is_none()),
        "ambiguous symbol boundaries must not emit bit signs: {epochs:?}"
    );
    assert!(epochs.iter().all(|epoch| !epoch.nav_bit_lock));
    assert!(epochs
        .iter()
        .all(|epoch| !epoch.tracking_provenance.contains("nav_symbol_sync=confident")));
}

#[cfg(feature = "nav")]
#[test]
fn annotate_navigation_bit_signs_emits_confident_symbol_windows() {
    let signal_model = gps_l1ca_tracking_signal_model();
    let mut prompt = vec![0.2_f32; 6];
    prompt.extend(std::iter::repeat_n(1.0_f32, 20));
    prompt.extend(std::iter::repeat_n(-1.0_f32, 20));
    prompt.extend(std::iter::repeat_n(1.0_f32, 20));
    let mut epochs = prompt_history_epochs(&prompt);

    super::annotate_navigation_bit_signs(&signal_model, &mut epochs);

    assert!(epochs[..6].iter().all(|epoch| epoch.navigation_bit_sign.is_none()));
    assert!(epochs[6..26]
        .iter()
        .all(|epoch| epoch.navigation_bit_sign == Some(1) && epoch.nav_bit_lock));
    assert!(epochs[26..46]
        .iter()
        .all(|epoch| epoch.navigation_bit_sign == Some(-1) && epoch.nav_bit_lock));
    assert!(epochs[46..66]
        .iter()
        .all(|epoch| epoch.navigation_bit_sign == Some(1) && epoch.nav_bit_lock));
    assert!(epochs[6..66]
        .iter()
        .all(|epoch| epoch.tracking_provenance.contains("nav_symbol_sync=confident")));
}

#[cfg(feature = "nav")]
#[test]
fn annotate_lnav_transmit_times_maps_decoded_how_to_tracking_epochs() {
    let signal_model = gps_l1ca_tracking_signal_model();
    let tow_count = 57_600;
    let prompt = lnav_prompt_history_with_offset(7, 1, tow_count);
    let mut epochs = prompt_history_epochs(&prompt);

    super::annotate_lnav_transmit_times(
        &signal_model,
        Some(GpsTime { week: 2200, tow_s: 345_600.073 }),
        &mut epochs,
    );

    assert!(epochs[..7].iter().all(|epoch| epoch.transmit_time.is_none()));
    let subframe_start = epochs[7].transmit_time.as_ref().expect("subframe start time");
    assert_eq!(subframe_start.source, "gps_l1ca_lnav_how");
    assert_eq!(subframe_start.transmit_gps_time.week, 2200);
    assert!((subframe_start.transmit_gps_time.tow_s - 345_600.0).abs() <= 1.0e-12);

    let later = epochs[257].transmit_time.as_ref().expect("offset transmit time");
    assert!((later.transmit_gps_time.tow_s - 345_600.250).abs() <= 1.0e-12);
    assert!(epochs[257].tracking_provenance.contains("lnav_transmit_time=decoded"));
}

#[cfg(feature = "nav")]
#[test]
fn lnav_transmit_time_week_resolution_tracks_receive_time_boundary() {
    let epoch = TrackEpoch {
        source_time: ReceiverSampleTrace::from_sample_index(20, 1000.0),
        ..TrackEpoch::default()
    };

    let transmit_time = super::gps_transmit_time_near_receive_time(
        GpsTime { week: 2201, tow_s: 0.010 },
        &epoch,
        604_799.950,
    );

    assert_eq!(transmit_time.week, 2200);
    assert!((transmit_time.tow_s - 604_799.950).abs() <= 1.0e-12);
}

#[cfg(feature = "nav")]
#[test]
fn decoded_lnav_tracking_time_feeds_absolute_observation_pseudorange() {
    let signal_model = gps_l1ca_tracking_signal_model();
    let prompt = lnav_prompt_history_with_offset(7, 1, 57_600);
    let mut epochs = prompt_history_epochs(&prompt);
    let tracking_capture_start = GpsTime { week: 2200, tow_s: 345_600.073 };
    super::annotate_lnav_transmit_times(&signal_model, Some(tracking_capture_start), &mut epochs);

    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let observation_capture_start = GpsTime { week: 2200, tow_s: 345_600.0 };
    let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.330 };
    let mut epoch = epochs[257].clone();
    epoch.sample_index = ((receive_gps_time.tow_s - observation_capture_start.tow_s)
        * config.sampling_freq_hz)
        .round() as u64;
    epoch.source_time =
        ReceiverSampleTrace::from_sample_index(epoch.sample_index, config.sampling_freq_hz);
    epoch.sat = SatId { constellation: Constellation::Gps, prn: 1 };
    epoch.signal_band = SignalBand::L1;
    epoch.signal_code = SignalCode::Ca;
    epoch.code_rate_hz = Hertz(1_023_000.0);
    epoch.carrier_hz = Hertz(0.0);
    epoch.code_phase_samples = Chips(0.0);
    epoch.lock = true;
    epoch.dll_lock = true;
    epoch.pll_lock = true;
    epoch.fll_lock = true;
    epoch.signal_delay_alignment = None;

    let track = super::TrackingResult {
        sat: epoch.sat,
        carrier_hz: epoch.carrier_hz.0,
        code_phase_samples: epoch.code_phase_samples.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: epoch.carrier_hz.0,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    };

    let observations = observations_from_tracking_results_with_gps_anchor(
        &config,
        Some(observation_capture_start),
        &[track],
        10,
    );
    let sat = observations.output[0].sats.first().expect("decoded time observation");

    assert_eq!(sat.metadata.pseudorange_model, "decoded_transmit_time_code_phase");
    assert_eq!(sat.metadata.pseudorange_time_source, "gps_l1ca_lnav_how");
    assert_eq!(sat.metadata.pseudorange_integer_code_periods, Some(80));
    assert_eq!(sat.metadata.signal_delay_alignment_source, "");
    let timing = sat.timing.expect("decoded timing");
    assert!((sat.pseudorange_m.0 - timing.signal_travel_time_s.0 * 299_792_458.0).abs() <= 1.0e-6);
    assert!((timing.signal_travel_time_s.0 - 0.080).abs() <= 2.5e-7);
}
