#![allow(missing_docs)]

use std::collections::{BTreeMap, BTreeSet};

use bijux_gnss_core::api::{
    glonass_slot_sat, Constellation, GlonassFrequencyChannel, GlonassSlot, SatId, SignalBand,
    SignalCode,
};
use bijux_gnss_receiver::api::sim::{
    default_supported_acquisition_operating_envelope_signal_cases,
    measure_truth_guided_acquisition_operating_envelopes,
    SyntheticAcquisitionOperatingEnvelopeAxis,
};

#[test]
fn supported_signal_operating_envelope_cases_cover_every_acquisition_supported_signal() {
    let cases = default_supported_acquisition_operating_envelope_signal_cases();

    let expected = expected_supported_acquisition_signals();
    let measured = cases.iter().map(signal_identity_from_case).collect::<BTreeSet<_>>();

    assert_eq!(cases.len(), expected.len());
    assert_eq!(measured, expected);
}

#[test]
fn operating_envelope_report_is_deterministic_and_covers_every_required_axis() {
    let cases = default_supported_acquisition_operating_envelope_signal_cases();
    let trial_seeds = trial_seeds(0x2407_2001, 1);

    let report = measure_truth_guided_acquisition_operating_envelopes(
        &cases,
        &trial_seeds,
        "acquisition_operating_envelope_supported_signals",
        2,
        1,
    );
    let repeated = measure_truth_guided_acquisition_operating_envelopes(
        &cases,
        &trial_seeds,
        "acquisition_operating_envelope_supported_signals",
        2,
        1,
    );

    assert_eq!(report, repeated);
    assert_eq!(report.signals.len(), cases.len());

    let expected_axis_counts = cases
        .iter()
        .map(|case| {
            (
                signal_identity_from_case(case),
                BTreeMap::from([
                    (
                        axis_name(SyntheticAcquisitionOperatingEnvelopeAxis::Cn0DbHz),
                        case.cn0_db_hz_points.len(),
                    ),
                    (
                        axis_name(SyntheticAcquisitionOperatingEnvelopeAxis::IntegrationProfile),
                        case.integration_profiles.len(),
                    ),
                    (
                        axis_name(SyntheticAcquisitionOperatingEnvelopeAxis::DopplerHz),
                        case.doppler_hz_points.len(),
                    ),
                    (
                        axis_name(
                            SyntheticAcquisitionOperatingEnvelopeAxis::ReceiverClockFrequencyBiasHz,
                        ),
                        case.receiver_clock_frequency_bias_hz_points.len(),
                    ),
                    (
                        axis_name(SyntheticAcquisitionOperatingEnvelopeAxis::CodePhaseChips),
                        case.code_phase_chips_points.len(),
                    ),
                ]),
            )
        })
        .collect::<BTreeMap<_, _>>();

    for signal_report in &report.signals {
        let identity = signal_identity_from_report(signal_report);
        let expected_counts = expected_axis_counts.get(&identity).expect("expected signal case");
        let observed_counts = signal_report.points.iter().fold(
            BTreeMap::<&'static str, usize>::new(),
            |mut acc, point| {
                *acc.entry(axis_name(point.axis)).or_default() += 1;
                acc
            },
        );

        assert_eq!(observed_counts, *expected_counts, "{signal_report:?}");
        assert_eq!(
            observed_counts.keys().copied().collect::<BTreeSet<_>>(),
            BTreeSet::from([
                axis_name(SyntheticAcquisitionOperatingEnvelopeAxis::Cn0DbHz),
                axis_name(SyntheticAcquisitionOperatingEnvelopeAxis::IntegrationProfile),
                axis_name(SyntheticAcquisitionOperatingEnvelopeAxis::DopplerHz),
                axis_name(SyntheticAcquisitionOperatingEnvelopeAxis::ReceiverClockFrequencyBiasHz),
                axis_name(SyntheticAcquisitionOperatingEnvelopeAxis::CodePhaseChips),
            ]),
            "{signal_report:?}"
        );
        assert_eq!(
            signal_report.points.len(),
            expected_counts.values().sum::<usize>(),
            "{signal_report:?}"
        );
        assert!(
            signal_report.points.iter().all(|point| {
                point.trial_count == trial_seeds.len()
                    && point.false_alarm_trial_count == trial_seeds.len()
                    && point.accepted_count <= point.trial_count
                    && point.detected_count <= point.trial_count
                    && point.false_alarm_count <= point.false_alarm_trial_count
                    && point.acceptance_probability.is_finite()
                    && point.detection_probability.is_finite()
                    && point.false_alarm_rate.is_finite()
                    && (0.0..=1.0).contains(&point.acceptance_probability)
                    && (0.0..=1.0).contains(&point.detection_probability)
                    && (0.0..=1.0).contains(&point.false_alarm_rate)
            }),
            "{signal_report:?}"
        );
    }
}

fn expected_supported_acquisition_signals() -> BTreeSet<SignalIdentity> {
    BTreeSet::from([
        signal_identity(
            SatId { constellation: Constellation::Gps, prn: 7 },
            None,
            SignalBand::L1,
            SignalCode::Ca,
        ),
        signal_identity(
            SatId { constellation: Constellation::Gps, prn: 12 },
            None,
            SignalBand::L5,
            SignalCode::L5I,
        ),
        signal_identity(
            SatId { constellation: Constellation::Gps, prn: 24 },
            None,
            SignalBand::L5,
            SignalCode::L5Q,
        ),
        signal_identity(
            SatId { constellation: Constellation::Galileo, prn: 11 },
            None,
            SignalBand::E1,
            SignalCode::E1B,
        ),
        signal_identity(
            SatId { constellation: Constellation::Galileo, prn: 11 },
            None,
            SignalBand::E5,
            SignalCode::E5a,
        ),
        signal_identity(
            SatId { constellation: Constellation::Galileo, prn: 11 },
            None,
            SignalBand::E5,
            SignalCode::E5b,
        ),
        signal_identity(
            glonass_slot_sat(GlonassSlot::new(8).expect("slot 8 must be valid")),
            Some(GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid")),
            SignalBand::L1,
            SignalCode::Unknown,
        ),
        signal_identity(
            SatId { constellation: Constellation::Beidou, prn: 11 },
            None,
            SignalBand::B1,
            SignalCode::B1I,
        ),
        signal_identity(
            SatId { constellation: Constellation::Beidou, prn: 11 },
            None,
            SignalBand::B2,
            SignalCode::B2I,
        ),
    ])
}

type SignalIdentity = String;

fn signal_identity_from_case(
    case: &bijux_gnss_receiver::api::sim::SyntheticAcquisitionOperatingEnvelopeSignalCase,
) -> SignalIdentity {
    signal_identity(
        case.signal.sat,
        case.signal.glonass_frequency_channel,
        case.signal.signal_band,
        case.signal.signal_code,
    )
}

fn signal_identity_from_report(
    report: &bijux_gnss_receiver::api::sim::SyntheticAcquisitionOperatingEnvelopeSignalReport,
) -> SignalIdentity {
    signal_identity(
        report.sat,
        report.glonass_frequency_channel,
        report.signal_band,
        report.signal_code,
    )
}

fn signal_identity(
    sat: SatId,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> SignalIdentity {
    format!(
        "{:?}:{}:{:?}:{:?}:{}",
        sat.constellation,
        sat.prn,
        signal_band,
        signal_code,
        glonass_frequency_channel
            .map(|channel| channel.value().to_string())
            .unwrap_or_else(|| "none".to_string()),
    )
}

fn axis_name(axis: SyntheticAcquisitionOperatingEnvelopeAxis) -> &'static str {
    match axis {
        SyntheticAcquisitionOperatingEnvelopeAxis::Cn0DbHz => "cn0",
        SyntheticAcquisitionOperatingEnvelopeAxis::IntegrationProfile => "integration",
        SyntheticAcquisitionOperatingEnvelopeAxis::DopplerHz => "doppler",
        SyntheticAcquisitionOperatingEnvelopeAxis::ReceiverClockFrequencyBiasHz => "clock_bias",
        SyntheticAcquisitionOperatingEnvelopeAxis::CodePhaseChips => "code_phase",
    }
}

fn trial_seeds(base_seed: u64, count: usize) -> Vec<u64> {
    (0..count)
        .map(|index| base_seed.wrapping_add((index as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)))
        .collect()
}
