#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqCodePhaseRefinement, AcqHypothesis, AcqResult, AcqUncertainty, Hertz, ReceiverSampleTrace,
    SignalBand,
};
use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{sat_state_gps_l1ca, GpsEphemeris};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz,
    sim::{SyntheticScenario, SyntheticSignalParams},
    ReceiverPipelineConfig,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const GPS_L1_CA_CODE_RATE_HZ: f64 = 1_023_000.0;
const GPS_L1_CA_CODE_PERIOD_CHIPS: f64 = 1023.0;

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct SyntheticPvtScenario {
    pub scenario: SyntheticScenario,
    pub ephemerides: Vec<GpsEphemeris>,
    pub truth_ecef_m: (f64, f64, f64),
    pub target_epoch_idx: u64,
    pub pseudorange_epoch_base: u64,
}

pub fn four_satellite_pvt_scenario(config: &ReceiverPipelineConfig) -> SyntheticPvtScenario {
    multisatellite_pvt_scenario(config, 0.125, "four-satellite-pvt-readiness")
}

pub fn multisatellite_pvt_scenario(
    config: &ReceiverPipelineConfig,
    duration_s: f64,
    scenario_id: &str,
) -> SyntheticPvtScenario {
    let t_rx_s = 100_000.0;
    let truth_ecef_m = (5_000.0, -4_000.0, 3_000.0);
    let ephemerides = vec![
        make_near_isorange_ephemeris(3, 0.0, 0.0, t_rx_s),
        make_near_isorange_ephemeris(7, 0.8, 0.8, t_rx_s),
        make_near_isorange_ephemeris(11, 1.6, 1.6, t_rx_s),
        make_near_isorange_ephemeris(19, 2.4, 2.4, t_rx_s),
        make_near_isorange_ephemeris(23, 3.2, 3.2, t_rx_s),
    ];
    let pseudorange_chips = ephemerides
        .iter()
        .map(|ephemeris| {
            synthetic_pseudorange_m(ephemeris, t_rx_s, truth_ecef_m) * GPS_L1_CA_CODE_RATE_HZ
                / SPEED_OF_LIGHT_MPS
        })
        .collect::<Vec<_>>();
    let pseudorange_epoch_base = shared_pseudorange_epoch_base(&pseudorange_chips);
    let satellites = ephemerides
        .iter()
        .zip([-1_500.0, -700.0, 0.0, 700.0, 1_500.0])
        .zip(pseudorange_chips.iter().copied())
        .map(|((ephemeris, doppler_hz), pseudorange_chips)| SyntheticSignalParams {
            sat: ephemeris.sat,
            doppler_hz,
            code_phase_chips: encode_code_phase_chips(pseudorange_chips, pseudorange_epoch_base),
            carrier_phase_rad: 0.0,
            cn0_db_hz: 52.0,
            data_bit_flip: false,
        })
        .collect();

    SyntheticPvtScenario {
        scenario: SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s,
            seed: 0x5EED_66,
            satellites,
            ephemerides: ephemerides.clone(),
            id: scenario_id.to_string(),
        },
        ephemerides,
        truth_ecef_m,
        target_epoch_idx: 0,
        pseudorange_epoch_base,
    }
}

pub fn synthetic_pseudorange_m(
    ephemeris: &GpsEphemeris,
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    let mut tau = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let sat = sat_state_gps_l1ca(ephemeris, t_rx_s - tau, tau);
        let dx = truth_ecef_m.0 - sat.x_m;
        let dy = truth_ecef_m.1 - sat.y_m;
        let dz = truth_ecef_m.2 - sat.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = range_m - sat.clock_bias_s * SPEED_OF_LIGHT_MPS;
        let next_tau = pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_tau - tau).abs() < 1e-12 {
            break;
        }
        tau = next_tau;
    }
    pseudorange_m
}

pub fn truth_seeded_acquisition_results(
    config: &ReceiverPipelineConfig,
    source_time: ReceiverSampleTrace,
    scenario: &SyntheticScenario,
) -> Vec<AcqResult> {
    scenario
        .satellites
        .iter()
        .map(|signal| {
            let code_phase_samples = signal.code_phase_chips.round().clamp(0.0, 1022.0);
            AcqResult {
                sat: signal.sat,
                signal_band: SignalBand::L1,
                source_time,
                candidate_rank: 1,
                is_primary_candidate: true,
                doppler_hz: Hertz(signal.doppler_hz),
                carrier_hz: Hertz(carrier_hz_from_doppler_hz(
                    config.intermediate_freq_hz,
                    signal.doppler_hz,
                )),
                code_phase_samples: code_phase_samples as usize,
                peak: 1.0,
                second_peak: 0.1,
                mean: 0.1,
                peak_mean_ratio: 10.0,
                peak_second_ratio: 10.0,
                cn0_proxy: signal.cn0_db_hz,
                score: 1.0,
                hypothesis: AcqHypothesis::Accepted,
                assumptions: None,
                evidence: Vec::new(),
                threshold_provenance: None,
                explain_selection_reason: Some("truth_seed".to_string()),
                doppler_refinement: None,
                code_phase_refinement: Some(AcqCodePhaseRefinement {
                    method: "truth_seed".to_string(),
                    offset_samples: 0.0,
                    refined_code_phase_samples: signal.code_phase_chips,
                    left_correlation_norm: 1.0,
                    center_correlation_norm: 1.0,
                    right_correlation_norm: 1.0,
                }),
                uncertainty: Some(AcqUncertainty { doppler_hz: 1.0, code_phase_samples: 0.25 }),
            }
        })
        .collect()
}

fn make_near_isorange_ephemeris(prn: u8, omega0: f64, m0: f64, t_ref_s: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        toe_s: t_ref_s,
        toc_s: t_ref_s,
        sqrt_a: 5153.7954775,
        e: 0.0,
        i0: 0.94,
        idot: 0.0,
        omega0,
        omegadot: 0.0,
        w: 0.0,
        m0,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 0.0,
        af1: 0.0,
        af2: 0.0,
        tgd: 0.0,
    }
}

fn shared_pseudorange_epoch_base(pseudorange_chips: &[f64]) -> u64 {
    let mut epoch_indices =
        pseudorange_chips.iter().map(|chips| (chips / GPS_L1_CA_CODE_PERIOD_CHIPS).floor() as u64);
    let target_epoch_idx = epoch_indices.next().expect("pseudorange chip counts");
    assert!(
        epoch_indices.all(|epoch_idx| epoch_idx == target_epoch_idx),
        "synthetic pseudoranges must share a receiver epoch base",
    );
    target_epoch_idx
}

fn encode_code_phase_chips(pseudorange_chips: f64, target_epoch_idx: u64) -> f64 {
    let code_phase_chips =
        pseudorange_chips - target_epoch_idx as f64 * GPS_L1_CA_CODE_PERIOD_CHIPS;
    assert!(
        (0.0..GPS_L1_CA_CODE_PERIOD_CHIPS).contains(&code_phase_chips),
        "code phase chips must fit within one C/A period",
    );
    code_phase_chips
}
