use super::*;

#[test]
fn sustained_lock_loss_reacquire_seed_ignores_degraded_short_fade_epochs() {
    let epochs = vec![
        track_epoch_with_state(0, false, "degraded", Some("signal_fade")),
        track_epoch_with_state(1, false, "degraded", Some("signal_fade")),
        track_epoch_with_state(2, false, "degraded", Some("signal_fade")),
    ];

    assert_eq!(super::sustained_lock_loss_reacquire_seed(&epochs), None);
}

#[test]
fn sustained_lock_loss_reacquire_seed_requires_three_lost_epochs() {
    let epochs = vec![
        track_epoch_with_state(0, false, "lost", Some("lock_lost")),
        track_epoch_with_state(1, false, "lost", Some("lock_lost")),
        track_epoch_with_state(2, false, "lost", Some("lock_lost")),
    ];

    assert_eq!(
        super::sustained_lock_loss_reacquire_seed(&epochs),
        Some(super::SustainedLockLossSeed {
            carrier_hz: 2.0,
            code_phase_samples: 2.5,
            code_rate_hz: 0.0,
            sample_index: 2,
        })
    );
}

#[test]
fn sustained_lock_loss_reacquire_seed_accepts_explicit_loss_causes() {
    let epochs = vec![
        track_epoch_with_state(0, false, "lost", Some("prompt_power_drop")),
        track_epoch_with_state(1, false, "lost", Some("prompt_power_drop")),
        track_epoch_with_state(2, false, "lost", Some("prompt_power_drop")),
    ];

    assert_eq!(
        super::sustained_lock_loss_reacquire_seed(&epochs),
        Some(super::SustainedLockLossSeed {
            carrier_hz: 2.0,
            code_phase_samples: 2.5,
            code_rate_hz: 0.0,
            sample_index: 2,
        })
    );
}

#[test]
fn sustained_lock_loss_reacquire_seed_allows_retry_after_failed_attempt() {
    let epochs = vec![
        track_epoch_with_state(0, false, "lost", Some("reacquisition_failed")),
        track_epoch_with_state(1, false, "lost", Some("reacquisition_failed")),
        track_epoch_with_state(2, false, "lost", Some("reacquisition_failed")),
    ];

    assert_eq!(
        super::sustained_lock_loss_reacquire_seed(&epochs),
        Some(super::SustainedLockLossSeed {
            carrier_hz: 2.0,
            code_phase_samples: 2.5,
            code_rate_hz: 0.0,
            sample_index: 2,
        })
    );
}

#[test]
fn project_reacquisition_code_phase_samples_advances_loss_anchor_to_current_epoch() {
    let seed = super::SustainedLockLossSeed {
        carrier_hz: 1200.0,
        code_phase_samples: 5.5,
        code_rate_hz: 1_023_000.0,
        sample_index: 1_000,
    };

    let projected = super::project_reacquisition_code_phase_samples(
        seed,
        1_000 + 2 * 1_023,
        1_023_000.0,
        1_023,
    );

    assert_eq!(projected, 5.5);
}

fn reacquisition_candidate(
    doppler_bin: i8,
    code_bin: i8,
    carrier_sign: super::ReacquisitionCarrierSign,
    secondary_code_phase_periods: Option<usize>,
    cn0_dbhz: f64,
    prompt_power: f32,
) -> super::ReacquisitionCandidate {
    super::ReacquisitionCandidate {
        hypothesis: super::ReacquisitionHypothesis {
            doppler_bin,
            code_bin,
            carrier_sign,
            secondary_code_phase_periods,
        },
        carrier_hz: 1000.0 + f64::from(doppler_bin),
        code_phase_samples: 42.0 + f64::from(code_bin),
        cn0_dbhz,
        prompt_power,
    }
}

#[test]
fn reacquisition_selection_chooses_strongest_hypothesis_not_first_peak() {
    let selection = super::select_reacquisition_candidate(
        &[
            reacquisition_candidate(
                -2,
                -2,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                39.0,
                200.0,
            ),
            reacquisition_candidate(
                1,
                1,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                44.0,
                300.0,
            ),
        ],
        35.0,
        0.0,
    );

    assert_eq!(
        selection,
        super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
            carrier_hz: 1001.0,
            code_phase_samples: 43.0,
            cn0_dbhz: 44.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        })
    );
}

#[test]
fn reacquisition_selection_refuses_ambiguous_hypotheses() {
    let selection = super::select_reacquisition_candidate(
        &[
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                44.0,
                300.0,
            ),
            reacquisition_candidate(
                2,
                2,
                super::ReacquisitionCarrierSign::Inverted,
                None,
                43.0,
                292.0,
            ),
        ],
        35.0,
        0.0,
    );

    assert_eq!(selection, super::ReacquisitionSelection::Refused);
}

#[test]
fn reacquisition_selection_refuses_competing_secondary_code_phases() {
    let selection = super::select_reacquisition_candidate(
        &[
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                Some(3),
                44.0,
                300.0,
            ),
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                Some(4),
                43.5,
                294.0,
            ),
        ],
        35.0,
        0.0,
    );

    assert_eq!(selection, super::ReacquisitionSelection::Refused);
}

#[test]
fn reacquisition_selection_treats_carrier_sign_only_tie_as_same_location() {
    let selection = super::select_reacquisition_candidate(
        &[
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                44.0,
                300.0,
            ),
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Inverted,
                None,
                44.0,
                300.0,
            ),
        ],
        35.0,
        0.0,
    );

    assert_eq!(
        selection,
        super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
            carrier_hz: 1000.0,
            code_phase_samples: 42.0,
            cn0_dbhz: 44.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        })
    );
}

#[test]
fn reacquisition_selection_preserves_aligned_sign_for_same_location() {
    let selection = super::select_reacquisition_candidate(
        &[
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Aligned,
                None,
                44.0,
                300.0,
            ),
            reacquisition_candidate(
                0,
                0,
                super::ReacquisitionCarrierSign::Inverted,
                None,
                44.1,
                301.0,
            ),
        ],
        35.0,
        0.0,
    );

    assert_eq!(
        selection,
        super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
            carrier_hz: 1000.0,
            code_phase_samples: 42.0,
            cn0_dbhz: 44.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        })
    );
}

#[test]
fn reacquisition_selection_accepts_strong_prompt_when_cn0_is_conservative() {
    let selection = super::select_reacquisition_candidate(
        &[reacquisition_candidate(
            0,
            0,
            super::ReacquisitionCarrierSign::Aligned,
            None,
            45.0,
            300_000.0,
        )],
        48.0,
        100_000.0,
    );

    assert_eq!(
        selection,
        super::ReacquisitionSelection::Accepted(super::ReacquisitionSeed {
            carrier_hz: 1000.0,
            code_phase_samples: 42.0,
            cn0_dbhz: 45.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        })
    );
}

#[test]
fn reacquisition_selection_rejects_strong_prompt_when_cn0_is_too_low() {
    let selection = super::select_reacquisition_candidate(
        &[reacquisition_candidate(
            0,
            0,
            super::ReacquisitionCarrierSign::Aligned,
            None,
            41.0,
            300_000.0,
        )],
        48.0,
        100_000.0,
    );

    assert_eq!(selection, super::ReacquisitionSelection::Refused);
}

#[test]
fn quick_reacquire_recovers_offset_code_hypothesis() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 70.0,
            navigation_data: false.into(),
        },
        0x51AC_0301,
        0.001,
    );
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());

    let seed = tracking
        .quick_reacquire(&frame, sat, 0.0, 2.0, 35.0, 0.0, None)
        .expect("reacquisition seed");
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let code_error =
        wrapped_code_phase_delta_samples(seed.code_phase_samples, 0.0, samples_per_code).abs();

    assert!(
            code_error <= 0.1,
            "reacquisition should recover the actual code hypothesis: seed={seed:?}, code_error={code_error}"
        );
    assert!((seed.carrier_hz - 0.0).abs() <= f64::EPSILON, "{seed:?}");
}

#[test]
fn reacquisition_secondary_code_phases_collapse_when_signal_has_no_secondary_code() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        SignalCode::Ca,
        None,
    );

    assert_eq!(super::reacquisition_secondary_code_phase_periods(&signal_model), vec![None]);
}

#[test]
fn reacquisition_secondary_code_phases_cover_supported_period() {
    let config = ReceiverPipelineConfig::default();
    let signal_model = super::TrackingSignalModel::for_sat_signal_band(
        &config,
        SatId { constellation: Constellation::Gps, prn: 18 },
        SignalBand::L5,
        SignalCode::L5I,
        None,
    );
    let phases = super::reacquisition_secondary_code_phase_periods(&signal_model);

    assert!(phases.len() > 1);
    assert_eq!(phases.first(), Some(&Some(0)));
    assert_eq!(phases.last(), Some(&Some(phases.len() - 1)));
}

#[test]
fn reacquisition_seed_matches_respects_acquisition_uncertainty_tolerances() {
    let tracking = Tracking::new(ReceiverPipelineConfig::default(), ReceiverRuntime::default());
    let uncertainty = AcqUncertainty {
        doppler_hz: 250.0,
        code_phase_samples: 0.75,
        doppler_rate_hz_per_s: None,
        covariance: None,
    };

    assert!(tracking.reacquisition_seed_matches(
        super::ReacquisitionSeed {
            carrier_hz: 1250.0,
            code_phase_samples: 42.0,
            cn0_dbhz: 36.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        },
        super::ReacquisitionSeed {
            carrier_hz: 1400.0,
            code_phase_samples: 42.5,
            cn0_dbhz: 34.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        },
        Some(&uncertainty),
    ));
    assert!(!tracking.reacquisition_seed_matches(
        super::ReacquisitionSeed {
            carrier_hz: 1250.0,
            code_phase_samples: 42.0,
            cn0_dbhz: 36.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        },
        super::ReacquisitionSeed {
            carrier_hz: 1705.0,
            code_phase_samples: 43.0,
            cn0_dbhz: 34.0,
            carrier_sign: super::ReacquisitionCarrierSign::Aligned,
            secondary_code_phase_periods: None,
        },
        Some(&uncertainty),
    ));
}

#[test]
fn reacquisition_min_cn0_dbhz_preserves_lock_reference_headroom() {
    assert_eq!(super::reacquisition_min_cn0_dbhz(60.0), 48.0);
    assert_eq!(super::reacquisition_min_cn0_dbhz(35.0), 28.0);
    assert_eq!(super::reacquisition_min_cn0_dbhz(f64::NAN), super::TRACKING_LOCK_MIN_CN0_DBHZ);
}
