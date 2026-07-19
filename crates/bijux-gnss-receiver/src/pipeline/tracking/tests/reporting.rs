use super::*;

#[derive(Debug, Deserialize)]
struct TrackingEventFixture {
    lock: bool,
    anti_false_lock: bool,
    cycle_slip: bool,
}

#[derive(Debug, Deserialize)]
struct TrackingScenarioFixture {
    id: String,
    initial_state: String,
    events: Vec<TrackingEventFixture>,
    expected_final_state: String,
}

#[test]
fn tracking_scenario_fixtures_are_deterministic() {
    for (fixture_name, fixture_raw) in tracking_fixture_specs() {
        let fixture = load_tracking_fixture(fixture_raw, fixture_name);
        let mut state = parse_state(&fixture.initial_state);
        let mut unlocked = 0u8;
        let mut degraded_epochs = 0u16;
        for event in &fixture.events {
            let decision = super::deterministic_transition_rule(super::ChannelTransitionRequest {
                from_state: state,
                lock: event.lock,
                ready_for_tracking: event.lock,
                anti_false_lock: event.anti_false_lock,
                loss_of_lock_cause: event.cycle_slip.then_some(super::LossOfLockCause::PhaseJump),
                unlocked_count: unlocked,
                degraded_epochs,
                short_fade_epoch_budget: 100,
                short_fade_relock_evidence: false,
                degraded_tracking_reason: None,
            });
            state = decision.to_state;
            unlocked = decision.next_unlocked_count;
            degraded_epochs = decision.next_degraded_epochs;
        }
        assert_eq!(
            state,
            parse_state(&fixture.expected_final_state),
            "fixture {} final state mismatch",
            fixture.id
        );
    }
}

fn tracking_result_with_epochs(
    channel_id: u8,
    sat: SatId,
    epochs: Vec<TrackEpoch>,
) -> super::TrackingResult {
    let epochs = epochs
        .into_iter()
        .map(|mut epoch| {
            epoch.channel_id = Some(channel_id);
            epoch.channel_uid = super::tracking_channel_uid(sat, channel_id);
            epoch.sat = sat;
            epoch
        })
        .collect();
    super::TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "accepted".to_string(),
        epochs,
        transitions: Vec::new(),
    }
}

#[test]
fn tracking_channel_state_report_emits_unique_steady_states_and_reacquired_marker() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
        2,
        sat,
        vec![
            track_epoch_with_state(0, false, "pull_in", Some("carrier_pull_in")),
            track_epoch_with_state(1, true, "tracking", Some("carrier_converged")),
            track_epoch_with_state(2, false, "lost", Some("prompt_power_drop")),
            track_epoch_with_state(3, false, "pull_in", Some("carrier_pull_in")),
            track_epoch_with_state(4, true, "tracking", Some("reacquired")),
            track_epoch_with_state(5, true, "tracking", Some("stable_tracking")),
        ],
    ));

    let emitted_states = report.emitted_states.iter().map(|event| event.state).collect::<Vec<_>>();
    assert_eq!(
        emitted_states,
        vec![
            super::TrackingChannelState::Acquired,
            super::TrackingChannelState::PullIn,
            super::TrackingChannelState::Locked,
            super::TrackingChannelState::Lost,
            super::TrackingChannelState::PullIn,
            super::TrackingChannelState::Locked,
            super::TrackingChannelState::Reacquired,
        ]
    );
    assert_eq!(report.final_state, super::TrackingChannelState::Locked);
    assert_eq!(report.final_reason.as_deref(), Some("stable_tracking"));
}

#[test]
fn tracking_channel_state_report_marks_cn0_lock_refusal() {
    let sat = SatId { constellation: Constellation::Gps, prn: 16 };
    let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
        1,
        sat,
        vec![
            track_epoch_with_state(0, false, "pull_in", Some("carrier_pull_in")),
            track_epoch_with_state(1, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
            track_epoch_with_state(2, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
        ],
    ));

    let emitted_states = report.emitted_states.iter().map(|event| event.state).collect::<Vec<_>>();
    assert_eq!(
        emitted_states,
        vec![
            super::TrackingChannelState::Acquired,
            super::TrackingChannelState::PullIn,
            super::TrackingChannelState::Refused,
        ]
    );
    assert_eq!(report.final_state, super::TrackingChannelState::Refused);
    assert_eq!(report.final_reason.as_deref(), Some("cn0_below_tracking_lock_floor"));
}

#[test]
fn tracking_channel_state_report_keeps_degraded_final_state() {
    let sat = SatId { constellation: Constellation::Gps, prn: 9 };
    let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
        3,
        sat,
        vec![
            track_epoch_with_state(0, true, "tracking", Some("carrier_converged")),
            track_epoch_with_state(1, true, "degraded", Some("signal_fade")),
        ],
    ));

    assert_eq!(
        report.emitted_states.iter().map(|event| event.state).collect::<Vec<_>>(),
        vec![
            super::TrackingChannelState::Acquired,
            super::TrackingChannelState::Locked,
            super::TrackingChannelState::Degraded,
        ]
    );
    assert_eq!(report.final_state, super::TrackingChannelState::Degraded);
    assert_eq!(report.final_reason.as_deref(), Some("signal_fade"));
}

#[test]
fn tracking_channel_state_report_suppresses_duplicate_refused_markers() {
    let sat = SatId { constellation: Constellation::Gps, prn: 16 };
    let report = super::tracking_channel_state_report(&tracking_result_with_epochs(
        1,
        sat,
        vec![
            track_epoch_with_state(0, false, "pull_in", Some("carrier_pull_in")),
            track_epoch_with_state(1, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
            track_epoch_with_state(2, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
            track_epoch_with_state(3, false, "pull_in", Some("cn0_below_tracking_lock_floor")),
        ],
    ));

    let refused_count = report
        .emitted_states
        .iter()
        .filter(|event| event.state == super::TrackingChannelState::Refused)
        .count();
    assert_eq!(refused_count, 1, "{report:?}");
}

#[test]
#[cfg(feature = "alloc-audit")]
fn tracking_allocations_under_threshold() {
    let config = crate::engine::receiver_config::ReceiverPipelineConfig::default();
    let runtime = crate::engine::runtime::ReceiverRuntime::default();
    let samples_per_code = bijux_gnss_signal::api::samples_per_code(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
    );
    let samples = vec![num_complex::Complex::new(0.0, 0.0); samples_per_code];
    let tracking = Tracking::new(config, runtime);

    let before = crate::engine::alloc::allocation_count();
    let _ = tracking.run(&samples);
    let after = crate::engine::alloc::allocation_count();

    let allocated = after.saturating_sub(before);
    let threshold = 200;
    assert!(
        allocated <= threshold,
        "tracking allocations exceeded threshold: {allocated} > {threshold}"
    );
}

fn tracking_fixture_specs() -> Vec<(&'static str, &'static str)> {
    vec![
        (
            "interference_like.json",
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/tests/data/tracking/interference_like.json"
            )),
        ),
        (
            "lock.json",
            include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/tracking/lock.json")),
        ),
        (
            "relock.json",
            include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/tracking/relock.json")),
        ),
        (
            "weak_signal.json",
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/tests/data/tracking/weak_signal.json"
            )),
        ),
    ]
}

fn load_tracking_fixture(raw: &str, fixture_name: &str) -> TrackingScenarioFixture {
    serde_json::from_str(raw)
        .unwrap_or_else(|err| panic!("parse tracking fixture {fixture_name}: {err}"))
}

fn parse_state(value: &str) -> ChannelState {
    match value {
        "Idle" => ChannelState::Idle,
        "Acquired" => ChannelState::Acquired,
        "PullIn" => ChannelState::PullIn,
        "Tracking" => ChannelState::Tracking,
        "Degraded" => ChannelState::Degraded,
        "Lost" => ChannelState::Lost,
        _ => panic!("unsupported state {value}"),
    }
}
