#![allow(missing_docs)]

mod support;

use support::orbit_reference::{broadcast_reference_fixtures, orbit_reference_samples};

#[test]
fn broadcast_orbit_reference_fixtures_cover_each_compared_epoch() {
    for fixture in broadcast_reference_fixtures() {
        assert_eq!(
            fixture.transmit_times_s.len(),
            fixture.reference_times_s.len(),
            "{} fixture must pair every transmit epoch with a reference epoch",
            fixture.label
        );

        let (coverage_start_s, coverage_end_s) =
            fixture.sp3.coverage_s(fixture.sat).expect("SP3 coverage for orbit reference fixture");
        let samples = orbit_reference_samples(&fixture);
        assert!(
            !samples.is_empty(),
            "{} fixture must provide at least one precise orbit sample",
            fixture.label
        );

        let mut previous_reference_t_s = None;
        for sample in samples {
            assert!(
                sample.reference_t_s >= coverage_start_s && sample.reference_t_s <= coverage_end_s,
                "{} sample at {:.1}s falls outside SP3 coverage [{:.1}, {:.1}]",
                sample.label,
                sample.reference_t_s,
                coverage_start_s,
                coverage_end_s
            );
            if let Some(previous_reference_t_s) = previous_reference_t_s {
                assert!(
                    sample.reference_t_s > previous_reference_t_s,
                    "{} reference epochs must be strictly increasing",
                    sample.label
                );
            }
            previous_reference_t_s = Some(sample.reference_t_s);
        }
    }
}
