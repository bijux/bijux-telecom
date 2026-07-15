use super::*;

#[derive(Debug, Clone)]
struct StubProductsProvider {
    state: GpsSatState,
    precise_clock_bias_s: Option<f64>,
}

impl ProductsProvider for StubProductsProvider {
    fn sat_state(
        &self,
        _sat: SatId,
        _t_s: f64,
        _diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatState> {
        Some(self.state.clone())
    }

    fn clock_correction(
        &self,
        _sat: SatId,
        _t_s: f64,
        _diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatelliteClockCorrection> {
        self.precise_clock_bias_s.map(GpsSatelliteClockCorrection::from_bias_s)
    }

    fn coverage_s(&self, _sat: SatId) -> Option<(f64, f64)> {
        None
    }
}

#[test]
fn ppp_filter_uses_precise_clock_correction_when_available() {
    let eph = make_eph(1);
    let t_s = 1_350.0;
    let state = BroadcastProductsProvider::new(vec![eph.clone()])
        .sat_state(eph.sat, t_s, &mut ProductDiagnostics::default())
        .expect("broadcast state");
    let provider = StubProductsProvider { state, precise_clock_bias_s: Some(2.5e-9) };
    let filter = PppFilter::new(PppConfig::default());

    let (_state, clock_bias_s, fallback, _support, _discontinuities) =
        filter.sat_state(&provider, &eph, eph.sat, t_s).expect("PPP precise clock state");

    assert!((clock_bias_s - 2.5e-9).abs() < 1e-18);
    assert!(!fallback);
}

#[test]
fn ppp_filter_falls_back_to_current_broadcast_clock_when_precise_clock_is_missing() {
    let eph = make_eph(1);
    let t_s = 1_350.0;
    let state = BroadcastProductsProvider::new(vec![eph.clone()])
        .sat_state(eph.sat, t_s, &mut ProductDiagnostics::default())
        .expect("broadcast state");
    let provider = StubProductsProvider { state, precise_clock_bias_s: None };
    let filter = PppFilter::new(PppConfig::default());

    let (_state, clock_bias_s, fallback, _support, _discontinuities) = filter
        .sat_state(&provider, &eph, eph.sat, t_s)
        .expect("PPP broadcast fallback clock state");
    let expected = crate::api::gps_satellite_clock_correction(&eph, t_s);

    assert!((clock_bias_s - expected.bias_s).abs() < 1e-18);
    assert!(fallback);
}
