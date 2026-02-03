pub fn sat_state_gps_l1ca(eph: &GpsEphemeris, t_tx_s: f64, tau_s: f64) -> GpsSatState {
    let a = eph.sqrt_a * eph.sqrt_a;
    let n0 = (MU / (a * a * a)).sqrt();
    let n = n0 + eph.delta_n;
    let mut tk = t_tx_s - eph.toe_s;
    tk = wrap_time(tk);

    let m = eph.m0 + n * tk;
    let (_e_anom, sin_e, cos_e) = solve_kepler(m, eph.e);
    let v = (1.0 - eph.e * eph.e).sqrt() * sin_e;
    let v = v.atan2(cos_e - eph.e);
    let phi = v + eph.w;
    let sin2phi = (2.0 * phi).sin();
    let cos2phi = (2.0 * phi).cos();
    let u = phi + eph.cuc * cos2phi + eph.cus * sin2phi;
    let r = a * (1.0 - eph.e * cos_e) + eph.crc * cos2phi + eph.crs * sin2phi;
    let i = eph.i0 + eph.idot * tk + eph.cic * cos2phi + eph.cis * sin2phi;

    let x_orb = r * u.cos();
    let y_orb = r * u.sin();

    let omega = eph.omega0 + (eph.omegadot - OMEGA_E_DOT) * tk - OMEGA_E_DOT * eph.toe_s;

    let cos_omega = omega.cos();
    let sin_omega = omega.sin();
    let cos_i = i.cos();
    let sin_i = i.sin();

    let mut x = x_orb * cos_omega - y_orb * cos_i * sin_omega;
    let mut y = x_orb * sin_omega + y_orb * cos_i * cos_omega;
    let z = y_orb * sin_i;

    let rot = OMEGA_E_DOT * tau_s;
    if rot.abs() > 0.0 {
        let cos_rot = rot.cos();
        let sin_rot = rot.sin();
        let xr = cos_rot * x + sin_rot * y;
        let yr = -sin_rot * x + cos_rot * y;
        x = xr;
        y = yr;
    }

    let dt = wrap_time(t_tx_s - eph.toc_s);
    let relativistic = RELATIVISTIC_F * eph.e * eph.sqrt_a * sin_e;
    let clock_bias = eph.af0 + eph.af1 * dt + eph.af2 * dt * dt + relativistic - eph.tgd;
    let clock_drift = eph.af1 + 2.0 * eph.af2 * dt;

    GpsSatState {
        x_m: x,
        y_m: y,
        z_m: z,
        clock_bias_s: clock_bias,
        clock_drift_s: clock_drift,
        relativistic_s: relativistic,
    }
}

pub fn solve_kepler(m: f64, e: f64) -> (f64, f64, f64) {
    let mut e_anom = m;
    for _ in 0..12 {
        let f = e_anom - e * e_anom.sin() - m;
        let f_prime = 1.0 - e * e_anom.cos();
        let step = f / f_prime;
        e_anom -= step;
        if step.abs() < 1e-12 {
            break;
        }
    }
    if !e_anom.is_finite() {
        e_anom = m;
    }
    (e_anom, e_anom.sin(), e_anom.cos())
}

fn wrap_time(mut t: f64) -> f64 {
    let half = 302_400.0;
    while t > half {
        t -= 604_800.0;
    }
    while t < -half {
        t += 604_800.0;
    }
    t
}

