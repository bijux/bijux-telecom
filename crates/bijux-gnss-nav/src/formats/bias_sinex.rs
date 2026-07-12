#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::str::FromStr;

use bijux_gnss_core::api::{
    signal_registry, utc_to_gps, Constellation, GpsTime, LeapSeconds, SatId, SigId, SignalBand,
    SignalCode, UtcTime,
};
use time::{Date, Month, PrimitiveDateTime, Time};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BiasTimeSystem {
    Gps,
    Utc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct SignalPair {
    first: SigId,
    second: SigId,
}

#[derive(Debug, Clone)]
struct RawBiasWindow {
    start: GpsTime,
    end: GpsTime,
    observable_biases_m: BTreeMap<SigId, f64>,
    differential_biases_m: BTreeMap<SignalPair, f64>,
    iono_free_biases_m: BTreeMap<SignalPair, f64>,
}

impl RawBiasWindow {
    fn new(start: GpsTime, end: GpsTime) -> Self {
        Self {
            start,
            end,
            observable_biases_m: BTreeMap::new(),
            differential_biases_m: BTreeMap::new(),
            iono_free_biases_m: BTreeMap::new(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct BiasSinexWindow {
    pub start: GpsTime,
    pub end: GpsTime,
    pub observable_biases_m: BTreeMap<SigId, f64>,
    pub differential_biases_m: BTreeMap<(SigId, SigId), f64>,
    pub iono_free_biases_m: BTreeMap<(SigId, SigId), f64>,
}

#[derive(Debug, Clone)]
pub struct BiasSinexProvider {
    windows: BTreeMap<SatId, Vec<BiasSinexWindow>>,
}

impl BiasSinexProvider {
    fn parse_internal(input: &str) -> Result<Self, String> {
        let mut time_system = BiasTimeSystem::Gps;
        let mut in_description = false;
        let mut in_solution = false;
        let mut raw_windows: BTreeMap<(SatId, i64, i64), RawBiasWindow> = BTreeMap::new();

        for line in input.lines() {
            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with('*') || trimmed.starts_with("%=BIA") {
                continue;
            }
            if trimmed == "+BIAS/DESCRIPTION" {
                in_description = true;
                continue;
            }
            if trimmed == "-BIAS/DESCRIPTION" {
                in_description = false;
                continue;
            }
            if trimmed == "+BIAS/SOLUTION" {
                in_solution = true;
                continue;
            }
            if trimmed == "-BIAS/SOLUTION" || trimmed == "%=ENDBIA" {
                in_solution = false;
                continue;
            }

            if in_description {
                let fields = trimmed.split_whitespace().collect::<Vec<_>>();
                if fields.first() == Some(&"TIME_SYSTEM") {
                    time_system = match fields.get(1).copied().unwrap_or("G") {
                        "G" => BiasTimeSystem::Gps,
                        "UTC" | "U" => BiasTimeSystem::Utc,
                        other => {
                            return Err(format!("unsupported Bias-SINEX TIME_SYSTEM '{other}'"));
                        }
                    };
                }
                continue;
            }

            if !in_solution {
                continue;
            }

            let fields = trimmed.split_whitespace().collect::<Vec<_>>();
            let Some(kind) = fields.first().copied() else {
                continue;
            };
            if !matches!(kind, "OSB" | "DSB" | "ISB") {
                continue;
            }

            let Some(sat) = fields.get(2).and_then(|value| parse_sat_id(value).ok()) else {
                continue;
            };
            let (obs_1, obs_2, start_field, end_field, unit_field, value_field) = match kind {
                "OSB" if fields.len() >= 8 => {
                    (fields[3], None, fields[4], fields[5], fields[6], fields[7])
                }
                "DSB" | "ISB" if fields.len() >= 9 => {
                    (fields[3], Some(fields[4]), fields[5], fields[6], fields[7], fields[8])
                }
                _ => continue,
            };

            if unit_field != "ns" {
                return Err(format!(
                    "unsupported Bias-SINEX unit '{unit_field}' for {kind} {:?}",
                    sat
                ));
            }

            let signal_1 = match parse_observable_signal(sat, obs_1) {
                Some(signal) => signal,
                None => continue,
            };
            let signal_2 = match obs_2 {
                Some(code) => match parse_observable_signal(sat, code) {
                    Some(signal) => Some(signal),
                    None => continue,
                },
                None => None,
            };

            let start = parse_bias_time(start_field, time_system)?;
            let end = parse_bias_time(end_field, time_system)?;
            let value_ns = value_field
                .parse::<f64>()
                .map_err(|err| format!("invalid Bias-SINEX value '{value_field}': {err}"))?;
            let value_m = value_ns * 1.0e-9 * SPEED_OF_LIGHT_MPS;
            let key = (sat, start.to_seconds().round() as i64, end.to_seconds().round() as i64);
            let window = raw_windows.entry(key).or_insert_with(|| RawBiasWindow::new(start, end));

            match (kind, signal_2) {
                ("OSB", None) => {
                    window.observable_biases_m.insert(signal_1, value_m);
                }
                ("DSB", Some(signal_2)) => {
                    window
                        .differential_biases_m
                        .insert(SignalPair { first: signal_1, second: signal_2 }, value_m);
                }
                ("ISB", Some(signal_2)) => {
                    window
                        .iono_free_biases_m
                        .insert(SignalPair { first: signal_1, second: signal_2 }, value_m);
                }
                _ => {}
            }
        }

        let mut windows: BTreeMap<SatId, Vec<BiasSinexWindow>> = BTreeMap::new();
        for ((sat, _, _), raw) in raw_windows {
            windows.entry(sat).or_default().push(finalize_window(raw)?);
        }
        for entries in windows.values_mut() {
            entries.sort_by(|left, right| {
                left.start.to_seconds().total_cmp(&right.start.to_seconds())
            });
        }

        Ok(Self { windows })
    }

    pub fn satellites(&self) -> Vec<SatId> {
        self.windows.keys().copied().collect()
    }

    pub fn coverage(&self, sat: SatId) -> Option<(GpsTime, GpsTime)> {
        let windows = self.windows.get(&sat)?;
        let start = windows.first()?.start;
        let end = windows.last()?.end;
        Some((start, end))
    }

    pub fn code_bias_m_at(&self, sig: SigId, time: Option<GpsTime>) -> Option<f64> {
        let window = self.window_for_signal(sig, time)?;
        window.observable_biases_m.get(&sig).copied()
    }

    pub fn differential_code_bias_m_at(
        &self,
        first: SigId,
        second: SigId,
        time: Option<GpsTime>,
    ) -> Option<f64> {
        let window = self.window_for_signal(first, time)?;
        if let Some(value) = window.differential_biases_m.get(&(first, second)) {
            return Some(*value);
        }
        if let Some(value) = window.differential_biases_m.get(&(second, first)) {
            return Some(-value);
        }
        let first_bias = window.observable_biases_m.get(&first)?;
        let second_bias = window.observable_biases_m.get(&second)?;
        Some(first_bias - second_bias)
    }

    pub fn iono_free_code_bias_m_at(
        &self,
        first: SigId,
        second: SigId,
        f1_hz: f64,
        f2_hz: f64,
        time: Option<GpsTime>,
    ) -> Option<f64> {
        let window = self.window_for_signal(first, time)?;
        if let Some(value) = window.iono_free_biases_m.get(&(first, second)) {
            return Some(*value);
        }
        let (weight_1, weight_2) = iono_free_weights(f1_hz, f2_hz)?;
        let first_bias = window.observable_biases_m.get(&first)?;
        let second_bias = window.observable_biases_m.get(&second)?;
        Some(weight_1 * first_bias + weight_2 * second_bias)
    }

    fn window_for_signal(&self, sig: SigId, time: Option<GpsTime>) -> Option<&BiasSinexWindow> {
        let windows = self.windows.get(&sig.sat)?;
        match time {
            Some(time) => {
                let time_s = time.to_seconds();
                windows.iter().find(|window| {
                    time_s >= window.start.to_seconds() && time_s <= window.end.to_seconds()
                })
            }
            None if windows.len() == 1 => windows.first(),
            None => None,
        }
    }
}

impl FromStr for BiasSinexProvider {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        Self::parse_internal(s)
    }
}

fn finalize_window(raw: RawBiasWindow) -> Result<BiasSinexWindow, String> {
    let mut observable_biases_m = raw.observable_biases_m.clone();
    let differential_biases_m = raw
        .differential_biases_m
        .iter()
        .map(|(pair, value)| ((pair.first, pair.second), *value))
        .collect::<BTreeMap<_, _>>();
    let iono_free_biases_m = raw
        .iono_free_biases_m
        .iter()
        .map(|(pair, value)| ((pair.first, pair.second), *value))
        .collect::<BTreeMap<_, _>>();

    let mut changed = true;
    while changed {
        changed = false;

        for (pair, value) in &raw.iono_free_biases_m {
            let (weight_1, weight_2) = signal_pair_iono_free_weights(pair.first, pair.second)
                .ok_or_else(|| {
                    format!("unsupported signal pair {:?} -> {:?}", pair.first, pair.second)
                })?;
            let first_bias = observable_biases_m.get(&pair.first).copied();
            let second_bias = observable_biases_m.get(&pair.second).copied();
            let differential = raw
                .differential_bias_for(pair.first, pair.second)
                .or(raw.differential_bias_for(pair.second, pair.first));

            match (first_bias, second_bias, differential) {
                (None, None, Some(dsb_m)) => {
                    observable_biases_m.insert(pair.first, *value + weight_2 * dsb_m);
                    observable_biases_m.insert(pair.second, *value - weight_1 * dsb_m);
                    changed = true;
                }
                (Some(first_bias), None, _) if weight_2.abs() > f64::EPSILON => {
                    observable_biases_m
                        .insert(pair.second, (*value - weight_1 * first_bias) / weight_2);
                    changed = true;
                }
                (None, Some(second_bias), _) if weight_1.abs() > f64::EPSILON => {
                    observable_biases_m
                        .insert(pair.first, (*value - weight_2 * second_bias) / weight_1);
                    changed = true;
                }
                _ => {}
            }
        }

        for (pair, value) in &raw.differential_biases_m {
            let first_bias = observable_biases_m.get(&pair.first).copied();
            let second_bias = observable_biases_m.get(&pair.second).copied();
            match (first_bias, second_bias) {
                (Some(first_bias), None) => {
                    observable_biases_m.insert(pair.second, first_bias - value);
                    changed = true;
                }
                (None, Some(second_bias)) => {
                    observable_biases_m.insert(pair.first, second_bias + value);
                    changed = true;
                }
                _ => {}
            }
        }
    }

    Ok(BiasSinexWindow {
        start: raw.start,
        end: raw.end,
        observable_biases_m,
        differential_biases_m,
        iono_free_biases_m,
    })
}

impl RawBiasWindow {
    fn differential_bias_for(&self, first: SigId, second: SigId) -> Option<f64> {
        if let Some(value) = self.differential_biases_m.get(&SignalPair { first, second }) {
            return Some(*value);
        }
        self.differential_biases_m
            .get(&SignalPair { first: second, second: first })
            .map(|value| -*value)
    }
}

fn parse_observable_signal(sat: SatId, observable: &str) -> Option<SigId> {
    let (band, code) = observable_band_code(sat.constellation, observable)?;
    Some(SigId { sat, band, code })
}

fn observable_band_code(
    constellation: Constellation,
    observable: &str,
) -> Option<(SignalBand, SignalCode)> {
    let observable = observable.trim();
    if !observable.starts_with('C') {
        return None;
    }

    match constellation {
        Constellation::Gps => match observable {
            "C1C" | "C1" | "C1W" | "C1P" | "P1" => Some((SignalBand::L1, SignalCode::Ca)),
            "C2L" | "C2M" | "C2X" | "C2S" => Some((SignalBand::L2, SignalCode::L2C)),
            "C2W" | "P2" | "C2P" | "C2" | "C2C" => Some((SignalBand::L2, SignalCode::Py)),
            "C5Q" | "C5X" | "C5I" | "C5" => Some((SignalBand::L5, SignalCode::Unknown)),
            _ => None,
        },
        Constellation::Galileo => match observable {
            "C1C" | "C1X" | "C1B" => Some((SignalBand::E1, SignalCode::E1B)),
            "C5Q" | "C5X" | "C5I" => Some((SignalBand::E5, SignalCode::E5a)),
            _ => None,
        },
        Constellation::Beidou => match observable {
            "C2I" | "C2X" | "C2Q" | "C1I" | "C1X" | "C1Q" => {
                Some((SignalBand::B1, SignalCode::B1I))
            }
            "C7I" | "C7X" | "C7Q" => Some((SignalBand::B2, SignalCode::B2I)),
            _ => None,
        },
        Constellation::Glonass => match observable {
            "C1C" | "C1P" => Some((SignalBand::L1, SignalCode::Unknown)),
            _ => None,
        },
        Constellation::Unknown => None,
    }
}

fn signal_pair_iono_free_weights(first: SigId, second: SigId) -> Option<(f64, f64)> {
    let f1_hz =
        signal_registry(first.sat.constellation, first.band, first.code)?.spec.carrier_hz.value();
    let f2_hz = signal_registry(second.sat.constellation, second.band, second.code)?
        .spec
        .carrier_hz
        .value();
    iono_free_weights(f1_hz, f2_hz)
}

fn iono_free_weights(f1_hz: f64, f2_hz: f64) -> Option<(f64, f64)> {
    if !f1_hz.is_finite() || !f2_hz.is_finite() || f1_hz <= 0.0 || f2_hz <= 0.0 {
        return None;
    }
    let f1_2 = f1_hz * f1_hz;
    let f2_2 = f2_hz * f2_hz;
    let denom = f1_2 - f2_2;
    if !denom.is_finite() || denom.abs() <= f64::EPSILON {
        return None;
    }
    Some((f1_2 / denom, -f2_2 / denom))
}

fn parse_sat_id(value: &str) -> Result<SatId, String> {
    let value = value.trim();
    if value.len() < 2 {
        return Err("invalid satellite id".to_string());
    }
    let constellation = match value.chars().next().unwrap_or('G') {
        'G' => Constellation::Gps,
        'R' => Constellation::Glonass,
        'E' => Constellation::Galileo,
        'C' => Constellation::Beidou,
        other => return Err(format!("unsupported constellation '{other}'")),
    };
    let prn = value[1..].parse::<u8>().map_err(|err| format!("invalid PRN '{value}': {err}"))?;
    Ok(SatId { constellation, prn })
}

fn parse_bias_time(value: &str, time_system: BiasTimeSystem) -> Result<GpsTime, String> {
    let fields = value.split(':').collect::<Vec<_>>();
    if fields.len() != 3 {
        return Err(format!("invalid Bias-SINEX time '{value}'"));
    }
    let year = fields[0].parse::<i32>().map_err(|err| format!("invalid year '{value}': {err}"))?;
    let day_of_year =
        fields[1].parse::<u16>().map_err(|err| format!("invalid day-of-year '{value}': {err}"))?;
    let seconds_of_day = fields[2]
        .parse::<f64>()
        .map_err(|err| format!("invalid seconds-of-day '{value}': {err}"))?;
    match time_system {
        BiasTimeSystem::Gps => gps_time_from_gps_calendar(year, day_of_year, seconds_of_day),
        BiasTimeSystem::Utc => gps_time_from_utc_calendar(year, day_of_year, seconds_of_day),
    }
}

fn gps_time_from_gps_calendar(
    year: i32,
    day_of_year: u16,
    seconds_of_day: f64,
) -> Result<GpsTime, String> {
    let date = Date::from_ordinal_date(year, day_of_year)
        .map_err(|err| format!("invalid Bias-SINEX GPS date {year}:{day_of_year:03}: {err}"))?;
    let gps_epoch = Date::from_calendar_date(1980, Month::January, 6)
        .map_err(|err| format!("invalid GPS epoch date: {err}"))?;
    let day_span = (date - gps_epoch).whole_days();
    Ok(GpsTime::from_seconds(day_span as f64 * 86_400.0 + seconds_of_day))
}

fn gps_time_from_utc_calendar(
    year: i32,
    day_of_year: u16,
    seconds_of_day: f64,
) -> Result<GpsTime, String> {
    let date = Date::from_ordinal_date(year, day_of_year)
        .map_err(|err| format!("invalid Bias-SINEX UTC date {year}:{day_of_year:03}: {err}"))?;
    let whole_seconds = seconds_of_day.floor();
    let nanos = ((seconds_of_day - whole_seconds) * 1_000_000_000.0).round() as u32;
    let time = Time::from_hms_nano(
        (whole_seconds / 3600.0) as u8,
        ((whole_seconds % 3600.0) / 60.0) as u8,
        (whole_seconds % 60.0) as u8,
        nanos,
    )
    .map_err(|err| format!("invalid Bias-SINEX UTC time {seconds_of_day}: {err}"))?;
    let utc = PrimitiveDateTime::new(date, time).assume_utc();
    Ok(utc_to_gps(
        UtcTime { unix_s: utc.unix_timestamp_nanos() as f64 / 1_000_000_000.0 },
        &LeapSeconds::default_table(),
    ))
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{
        Constellation, SatId, SigId, SignalBand, SignalCode, GPS_L1_CA_CARRIER_HZ,
        GPS_L2_PY_CARRIER_HZ,
    };

    use super::{parse_bias_time, BiasSinexProvider, BiasTimeSystem};

    fn gps_signal(prn: u8, band: SignalBand, code: SignalCode) -> SigId {
        SigId { sat: SatId { constellation: Constellation::Gps, prn }, band, code }
    }

    #[test]
    fn parses_absolute_observable_specific_biases() {
        let data = "\
%=BIA 1.00 COD 2016:327:06748 IGS 2016:323:00000 2016:324:00000 A 00000004
+BIAS/DESCRIPTION
TIME_SYSTEM G
-BIAS/DESCRIPTION
+BIAS/SOLUTION
OSB G063 G01 C1C 2016:323:00000 2016:324:00000 ns 10.2669 0.0257
OSB G063 G01 C1W 2016:323:00000 2016:324:00000 ns 11.7118 0.0174
OSB G063 G01 C2W 2016:323:00000 2016:324:00000 ns 19.2886 0.0281
-BIAS/SOLUTION
%=ENDBIA
";
        let provider = data.parse::<BiasSinexProvider>().expect("bias sinex parse");
        let l1 = gps_signal(1, SignalBand::L1, SignalCode::Ca);
        let l2 = gps_signal(1, SignalBand::L2, SignalCode::Py);

        assert!(provider.code_bias_m_at(l1, None).is_some());
        assert!(provider.code_bias_m_at(l2, None).is_some());
        assert!(provider.differential_code_bias_m_at(l1, l2, None).is_some());
    }

    #[test]
    fn derives_observable_specific_biases_from_relative_pairs() {
        let data = "\
%=BIA 1.00 COD 2016:330:30148 IGS 2016:271:00000 2016:272:00000 R 00000005
+BIAS/DESCRIPTION
TIME_SYSTEM G
-BIAS/DESCRIPTION
+BIAS/SOLUTION
ISB G063 G01 C1W C2W 2016:271:00000 2016:272:00000 ns 0 0
DSB G063 G01 C1W C1C 2016:271:00000 2016:272:00000 ns 1.4295 0.0183
DSB G063 G01 C1W C2W 2016:271:00000 2016:272:00000 ns -7.4709 0.0183
DSB G063 G01 C2W C2C 2016:271:00000 2016:272:00000 ns 8.2266 0.0272
-BIAS/SOLUTION
%=ENDBIA
";
        let provider = data.parse::<BiasSinexProvider>().expect("bias sinex parse");
        let c1w = gps_signal(1, SignalBand::L1, SignalCode::Ca);
        let c2w = gps_signal(1, SignalBand::L2, SignalCode::Py);

        let l1_bias_m = provider.code_bias_m_at(c1w, None).expect("L1 bias");
        let l2_bias_m = provider.code_bias_m_at(c2w, None).expect("L2 bias");
        let dsb_m =
            provider.differential_code_bias_m_at(c1w, c2w, None).expect("differential bias");
        let iono_free_bias_m = provider
            .iono_free_code_bias_m_at(
                c1w,
                c2w,
                GPS_L1_CA_CARRIER_HZ.value(),
                GPS_L2_PY_CARRIER_HZ.value(),
                None,
            )
            .expect("iono-free bias");

        assert!((l1_bias_m - l2_bias_m - dsb_m).abs() < 1.0e-9);
        assert!(iono_free_bias_m.abs() < 1.0e-9);
    }

    #[test]
    fn selects_bias_window_by_gps_time() {
        let data = "\
%=BIA 1.00 COD 2016:330:30148 IGS 2016:271:00000 2016:273:00000 A 00000002
+BIAS/DESCRIPTION
TIME_SYSTEM G
-BIAS/DESCRIPTION
+BIAS/SOLUTION
OSB G063 G01 C1C 2016:271:00000 2016:272:00000 ns 10.0000 0.0100
OSB G063 G01 C1C 2016:272:00000 2016:273:00000 ns 20.0000 0.0100
-BIAS/SOLUTION
%=ENDBIA
";
        let provider = data.parse::<BiasSinexProvider>().expect("bias sinex parse");
        let signal = gps_signal(1, SignalBand::L1, SignalCode::Ca);
        let first_day = parse_bias_time("2016:271:43200", BiasTimeSystem::Gps).expect("first day");
        let second_day =
            parse_bias_time("2016:272:43200", BiasTimeSystem::Gps).expect("second day");

        assert!(provider.code_bias_m_at(signal, None).is_none());
        assert!(provider.code_bias_m_at(signal, Some(first_day)).expect("first day") > 2.9);
        assert!(provider.code_bias_m_at(signal, Some(second_day)).expect("second day") > 5.9);
    }
}
