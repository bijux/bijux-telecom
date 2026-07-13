use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    signal_spec_beidou_b1i, signal_spec_beidou_b2i, signal_spec_galileo_e1b,
    signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l2c,
    signal_spec_gps_l5, utc_to_gps, Constellation, Cycles, GpsTime, Hertz, LeapSeconds, LockFlags,
    Meters, ObsEpoch, ObsMetadata, ObsSatellite, ObservationStatus, ParseError, ReceiverRole,
    SatId, Seconds, SigId, SignalBand, SignalCode, SignalSpec,
};
use time::{Date, Month, PrimitiveDateTime, Time};

const OBSERVATION_FIELD_WIDTH: usize = 16;
const OBSERVATIONS_PER_LINE: usize = 5;
const GPS_UNIX_EPOCH_OFFSET_S: f64 = 315_964_800.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RinexObservationTimeSystem {
    Gps,
    Utc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RinexCodeBiasState {
    Unknown,
    Applied,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RinexCodeBiasStatus {
    pub state: RinexCodeBiasState,
    pub source: Option<String>,
}

impl RinexCodeBiasStatus {
    fn unknown() -> Self {
        Self { state: RinexCodeBiasState::Unknown, source: None }
    }

    fn applied(source: String) -> Self {
        Self { state: RinexCodeBiasState::Applied, source: Some(source) }
    }
}

#[derive(Debug, Clone)]
struct RinexObservationHeader {
    version: f64,
    marker_name: Option<String>,
    approx_position_ecef_m: Option<(f64, f64, f64)>,
    interval_s: Option<f64>,
    time_system: RinexObservationTimeSystem,
    obs_types_v2: Option<Vec<String>>,
    obs_types_by_system: BTreeMap<char, Vec<String>>,
    dcbs_applied_by_system: BTreeMap<char, String>,
}

impl RinexObservationHeader {
    fn gps_observation_types(&self) -> Option<&[String]> {
        self.observation_types('G')
    }

    fn galileo_observation_types(&self) -> Option<&[String]> {
        self.observation_types('E')
    }

    fn beidou_observation_types(&self) -> Option<&[String]> {
        self.observation_types('C')
    }

    fn observation_types(&self, system: char) -> Option<&[String]> {
        if self.version >= 3.0 {
            self.obs_types_by_system.get(&system).map(Vec::as_slice)
        } else {
            self.obs_types_v2.as_deref()
        }
    }

    fn code_bias_status(&self, system: char) -> RinexCodeBiasStatus {
        self.dcbs_applied_by_system
            .get(&system)
            .cloned()
            .map(RinexCodeBiasStatus::applied)
            .unwrap_or_else(RinexCodeBiasStatus::unknown)
    }
}

/// GPS observations imported from a RINEX observation file.
#[derive(Debug, Clone)]
pub struct RinexGpsObservationDataset {
    /// RINEX format version read from the header.
    pub version: f64,
    /// Marker name if the file declares one.
    pub marker_name: Option<String>,
    /// Approximate station coordinates from `APPROX POSITION XYZ`.
    pub approx_position_ecef_m: Option<(f64, f64, f64)>,
    /// Observation interval in seconds if declared in the header.
    pub interval_s: Option<f64>,
    /// Differential code-bias provenance declared by the RINEX header for this system.
    pub code_bias_status: RinexCodeBiasStatus,
    /// Per-band GPS observation types imported into `epochs`.
    pub observation_channels: Vec<RinexGpsObservationChannel>,
    /// GPS observation epochs converted into the workspace observation model.
    pub epochs: Vec<ObsEpoch>,
}

#[derive(Debug, Clone)]
pub struct RinexGpsObservationChannel {
    /// Signal band represented by this imported observation channel.
    pub band: SignalBand,
    /// Signal code represented by this imported observation channel.
    pub code: SignalCode,
    /// Pseudorange observation type used to populate this channel.
    pub pseudorange_observation_type: String,
    /// Carrier-phase observation type used when present.
    pub carrier_phase_observation_type: Option<String>,
    /// Signal-strength observation type used when present.
    pub signal_strength_observation_type: Option<String>,
}

#[derive(Debug, Clone)]
pub struct RinexGalileoObservationDataset {
    /// RINEX format version read from the header.
    pub version: f64,
    /// Marker name if the file declares one.
    pub marker_name: Option<String>,
    /// Approximate station coordinates from `APPROX POSITION XYZ`.
    pub approx_position_ecef_m: Option<(f64, f64, f64)>,
    /// Observation interval in seconds if declared in the header.
    pub interval_s: Option<f64>,
    /// Differential code-bias provenance declared by the RINEX header for this system.
    pub code_bias_status: RinexCodeBiasStatus,
    /// Per-band Galileo observation types imported into `epochs`.
    pub observation_channels: Vec<RinexGalileoObservationChannel>,
    /// Galileo observation epochs converted into the workspace observation model.
    pub epochs: Vec<ObsEpoch>,
}

#[derive(Debug, Clone)]
pub struct RinexGalileoObservationChannel {
    /// Signal band represented by this imported observation channel.
    pub band: SignalBand,
    /// Signal code represented by this imported observation channel.
    pub code: SignalCode,
    /// Pseudorange observation type used to populate this channel.
    pub pseudorange_observation_type: String,
    /// Carrier-phase observation type used when present.
    pub carrier_phase_observation_type: Option<String>,
    /// Signal-strength observation type used when present.
    pub signal_strength_observation_type: Option<String>,
}

#[derive(Debug, Clone)]
pub struct RinexBeidouObservationDataset {
    /// RINEX format version read from the header.
    pub version: f64,
    /// Marker name if the file declares one.
    pub marker_name: Option<String>,
    /// Approximate station coordinates from `APPROX POSITION XYZ`.
    pub approx_position_ecef_m: Option<(f64, f64, f64)>,
    /// Observation interval in seconds if declared in the header.
    pub interval_s: Option<f64>,
    /// Differential code-bias provenance declared by the RINEX header for this system.
    pub code_bias_status: RinexCodeBiasStatus,
    /// Per-band BeiDou observation types imported into `epochs`.
    pub observation_channels: Vec<RinexBeidouObservationChannel>,
    /// BeiDou observation epochs converted into the workspace observation model.
    pub epochs: Vec<ObsEpoch>,
}

#[derive(Debug, Clone)]
pub struct RinexBeidouObservationChannel {
    /// Signal band represented by this imported observation channel.
    pub band: SignalBand,
    /// Signal code represented by this imported observation channel.
    pub code: SignalCode,
    /// Pseudorange observation type used to populate this channel.
    pub pseudorange_observation_type: String,
    /// Carrier-phase observation type used when present.
    pub carrier_phase_observation_type: Option<String>,
    /// Signal-strength observation type used when present.
    pub signal_strength_observation_type: Option<String>,
}

#[derive(Debug, Clone)]
struct ImportedObservationChannel {
    band: SignalBand,
    code: SignalCode,
    signal: SignalSpec,
    pseudorange_index: usize,
    pseudorange_type: String,
    carrier_phase_index: Option<usize>,
    carrier_phase_type: Option<String>,
    signal_strength_index: Option<usize>,
    signal_strength_type: Option<String>,
}

#[derive(Debug, Clone, Copy)]
struct RinexSatelliteToken {
    system: char,
    satellite: SatId,
}

#[derive(Debug, Clone)]
enum Rinex2Record {
    Event {
        special_record_count: usize,
    },
    ObservationEpoch {
        receive_gps_time: GpsTime,
        discontinuity: bool,
        satellites: Vec<SatId>,
        header_line_count: usize,
    },
}

fn parse_rinex_observation_header(
    data: &str,
) -> Result<(RinexObservationHeader, usize), ParseError> {
    let lines = data.lines().collect::<Vec<_>>();
    let mut header = RinexObservationHeader {
        version: 0.0,
        marker_name: None,
        approx_position_ecef_m: None,
        interval_s: None,
        time_system: RinexObservationTimeSystem::Gps,
        obs_types_v2: None,
        obs_types_by_system: BTreeMap::new(),
        dcbs_applied_by_system: BTreeMap::new(),
    };
    let mut saw_version = false;
    let mut saw_observation_data = false;
    let mut line_index = 0usize;

    while line_index < lines.len() {
        let line = lines[line_index];
        let label = rinex_header_label(line);

        if !saw_version && label == "RINEX VERSION / TYPE" {
            header.version = parse_rinex_version(line)?;
            saw_version = true;
            saw_observation_data = line.contains("OBSERVATION DATA");
            line_index += 1;
            continue;
        }

        match label {
            "MARKER NAME" => {
                let marker_name = line.get(..60).unwrap_or_default().trim();
                if !marker_name.is_empty() {
                    header.marker_name = Some(marker_name.to_string());
                }
                line_index += 1;
            }
            "APPROX POSITION XYZ" => {
                header.approx_position_ecef_m = Some(parse_three_floats(
                    line.get(..60).unwrap_or_default(),
                    "RINEX OBS APPROX POSITION XYZ",
                )?);
                line_index += 1;
            }
            "INTERVAL" => {
                let interval =
                    line.get(..10).unwrap_or_default().trim().parse::<f64>().map_err(|err| {
                        ParseError {
                            message: format!("invalid RINEX OBS interval '{}': {err}", line.trim()),
                        }
                    })?;
                header.interval_s = Some(interval);
                line_index += 1;
            }
            "TIME OF FIRST OBS" => {
                if let Some(time_system) = parse_rinex_time_system(line) {
                    header.time_system = time_system;
                }
                line_index += 1;
            }
            "# / TYPES OF OBSERV" => {
                let (obs_types, consumed_lines) =
                    parse_rinex_2_observation_types(&lines[line_index..])?;
                header.obs_types_v2 = Some(obs_types);
                line_index += consumed_lines;
            }
            "SYS / # / OBS TYPES" => {
                let (system, obs_types, consumed_lines) =
                    parse_rinex_3_observation_types(&lines[line_index..])?;
                header.obs_types_by_system.insert(system, obs_types);
                line_index += consumed_lines;
            }
            "SYS / DCBS APPLIED" => {
                if let Some((system, source)) = parse_rinex_code_bias_status(line) {
                    header.dcbs_applied_by_system.insert(system, source);
                }
                line_index += 1;
            }
            "END OF HEADER" => {
                line_index += 1;
                break;
            }
            _ => {
                line_index += 1;
            }
        }
    }

    if !saw_version || !saw_observation_data || line_index == 0 {
        return Err(ParseError { message: "invalid or incomplete RINEX OBS header".to_string() });
    }

    Ok((header, line_index))
}

/// Parse GPS observations from a RINEX observation file.
///
/// The parser preserves only GPS observation epochs and maps them into `ObsEpoch`
/// with pseudorange and carrier-phase measurements ready for downstream positioning.
pub fn parse_rinex_gps_observation_dataset(
    data: &str,
) -> Result<RinexGpsObservationDataset, ParseError> {
    let lines = data.lines().collect::<Vec<_>>();
    let (header, header_line_count) = parse_rinex_observation_header(data)?;
    let gps_observation_types = header.gps_observation_types().ok_or_else(|| ParseError {
        message: "RINEX OBS file does not declare GPS observation types".to_string(),
    })?;
    let channels = resolve_gps_observation_channels(gps_observation_types)?;
    let code_bias_status = header.code_bias_status('G');
    let epochs = if header.version >= 3.0 {
        parse_rinex_3_constellation_epochs(
            &lines[header_line_count..],
            &header,
            'G',
            Constellation::Gps,
            &channels,
        )?
    } else {
        parse_rinex_2_constellation_epochs(
            &lines[header_line_count..],
            gps_observation_types,
            header.time_system,
            Constellation::Gps,
            &channels,
        )?
    };

    Ok(RinexGpsObservationDataset {
        version: header.version,
        marker_name: header.marker_name,
        approx_position_ecef_m: header.approx_position_ecef_m,
        interval_s: header.interval_s,
        code_bias_status,
        observation_channels: channels
            .iter()
            .map(|channel| RinexGpsObservationChannel {
                band: channel.band,
                code: channel.code,
                pseudorange_observation_type: channel.pseudorange_type.clone(),
                carrier_phase_observation_type: channel.carrier_phase_type.clone(),
                signal_strength_observation_type: channel.signal_strength_type.clone(),
            })
            .collect(),
        epochs,
    })
}

/// Parse Galileo observations from a RINEX observation file.
///
/// The parser preserves only Galileo observation epochs and maps them into `ObsEpoch`
/// with pseudorange and carrier-phase measurements ready for downstream validation.
pub fn parse_rinex_galileo_observation_dataset(
    data: &str,
) -> Result<RinexGalileoObservationDataset, ParseError> {
    let lines = data.lines().collect::<Vec<_>>();
    let (header, header_line_count) = parse_rinex_observation_header(data)?;
    let galileo_observation_types =
        header.galileo_observation_types().ok_or_else(|| ParseError {
            message: "RINEX OBS file does not declare Galileo observation types".to_string(),
        })?;
    let channels = resolve_galileo_observation_channels(galileo_observation_types)?;
    let code_bias_status = header.code_bias_status('E');
    let epochs = if header.version >= 3.0 {
        parse_rinex_3_constellation_epochs(
            &lines[header_line_count..],
            &header,
            'E',
            Constellation::Galileo,
            &channels,
        )?
    } else {
        parse_rinex_2_constellation_epochs(
            &lines[header_line_count..],
            galileo_observation_types,
            header.time_system,
            Constellation::Galileo,
            &channels,
        )?
    };

    Ok(RinexGalileoObservationDataset {
        version: header.version,
        marker_name: header.marker_name,
        approx_position_ecef_m: header.approx_position_ecef_m,
        interval_s: header.interval_s,
        code_bias_status,
        observation_channels: channels
            .iter()
            .map(|channel| RinexGalileoObservationChannel {
                band: channel.band,
                code: channel.code,
                pseudorange_observation_type: channel.pseudorange_type.clone(),
                carrier_phase_observation_type: channel.carrier_phase_type.clone(),
                signal_strength_observation_type: channel.signal_strength_type.clone(),
            })
            .collect(),
        epochs,
    })
}

/// Parse BeiDou observations from a RINEX observation file.
///
/// The parser preserves only BeiDou observation epochs and maps them into `ObsEpoch`
/// with B1I/B2I pseudorange and carrier-phase measurements ready for dual-frequency validation.
pub fn parse_rinex_beidou_observation_dataset(
    data: &str,
) -> Result<RinexBeidouObservationDataset, ParseError> {
    let lines = data.lines().collect::<Vec<_>>();
    let (header, header_line_count) = parse_rinex_observation_header(data)?;
    let beidou_observation_types = header.beidou_observation_types().ok_or_else(|| ParseError {
        message: "RINEX OBS file does not declare BeiDou observation types".to_string(),
    })?;
    let channels = resolve_beidou_observation_channels(beidou_observation_types)?;
    let code_bias_status = header.code_bias_status('C');
    let epochs = if header.version >= 3.0 {
        parse_rinex_3_constellation_epochs(
            &lines[header_line_count..],
            &header,
            'C',
            Constellation::Beidou,
            &channels,
        )?
    } else {
        parse_rinex_2_constellation_epochs(
            &lines[header_line_count..],
            beidou_observation_types,
            header.time_system,
            Constellation::Beidou,
            &channels,
        )?
    };

    Ok(RinexBeidouObservationDataset {
        version: header.version,
        marker_name: header.marker_name,
        approx_position_ecef_m: header.approx_position_ecef_m,
        interval_s: header.interval_s,
        code_bias_status,
        observation_channels: channels
            .iter()
            .map(|channel| RinexBeidouObservationChannel {
                band: channel.band,
                code: channel.code,
                pseudorange_observation_type: channel.pseudorange_type.clone(),
                carrier_phase_observation_type: channel.carrier_phase_type.clone(),
                signal_strength_observation_type: channel.signal_strength_type.clone(),
            })
            .collect(),
        epochs,
    })
}

fn rinex_header_label(line: &str) -> &str {
    line.get(60..).unwrap_or_default().trim()
}

fn parse_rinex_code_bias_status(line: &str) -> Option<(char, String)> {
    let system = line.chars().next()?.to_ascii_uppercase();
    if !matches!(system, 'G' | 'E' | 'C' | 'R') {
        return None;
    }
    let source = line.get(..60).unwrap_or_default().trim().to_string();
    if source.is_empty() {
        return None;
    }
    Some((system, source))
}

fn parse_rinex_2_constellation_epochs(
    lines: &[&str],
    observation_types: &[String],
    time_system: RinexObservationTimeSystem,
    constellation: Constellation,
    channels: &[ImportedObservationChannel],
) -> Result<Vec<ObsEpoch>, ParseError> {
    let mut epochs = Vec::new();
    let mut line_index = 0usize;
    let mut epoch_idx = 0u64;

    while line_index < lines.len() {
        let line = lines[line_index];
        if line.trim().is_empty() {
            line_index += 1;
            continue;
        }

        let record = parse_rinex_2_epoch_header(&lines[line_index..], time_system)?;
        match record {
            Rinex2Record::Event { special_record_count } => {
                line_index += 1 + special_record_count;
                continue;
            }
            Rinex2Record::ObservationEpoch {
                receive_gps_time,
                discontinuity,
                satellites,
                header_line_count,
            } => {
                line_index += header_line_count;
                let observation_line_count = observation_record_line_count(observation_types.len());
                let mut imported_satellites = Vec::new();

                for satellite in satellites {
                    let record_lines = lines
                        .get(line_index..line_index + observation_line_count)
                        .ok_or_else(|| ParseError {
                        message: format!(
                            "truncated RINEX 2 OBS record for satellite {:?}-{:02}",
                            satellite.constellation, satellite.prn
                        ),
                    })?;
                    let observations =
                        parse_observation_record(record_lines, observation_types.len(), 0)?;
                    line_index += observation_line_count;
                    if satellite.constellation != constellation {
                        continue;
                    }
                    imported_satellites.extend(build_constellation_observations(
                        satellite,
                        &observations,
                        channels,
                    )?);
                }

                if !imported_satellites.is_empty() {
                    epochs.push(build_obs_epoch(
                        epoch_idx,
                        receive_gps_time,
                        discontinuity,
                        imported_satellites,
                    ));
                }
                epoch_idx += 1;
            }
        }
    }

    Ok(epochs)
}

fn parse_rinex_3_constellation_epochs(
    lines: &[&str],
    header: &RinexObservationHeader,
    system: char,
    constellation: Constellation,
    channels: &[ImportedObservationChannel],
) -> Result<Vec<ObsEpoch>, ParseError> {
    let mut epochs = Vec::new();
    let mut line_index = 0usize;
    let mut epoch_idx = 0u64;

    while line_index < lines.len() {
        let line = lines[line_index];
        if line.trim().is_empty() {
            line_index += 1;
            continue;
        }
        if !line.starts_with('>') {
            return Err(ParseError {
                message: format!("expected RINEX 3 epoch header line, found '{}'", line.trim()),
            });
        }

        let (receive_gps_time, discontinuity, satellite_count) =
            parse_rinex_3_epoch_header(line, header.time_system)?;
        line_index += 1;
        let mut imported_satellites = Vec::new();

        for _ in 0..satellite_count {
            let record_start = lines.get(line_index).ok_or_else(|| ParseError {
                message: "truncated RINEX 3 observation record".to_string(),
            })?;
            let satellite_token = parse_satellite_token(record_start.get(..3).unwrap_or_default())?;
            let observation_types = observation_types_for_system(header, satellite_token.system);
            if observation_types.is_empty() {
                return Err(ParseError {
                    message: format!(
                        "missing RINEX 3 observation types for {}{:02}",
                        satellite_token.system, satellite_token.satellite.prn
                    ),
                });
            }
            let record_line_count = rinex_3_observation_record_line_count(
                &lines[line_index..],
                observation_types.len(),
            );
            let record_lines =
                lines.get(line_index..line_index + record_line_count).ok_or_else(|| {
                    ParseError {
                        message: format!(
                            "truncated RINEX 3 OBS record for satellite {:?}-{:02}",
                            satellite_token.satellite.constellation, satellite_token.satellite.prn
                        ),
                    }
                })?;
            let observations = parse_observation_record(record_lines, observation_types.len(), 3)?;
            line_index += record_line_count;
            if satellite_token.system != system
                || satellite_token.satellite.constellation != constellation
            {
                continue;
            }
            imported_satellites.extend(build_constellation_observations(
                satellite_token.satellite,
                &observations,
                channels,
            )?);
        }

        if !imported_satellites.is_empty() {
            epochs.push(build_obs_epoch(
                epoch_idx,
                receive_gps_time,
                discontinuity,
                imported_satellites,
            ));
        }
        epoch_idx += 1;
    }

    Ok(epochs)
}

fn parse_rinex_2_epoch_header(
    lines: &[&str],
    time_system: RinexObservationTimeSystem,
) -> Result<Rinex2Record, ParseError> {
    let line = *lines
        .first()
        .ok_or_else(|| ParseError { message: "missing RINEX 2 epoch header".to_string() })?;
    if line.get(..26).unwrap_or_default().trim().is_empty() {
        let flag = parse_u8_field(line, 28, 29, "RINEX 2 OBS epoch flag")?;
        let special_record_count =
            parse_usize_field(line, 29, 32, "RINEX 2 OBS special record count")?;
        if flag <= 1 {
            return Err(ParseError {
                message: format!("invalid RINEX 2 data epoch without timestamp: '{}'", line.trim()),
            });
        }
        return Ok(Rinex2Record::Event { special_record_count });
    }
    let year = parse_i32_field(line, 0, 3, "RINEX 2 OBS epoch year")?;
    let month = parse_u8_field(line, 3, 6, "RINEX 2 OBS epoch month")?;
    let day = parse_u8_field(line, 6, 9, "RINEX 2 OBS epoch day")?;
    let hour = parse_u8_field(line, 9, 12, "RINEX 2 OBS epoch hour")?;
    let minute = parse_u8_field(line, 12, 15, "RINEX 2 OBS epoch minute")?;
    let second = parse_f64_field(line, 15, 26, "RINEX 2 OBS epoch second")?;
    let flag = parse_u8_field(line, 28, 29, "RINEX 2 OBS epoch flag")?;
    let satellite_count = parse_usize_field(line, 29, 32, "RINEX 2 OBS satellite count")?;
    if flag > 1 {
        return Ok(Rinex2Record::Event { special_record_count: satellite_count });
    }
    let receive_gps_time = rinex_epoch_to_gps_time(
        normalize_rinex_2_year(year),
        month,
        day,
        hour,
        minute,
        second,
        time_system,
    )?;
    let header_line_count = 1 + satellite_count.saturating_sub(1) / 12;
    let mut satellites = Vec::with_capacity(satellite_count);

    for header_line_offset in 0..header_line_count {
        let satellite_line = *lines.get(header_line_offset).ok_or_else(|| ParseError {
            message: format!(
                "truncated RINEX 2 epoch satellite list: expected {header_line_count} header lines"
            ),
        })?;
        for chunk in observation_tokens(satellite_line, 32, 3) {
            if satellites.len() == satellite_count {
                break;
            }
            satellites.push(parse_satellite_token(&chunk)?.satellite);
        }
    }

    if satellites.len() != satellite_count {
        return Err(ParseError {
            message: format!(
                "RINEX 2 epoch declared {satellite_count} satellites but listed {}",
                satellites.len()
            ),
        });
    }

    Ok(Rinex2Record::ObservationEpoch {
        receive_gps_time,
        discontinuity: flag == 1,
        satellites,
        header_line_count,
    })
}

fn parse_rinex_3_epoch_header(
    line: &str,
    time_system: RinexObservationTimeSystem,
) -> Result<(GpsTime, bool, usize), ParseError> {
    let fields = line.split_whitespace().collect::<Vec<_>>();
    if fields.len() < 9 || fields.first().copied() != Some(">") {
        return Err(ParseError {
            message: format!("invalid RINEX 3 epoch header '{}'", line.trim()),
        });
    }
    let year = fields[1].parse::<i32>().map_err(|err| ParseError {
        message: format!("invalid RINEX 3 epoch year '{}': {err}", fields[1]),
    })?;
    let month = fields[2].parse::<u8>().map_err(|err| ParseError {
        message: format!("invalid RINEX 3 epoch month '{}': {err}", fields[2]),
    })?;
    let day = fields[3].parse::<u8>().map_err(|err| ParseError {
        message: format!("invalid RINEX 3 epoch day '{}': {err}", fields[3]),
    })?;
    let hour = fields[4].parse::<u8>().map_err(|err| ParseError {
        message: format!("invalid RINEX 3 epoch hour '{}': {err}", fields[4]),
    })?;
    let minute = fields[5].parse::<u8>().map_err(|err| ParseError {
        message: format!("invalid RINEX 3 epoch minute '{}': {err}", fields[5]),
    })?;
    let second = fields[6].parse::<f64>().map_err(|err| ParseError {
        message: format!("invalid RINEX 3 epoch second '{}': {err}", fields[6]),
    })?;
    let flag = fields[7].parse::<u8>().map_err(|err| ParseError {
        message: format!("invalid RINEX 3 epoch flag '{}': {err}", fields[7]),
    })?;
    let satellite_count = fields[8].parse::<usize>().map_err(|err| ParseError {
        message: format!("invalid RINEX 3 epoch satellite count '{}': {err}", fields[8]),
    })?;
    if flag > 1 {
        return Err(ParseError { message: format!("unsupported RINEX 3 epoch flag {flag}") });
    }
    Ok((
        rinex_epoch_to_gps_time(year, month, day, hour, minute, second, time_system)?,
        flag == 1,
        satellite_count,
    ))
}

fn resolve_gps_observation_channels(
    observation_types: &[String],
) -> Result<Vec<ImportedObservationChannel>, ParseError> {
    let mut channels = Vec::new();

    for (
        band,
        code,
        signal,
        pseudorange_candidates,
        carrier_phase_candidates,
        signal_strength_candidates,
    ) in [
        (
            SignalBand::L1,
            SignalCode::Ca,
            signal_spec_gps_l1_ca(),
            vec![
                "C1C".to_string(),
                "C1".to_string(),
                "C1W".to_string(),
                "C1P".to_string(),
                "P1".to_string(),
            ],
            vec!["L1C".to_string(), "L1".to_string(), "L1W".to_string(), "L1P".to_string()],
            vec!["S1C".to_string(), "S1".to_string(), "S1W".to_string(), "S1P".to_string()],
        ),
        (
            SignalBand::L2,
            SignalCode::L2C,
            signal_spec_gps_l2c(),
            vec!["C2L".to_string(), "C2M".to_string(), "C2X".to_string()],
            vec!["L2L".to_string(), "L2M".to_string(), "L2X".to_string()],
            vec!["S2L".to_string(), "S2M".to_string(), "S2X".to_string()],
        ),
        (
            SignalBand::L2,
            SignalCode::Py,
            signal_spec_gps_l2_py(),
            vec!["C2W".to_string(), "P2".to_string(), "C2P".to_string(), "C2".to_string()],
            vec!["L2W".to_string(), "L2P".to_string(), "L2".to_string()],
            vec!["S2W".to_string(), "S2P".to_string(), "S2".to_string()],
        ),
        (
            SignalBand::L5,
            SignalCode::Unknown,
            signal_spec_gps_l5(),
            vec!["C5Q".to_string(), "C5X".to_string(), "C5I".to_string(), "C5".to_string()],
            vec!["L5Q".to_string(), "L5X".to_string(), "L5I".to_string(), "L5".to_string()],
            vec!["S5Q".to_string(), "S5X".to_string(), "S5I".to_string(), "S5".to_string()],
        ),
    ] {
        let Some(pseudorange_index) =
            find_observation_index(observation_types, &pseudorange_candidates)
        else {
            continue;
        };
        let pseudorange_type = observation_types[pseudorange_index].clone();
        channels.push(ImportedObservationChannel {
            band,
            code,
            signal,
            pseudorange_index,
            pseudorange_type,
            carrier_phase_index: find_observation_index(
                observation_types,
                &carrier_phase_candidates,
            ),
            carrier_phase_type: find_observation_type(observation_types, &carrier_phase_candidates),
            signal_strength_index: find_observation_index(
                observation_types,
                &signal_strength_candidates,
            ),
            signal_strength_type: find_observation_type(
                observation_types,
                &signal_strength_candidates,
            ),
        });
    }

    if channels.is_empty() {
        return Err(ParseError {
            message: format!(
                "RINEX OBS file does not provide a supported GPS observation channel; found {}",
                observation_types.join(", ")
            ),
        });
    }

    Ok(channels)
}

fn resolve_galileo_observation_channels(
    observation_types: &[String],
) -> Result<Vec<ImportedObservationChannel>, ParseError> {
    let mut channels = Vec::new();

    for (
        band,
        code,
        signal,
        pseudorange_candidates,
        carrier_phase_candidates,
        signal_strength_candidates,
    ) in [
        (
            SignalBand::E1,
            SignalCode::E1B,
            signal_spec_galileo_e1b(),
            vec!["C1C".to_string(), "C1X".to_string(), "C1B".to_string()],
            vec!["L1C".to_string(), "L1X".to_string(), "L1B".to_string()],
            vec!["S1C".to_string(), "S1X".to_string(), "S1B".to_string()],
        ),
        (
            SignalBand::E5,
            SignalCode::E5a,
            signal_spec_galileo_e5a(),
            vec!["C5Q".to_string(), "C5X".to_string(), "C5I".to_string()],
            vec!["L5Q".to_string(), "L5X".to_string(), "L5I".to_string()],
            vec!["S5Q".to_string(), "S5X".to_string(), "S5I".to_string()],
        ),
    ] {
        let Some(pseudorange_index) =
            find_observation_index(observation_types, &pseudorange_candidates)
        else {
            continue;
        };
        let pseudorange_type = observation_types[pseudorange_index].clone();
        channels.push(ImportedObservationChannel {
            band,
            code,
            signal,
            pseudorange_index,
            pseudorange_type,
            carrier_phase_index: find_observation_index(
                observation_types,
                &carrier_phase_candidates,
            ),
            carrier_phase_type: find_observation_type(observation_types, &carrier_phase_candidates),
            signal_strength_index: find_observation_index(
                observation_types,
                &signal_strength_candidates,
            ),
            signal_strength_type: find_observation_type(
                observation_types,
                &signal_strength_candidates,
            ),
        });
    }

    if channels.is_empty() {
        return Err(ParseError {
            message: format!(
                "RINEX OBS file does not provide a supported Galileo observation channel; found {}",
                observation_types.join(", ")
            ),
        });
    }

    Ok(channels)
}

fn resolve_beidou_observation_channels(
    observation_types: &[String],
) -> Result<Vec<ImportedObservationChannel>, ParseError> {
    let mut channels = Vec::new();

    for (
        band,
        code,
        signal,
        pseudorange_candidates,
        carrier_phase_candidates,
        signal_strength_candidates,
    ) in [
        (
            SignalBand::B1,
            SignalCode::B1I,
            signal_spec_beidou_b1i(),
            vec![
                "C2I".to_string(),
                "C2X".to_string(),
                "C2Q".to_string(),
                "C1I".to_string(),
                "C1X".to_string(),
                "C1Q".to_string(),
            ],
            vec![
                "L2I".to_string(),
                "L2X".to_string(),
                "L2Q".to_string(),
                "L1I".to_string(),
                "L1X".to_string(),
                "L1Q".to_string(),
            ],
            vec![
                "S2I".to_string(),
                "S2X".to_string(),
                "S2Q".to_string(),
                "S1I".to_string(),
                "S1X".to_string(),
                "S1Q".to_string(),
            ],
        ),
        (
            SignalBand::B2,
            SignalCode::B2I,
            signal_spec_beidou_b2i(),
            vec!["C7I".to_string(), "C7X".to_string(), "C7Q".to_string()],
            vec!["L7I".to_string(), "L7X".to_string(), "L7Q".to_string()],
            vec!["S7I".to_string(), "S7X".to_string(), "S7Q".to_string()],
        ),
    ] {
        let Some(pseudorange_index) =
            find_observation_index(observation_types, &pseudorange_candidates)
        else {
            continue;
        };
        let pseudorange_type = observation_types[pseudorange_index].clone();
        channels.push(ImportedObservationChannel {
            band,
            code,
            signal,
            pseudorange_index,
            pseudorange_type,
            carrier_phase_index: find_observation_index(
                observation_types,
                &carrier_phase_candidates,
            ),
            carrier_phase_type: find_observation_type(observation_types, &carrier_phase_candidates),
            signal_strength_index: find_observation_index(
                observation_types,
                &signal_strength_candidates,
            ),
            signal_strength_type: find_observation_type(
                observation_types,
                &signal_strength_candidates,
            ),
        });
    }

    if channels.is_empty() {
        return Err(ParseError {
            message: format!(
                "RINEX OBS file does not provide a supported BeiDou observation channel; found {}",
                observation_types.join(", ")
            ),
        });
    }

    Ok(channels)
}

fn build_constellation_observations(
    satellite: SatId,
    observations: &[Option<f64>],
    channels: &[ImportedObservationChannel],
) -> Result<Vec<ObsSatellite>, ParseError> {
    let mut satellites = Vec::new();

    for channel in channels {
        let pseudorange_m = match observations.get(channel.pseudorange_index).copied().flatten() {
            Some(value) if value.is_finite() && value > 0.0 => value,
            Some(value) => {
                return Err(ParseError {
                    message: format!(
                        "invalid pseudorange for {:?}-{:02} {:?}: {value}",
                        satellite.constellation, satellite.prn, channel.band
                    ),
                });
            }
            None => continue,
        };

        let carrier_phase_cycles = channel
            .carrier_phase_index
            .and_then(|index| observations.get(index).copied().flatten())
            .unwrap_or(0.0);
        let carrier_phase_present = channel
            .carrier_phase_index
            .and_then(|index| observations.get(index).copied().flatten())
            .is_some();
        let cn0_dbhz = imported_cn0_dbhz(observations, channel.signal_strength_index);
        let mut metadata = ObsMetadata::default();
        metadata.tracking_mode = "rinex_import".to_string();
        metadata.signal = channel.signal;
        metadata.tracking_state = "external_file".to_string();
        metadata.observation_lock_state = "imported".to_string();
        metadata.pseudorange_model = format!("rinex_observation:{}", channel.pseudorange_type);
        metadata.carrier_phase_model = channel
            .carrier_phase_type
            .as_ref()
            .map(|observation_type| format!("rinex_observation:{observation_type}"))
            .unwrap_or_else(|| "unavailable".to_string());
        metadata.doppler_model = "unavailable".to_string();
        metadata.time_tag_source = "rinex_epoch_utc".to_string();

        satellites.push(ObsSatellite {
            signal_id: SigId { sat: satellite, band: channel.band, code: channel.code },
            pseudorange_m: Meters(pseudorange_m),
            pseudorange_var_m2: 0.0,
            carrier_phase_cycles: Cycles(carrier_phase_cycles),
            carrier_phase_var_cycles2: 0.0,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 0.0,
            cn0_dbhz,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: carrier_phase_present,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: None,
            metadata,
        });
    }

    Ok(satellites)
}

fn imported_cn0_dbhz(observations: &[Option<f64>], signal_strength_index: Option<usize>) -> f64 {
    let Some(signal_strength_dbhz) =
        signal_strength_index.and_then(|index| observations.get(index).copied().flatten())
    else {
        return 0.0;
    };
    if signal_strength_dbhz.is_finite() && (0.0..=80.0).contains(&signal_strength_dbhz) {
        signal_strength_dbhz
    } else {
        0.0
    }
}

fn build_obs_epoch(
    epoch_idx: u64,
    receive_gps_time: GpsTime,
    discontinuity: bool,
    sats: Vec<ObsSatellite>,
) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: Seconds(receive_gps_time.to_seconds()),
        source_time: Default::default(),
        gps_week: Some(receive_gps_time.week),
        tow_s: Some(Seconds(receive_gps_time.tow_s)),
        epoch_idx,
        discontinuity,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: Default::default(),
        decision_reason: None,
        manifest: None,
    }
}

fn parse_observation_record(
    lines: &[&str],
    observation_count: usize,
    prefix_width: usize,
) -> Result<Vec<Option<f64>>, ParseError> {
    let mut observations = Vec::with_capacity(observation_count);

    for (line_offset, line) in lines.iter().enumerate() {
        let mut field_index = if line_offset == 0 { prefix_width } else { 0usize };
        while field_index < line.len() && observations.len() < observation_count {
            let end = (field_index + OBSERVATION_FIELD_WIDTH).min(line.len());
            let field = line.get(field_index..end).unwrap_or_default();
            let value = field.get(..14).unwrap_or_default().trim();
            if value.is_empty() {
                observations.push(None);
            } else {
                observations.push(Some(parse_rinex_float(value)?));
            }
            field_index += OBSERVATION_FIELD_WIDTH;
        }
    }

    while observations.len() < observation_count {
        observations.push(None);
    }

    Ok(observations)
}

fn observation_record_line_count(observation_count: usize) -> usize {
    observation_count.div_ceil(OBSERVATIONS_PER_LINE)
}

fn rinex_3_observation_record_line_count(lines: &[&str], observation_count: usize) -> usize {
    let mut line_count = 1usize;
    let max_line_count = observation_record_line_count(observation_count);

    while line_count < max_line_count {
        let Some(next_line) = lines.get(line_count) else {
            break;
        };
        if next_line.trim().is_empty()
            || next_line.starts_with('>')
            || looks_like_rinex_3_satellite_record(next_line)
        {
            break;
        }
        line_count += 1;
    }

    line_count
}

fn observation_tokens(line: &str, start: usize, width: usize) -> Vec<String> {
    line.get(start..)
        .unwrap_or_default()
        .as_bytes()
        .chunks(width)
        .map(|chunk| String::from_utf8_lossy(chunk).trim().to_string())
        .filter(|chunk| !chunk.is_empty())
        .collect()
}

fn looks_like_rinex_3_satellite_record(line: &str) -> bool {
    parse_satellite_token(line.get(..3).unwrap_or_default()).is_ok()
}

fn parse_satellite_token(token: &str) -> Result<RinexSatelliteToken, ParseError> {
    let trimmed = token.trim();
    if trimmed.is_empty() {
        return Err(ParseError { message: "empty RINEX OBS satellite token".to_string() });
    }
    let (system, constellation, prn_text) = match trimmed.chars().next() {
        Some('G') => ('G', Constellation::Gps, trimmed[1..].trim()),
        Some('R') => ('R', Constellation::Glonass, trimmed[1..].trim()),
        Some('E') => ('E', Constellation::Galileo, trimmed[1..].trim()),
        Some('C') => ('C', Constellation::Beidou, trimmed[1..].trim()),
        Some('S') => ('S', Constellation::Unknown, trimmed[1..].trim()),
        Some(char) if char.is_ascii_digit() => ('G', Constellation::Gps, trimmed),
        Some(other) => {
            return Err(ParseError {
                message: format!("unsupported RINEX OBS constellation designator '{other}'"),
            });
        }
        None => unreachable!(),
    };
    let prn = prn_text.trim().parse::<u8>().map_err(|err| ParseError {
        message: format!("invalid RINEX OBS satellite identifier '{trimmed}': {err}"),
    })?;
    Ok(RinexSatelliteToken { system, satellite: SatId { constellation, prn } })
}

fn observation_types_for_system(header: &RinexObservationHeader, system: char) -> &[String] {
    if system == 'G' && header.version < 3.0 {
        return header.gps_observation_types().unwrap_or(&[]);
    }
    header.obs_types_by_system.get(&system).map(Vec::as_slice).unwrap_or(&[])
}

fn rinex_epoch_to_gps_time(
    year: i32,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: f64,
    time_system: RinexObservationTimeSystem,
) -> Result<GpsTime, ParseError> {
    let whole_seconds = second.floor();
    let nanos = ((second - whole_seconds) * 1_000_000_000.0).round();
    let date = Date::from_calendar_date(
        year,
        Month::try_from(month)
            .map_err(|_| ParseError { message: format!("invalid RINEX OBS month {month}") })?,
        day,
    )
    .map_err(|err| ParseError { message: format!("invalid RINEX OBS date: {err}") })?;
    let time = Time::from_hms_nano(hour, minute, whole_seconds as u8, nanos as u32)
        .map_err(|err| ParseError { message: format!("invalid RINEX OBS time: {err}") })?;
    let utc = PrimitiveDateTime::new(date, time).assume_utc();
    let unix_s = utc.unix_timestamp_nanos() as f64 / 1_000_000_000.0;
    Ok(match time_system {
        RinexObservationTimeSystem::Gps => GpsTime::from_seconds(unix_s - GPS_UNIX_EPOCH_OFFSET_S),
        RinexObservationTimeSystem::Utc => {
            utc_to_gps(bijux_gnss_core::api::UtcTime { unix_s }, &LeapSeconds::default_table())
        }
    })
}

fn parse_rinex_time_system(line: &str) -> Option<RinexObservationTimeSystem> {
    match line.get(48..51).unwrap_or_default().trim() {
        "" | "GPS" => Some(RinexObservationTimeSystem::Gps),
        "UTC" => Some(RinexObservationTimeSystem::Utc),
        _ => None,
    }
}

fn parse_rinex_float(field: &str) -> Result<f64, ParseError> {
    field
        .trim()
        .replace(['D', 'd'], "E")
        .parse::<f64>()
        .map_err(|err| ParseError { message: format!("invalid RINEX OBS float '{field}': {err}") })
}

fn parse_i32_field(line: &str, start: usize, end: usize, label: &str) -> Result<i32, ParseError> {
    line.get(start..end)
        .unwrap_or_default()
        .trim()
        .parse::<i32>()
        .map_err(|err| ParseError { message: format!("invalid {label} '{}': {err}", line.trim()) })
}

fn parse_u8_field(line: &str, start: usize, end: usize, label: &str) -> Result<u8, ParseError> {
    line.get(start..end)
        .unwrap_or_default()
        .trim()
        .parse::<u8>()
        .map_err(|err| ParseError { message: format!("invalid {label} '{}': {err}", line.trim()) })
}

fn parse_usize_field(
    line: &str,
    start: usize,
    end: usize,
    label: &str,
) -> Result<usize, ParseError> {
    line.get(start..end)
        .unwrap_or_default()
        .trim()
        .parse::<usize>()
        .map_err(|err| ParseError { message: format!("invalid {label} '{}': {err}", line.trim()) })
}

fn parse_f64_field(line: &str, start: usize, end: usize, label: &str) -> Result<f64, ParseError> {
    parse_rinex_float(line.get(start..end).unwrap_or_default())
        .map_err(|err| ParseError { message: format!("{label}: {}", err.message) })
}

fn normalize_rinex_2_year(year: i32) -> i32 {
    if year >= 80 {
        1900 + year
    } else {
        2000 + year
    }
}

fn find_observation_index(observation_types: &[String], candidates: &[String]) -> Option<usize> {
    candidates.iter().find_map(|candidate| {
        observation_types.iter().position(|observation_type| observation_type == candidate)
    })
}

fn find_observation_type(observation_types: &[String], candidates: &[String]) -> Option<String> {
    find_observation_index(observation_types, candidates)
        .map(|index| observation_types[index].clone())
}

fn parse_rinex_version(line: &str) -> Result<f64, ParseError> {
    line.get(..9).unwrap_or_default().trim().parse::<f64>().map_err(|err| ParseError {
        message: format!("invalid RINEX OBS version '{}': {err}", line.trim()),
    })
}

fn parse_rinex_2_observation_types(lines: &[&str]) -> Result<(Vec<String>, usize), ParseError> {
    let first_line = lines.first().ok_or_else(|| ParseError {
        message: "missing RINEX OBS observation types line".to_string(),
    })?;
    let expected_count =
        first_line.get(..6).unwrap_or_default().trim().parse::<usize>().map_err(|err| {
            ParseError {
                message: format!("invalid RINEX 2 OBS type count '{}': {err}", first_line.trim()),
            }
        })?;
    let mut collected = Vec::with_capacity(expected_count);
    let mut consumed_lines = 0usize;

    while collected.len() < expected_count {
        let line = *lines.get(consumed_lines).ok_or_else(|| ParseError {
            message: format!(
                "truncated RINEX 2 OBS type block: expected {expected_count} observation types, found {}",
                collected.len()
            ),
        })?;
        if rinex_header_label(line) != "# / TYPES OF OBSERV" {
            return Err(ParseError {
                message: format!(
                    "expected RINEX 2 observation-type continuation, found '{}'",
                    rinex_header_label(line)
                ),
            });
        }
        let field_start = 6usize;
        let fields_per_line = 9usize;
        for field_index in 0..fields_per_line {
            if collected.len() == expected_count {
                break;
            }
            let start = field_start + field_index * 6;
            let end = start + 6;
            let token = line.get(start..end).unwrap_or_default().trim();
            if !token.is_empty() {
                collected.push(token.to_string());
            }
        }
        consumed_lines += 1;
    }

    Ok((collected, consumed_lines))
}

fn parse_rinex_3_observation_types(
    lines: &[&str],
) -> Result<(char, Vec<String>, usize), ParseError> {
    let first_line = lines.first().ok_or_else(|| ParseError {
        message: "missing RINEX 3 observation types line".to_string(),
    })?;
    let system = first_line.chars().next().ok_or_else(|| ParseError {
        message: "missing RINEX 3 observation-system designator".to_string(),
    })?;
    let expected_count =
        first_line.get(3..6).unwrap_or_default().trim().parse::<usize>().map_err(|err| {
            ParseError {
                message: format!("invalid RINEX 3 OBS type count '{}': {err}", first_line.trim()),
            }
        })?;
    let mut collected = Vec::with_capacity(expected_count);
    let mut consumed_lines = 0usize;

    while collected.len() < expected_count {
        let line = *lines.get(consumed_lines).ok_or_else(|| ParseError {
            message: format!(
                "truncated RINEX 3 OBS type block for system {system}: expected {expected_count} observation types, found {}",
                collected.len()
            ),
        })?;
        if rinex_header_label(line) != "SYS / # / OBS TYPES" {
            return Err(ParseError {
                message: format!(
                    "expected RINEX 3 observation-type continuation, found '{}'",
                    rinex_header_label(line)
                ),
            });
        }
        let current_system = line.chars().next().unwrap_or(' ');
        if current_system != system {
            return Err(ParseError {
                message: format!(
                    "mixed RINEX 3 observation-type continuation: expected system {system}, found {current_system}"
                ),
            });
        }
        let field_start = 7usize;
        let fields_per_line = 13usize;
        for field_index in 0..fields_per_line {
            if collected.len() == expected_count {
                break;
            }
            let start = field_start + field_index * 4;
            let end = start + 4;
            let token = line.get(start..end).unwrap_or_default().trim();
            if !token.is_empty() {
                collected.push(token.to_string());
            }
        }
        consumed_lines += 1;
    }

    Ok((system, collected, consumed_lines))
}

fn parse_three_floats(field: &str, label: &str) -> Result<(f64, f64, f64), ParseError> {
    let values = field
        .split_whitespace()
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|err| ParseError { message: format!("invalid {label}: {err}") })?;
    if values.len() < 3 {
        return Err(ParseError {
            message: format!("{label} requires three numeric fields, found {}", values.len()),
        });
    }
    Ok((values[0], values[1], values[2]))
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use bijux_gnss_core::api::{
        check_dual_frequency_observations, signal_spec_beidou_b1i, signal_spec_beidou_b2i,
        signal_spec_galileo_e1b, signal_spec_galileo_e5a, signal_spec_gps_l2c, signal_spec_gps_l5,
        validate_obs_epochs, Constellation, SatId, SignalBand, SignalCode,
    };

    use super::{
        parse_rinex_beidou_observation_dataset, parse_rinex_galileo_observation_dataset,
        parse_rinex_gps_observation_dataset, parse_rinex_observation_header, RinexCodeBiasState,
    };

    fn fixture(name: &str) -> String {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
        std::fs::read_to_string(&path)
            .unwrap_or_else(|_| panic!("read RINEX fixture {}", path.display()))
    }

    #[test]
    fn parse_public_rinex_2_observation_header() {
        let data = fixture("georinex_14601736_20180622.obs");
        let (header, header_line_count) =
            parse_rinex_observation_header(&data).expect("parse RINEX observation header");

        assert_eq!(header_line_count, 33);
        assert!((header.version - 2.11).abs() < 1.0e-12);
        assert_eq!(header.marker_name.as_deref(), Some("st"));
        assert_eq!(
            header.approx_position_ecef_m,
            Some((-4_647_137.5830, 2_562_189.6255, -3_526_626.7006))
        );
        assert_eq!(header.interval_s, Some(15.0));

        let gps_observation_types =
            header.gps_observation_types().expect("RINEX 2 observation types present");
        assert_eq!(gps_observation_types, ["C1", "C2", "C8", "L1", "L2", "L8", "P2"]);
    }

    #[test]
    fn parse_rinex_3_observation_type_header() {
        let data = [
            format!(
                "{:<60}{}",
                "     3.04           OBSERVATION DATA    G (GPS)", "RINEX VERSION / TYPE"
            ),
            format!("{:<60}{}", "G    5 C1C L1C D1C S1C C5Q", "SYS / # / OBS TYPES"),
            format!("{:<60}{}", "", "END OF HEADER"),
        ]
        .join("\n");
        let (header, header_line_count) =
            parse_rinex_observation_header(&data).expect("parse RINEX 3 observation header");

        assert_eq!(header_line_count, 3);
        assert_eq!(
            header.gps_observation_types().expect("GPS observation types"),
            ["C1C", "L1C", "D1C", "S1C", "C5Q"]
        );
    }

    #[test]
    fn parse_public_rinex_2_gps_observation_epochs() {
        let data = fixture("georinex_14601736_20180622.obs");
        let dataset =
            parse_rinex_gps_observation_dataset(&data).expect("parse RINEX GPS observation data");

        assert!((dataset.version - 2.11).abs() < 1.0e-12);
        assert_eq!(dataset.marker_name.as_deref(), Some("st"));
        assert_eq!(dataset.interval_s, Some(15.0));
        assert_eq!(dataset.code_bias_status.state, RinexCodeBiasState::Unknown);
        assert_eq!(dataset.code_bias_status.source, None);
        assert_eq!(dataset.observation_channels.len(), 2);
        assert_eq!(dataset.observation_channels[0].band, SignalBand::L1);
        assert_eq!(dataset.observation_channels[0].code, SignalCode::Ca);
        assert_eq!(dataset.observation_channels[0].pseudorange_observation_type, "C1");
        assert_eq!(
            dataset.observation_channels[0].carrier_phase_observation_type.as_deref(),
            Some("L1")
        );
        assert_eq!(dataset.observation_channels[1].band, SignalBand::L2);
        assert_eq!(dataset.observation_channels[1].code, SignalCode::Py);
        assert_eq!(dataset.observation_channels[1].pseudorange_observation_type, "P2");
        assert_eq!(
            dataset.observation_channels[1].carrier_phase_observation_type.as_deref(),
            Some("L2")
        );
        assert_eq!(dataset.epochs.len(), 3);
        validate_obs_epochs(&dataset.epochs).expect("imported dual-frequency epochs must validate");

        let first_epoch = &dataset.epochs[0];
        assert_eq!(first_epoch.gps_week, Some(2006));
        assert!(first_epoch.sats.len() > 5);
        assert!(first_epoch.valid);
        assert!(!first_epoch.discontinuity);

        let g03_l1 = first_epoch
            .sats
            .iter()
            .find(|sat| {
                sat.signal_id.sat == SatId { constellation: Constellation::Gps, prn: 3 }
                    && sat.signal_id.band == SignalBand::L1
            })
            .expect("G03 L1 observation");
        assert!((g03_l1.pseudorange_m.0 - 22_719_526.844).abs() < 1.0e-6);
        assert!((g03_l1.carrier_phase_cycles.0 - 119_391_903.878).abs() < 1.0e-6);
        assert_eq!(g03_l1.timing, None);

        let any_l2 = first_epoch
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::L2)
            .expect("at least one L2 observation");
        assert!(any_l2.lock_flags.code_lock);
        assert!(any_l2.lock_flags.carrier_lock);

        let dual_frequency = check_dual_frequency_observations(&dataset.epochs);
        assert!(dual_frequency.complete_pairs > 0);
        assert_eq!(dual_frequency.complete_pairs, dual_frequency.l1_l2_pairs);
    }

    #[test]
    fn parse_rinex_3_gps_observation_epoch() {
        let data = [
            format!(
                "{:<60}{}",
                "     3.04           OBSERVATION DATA    G (GPS)", "RINEX VERSION / TYPE"
            ),
            format!("{:<60}{}", "gps-station", "MARKER NAME"),
            format!("{:<60}{}", "G    4 C1C L1C C2P L2P", "SYS / # / OBS TYPES"),
            format!("{:<60}{}", "", "END OF HEADER"),
            "> 2022 05 14 00 00 00.0000000  0  2".to_string(),
            format!(
                "{:<3}{:>14}  {:>14}  {:>14}  {:>14}",
                "G01", 20345678.123, 123456.250, 20345680.750, 123450.500
            ),
            format!(
                "{:<3}{:>14}  {:>14}  {:>14}  {:>14}",
                "G02", 21345678.456, 223456.500, 21345681.125, 223450.750
            ),
        ]
        .join("\n");
        let dataset =
            parse_rinex_gps_observation_dataset(&data).expect("parse synthetic RINEX 3 data");

        assert_eq!(dataset.code_bias_status.state, RinexCodeBiasState::Unknown);
        assert_eq!(dataset.observation_channels.len(), 2);
        assert_eq!(dataset.epochs.len(), 1);
        assert_eq!(dataset.epochs[0].sats.len(), 4);
        assert_eq!(
            dataset.epochs[0].sats[0].signal_id.sat,
            SatId { constellation: Constellation::Gps, prn: 1 }
        );
        let dual_frequency = check_dual_frequency_observations(&dataset.epochs);
        assert_eq!(dual_frequency.complete_pairs, 2);
    }

    #[test]
    fn parse_rinex_3_l2c_observation_epoch() {
        let data = [
            format!(
                "{:<60}{}",
                "     3.04           OBSERVATION DATA    G (GPS)", "RINEX VERSION / TYPE"
            ),
            format!("{:<60}{}", "gps-station", "MARKER NAME"),
            format!("{:<60}{}", "G    4 C1C L1C C2L L2L", "SYS / # / OBS TYPES"),
            format!("{:<60}{}", "", "END OF HEADER"),
            "> 2022 05 14 00 00 00.0000000  0  1".to_string(),
            format!(
                "{:<3}{:>14}  {:>14}  {:>14}  {:>14}",
                "G01", 20345678.123, 123456.250, 20345680.750, 123450.500
            ),
        ]
        .join("\n");
        let dataset =
            parse_rinex_gps_observation_dataset(&data).expect("parse synthetic RINEX 3 L2C data");

        assert_eq!(dataset.observation_channels.len(), 2);
        assert_eq!(dataset.observation_channels[1].band, SignalBand::L2);
        assert_eq!(dataset.observation_channels[1].code, SignalCode::L2C);
        assert_eq!(dataset.observation_channels[1].pseudorange_observation_type, "C2L");
        assert_eq!(dataset.epochs.len(), 1);
        assert_eq!(dataset.epochs[0].sats.len(), 2);
        let l2 = dataset.epochs[0]
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::L2)
            .expect("L2C observation");
        assert_eq!(l2.signal_id.code, SignalCode::L2C);
        assert_eq!(l2.metadata.signal, signal_spec_gps_l2c());
    }

    #[test]
    fn parse_rinex_3_l5_observation_epoch() {
        let data = [
            format!(
                "{:<60}{}",
                "     3.04           OBSERVATION DATA    G (GPS)", "RINEX VERSION / TYPE"
            ),
            format!("{:<60}{}", "gps-station", "MARKER NAME"),
            format!("{:<60}{}", "G    5 C1C L1C C5Q L5Q S5Q", "SYS / # / OBS TYPES"),
            format!("{:<60}{}", "", "END OF HEADER"),
            "> 2022 05 14 00 00 00.0000000  0  1".to_string(),
            format!(
                "{:<3}{:>14}  {:>14}  {:>14}  {:>14}  {:>14}",
                "G01", 20345678.123, 123456.250, 20345680.750, 123450.500, 49.5
            ),
        ]
        .join("\n");
        let dataset =
            parse_rinex_gps_observation_dataset(&data).expect("parse synthetic RINEX 3 L5 data");

        assert_eq!(dataset.observation_channels.len(), 2);
        assert_eq!(dataset.observation_channels[1].band, SignalBand::L5);
        assert_eq!(dataset.observation_channels[1].code, SignalCode::Unknown);
        assert_eq!(dataset.observation_channels[1].pseudorange_observation_type, "C5Q");
        assert_eq!(
            dataset.observation_channels[1].carrier_phase_observation_type.as_deref(),
            Some("L5Q")
        );
        assert_eq!(
            dataset.observation_channels[1].signal_strength_observation_type.as_deref(),
            Some("S5Q")
        );
        assert_eq!(dataset.epochs.len(), 1);
        assert_eq!(dataset.epochs[0].sats.len(), 2);
        let l5 = dataset.epochs[0]
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::L5)
            .expect("L5 observation");
        assert_eq!(l5.signal_id.code, SignalCode::Unknown);
        assert_eq!(l5.metadata.signal, signal_spec_gps_l5());
        assert_eq!(l5.cn0_dbhz, 49.5);
    }

    #[test]
    fn parse_rinex_3_galileo_e1_e5_observation_epoch() {
        let data = [
            format!(
                "{:<60}{}",
                "     3.04           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"
            ),
            format!("{:<60}{}", "galileo-station", "MARKER NAME"),
            format!("{:<60}{}", "E    5 C1C L1C C5Q L5Q S5Q", "SYS / # / OBS TYPES"),
            format!("{:<60}{}", "", "END OF HEADER"),
            "> 2022 05 14 00 00 00.0000000  0  1".to_string(),
            format!(
                "{:<3}{:>14}  {:>14}  {:>14}  {:>14}  {:>14}",
                "E11", 24345678.123, 223456.250, 24345680.750, 223450.500, 47.0
            ),
        ]
        .join("\n");
        let dataset = parse_rinex_galileo_observation_dataset(&data)
            .expect("parse synthetic RINEX 3 Galileo E1/E5 data");

        assert_eq!(dataset.observation_channels.len(), 2);
        assert_eq!(dataset.observation_channels[0].band, SignalBand::E1);
        assert_eq!(dataset.observation_channels[0].code, SignalCode::E1B);
        assert_eq!(dataset.observation_channels[1].band, SignalBand::E5);
        assert_eq!(dataset.observation_channels[1].code, SignalCode::E5a);
        assert_eq!(dataset.epochs.len(), 1);
        assert_eq!(dataset.epochs[0].sats.len(), 2);
        let e1 = dataset.epochs[0]
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::E1)
            .expect("E1 observation");
        let e5 = dataset.epochs[0]
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::E5)
            .expect("E5 observation");
        assert_eq!(e1.metadata.signal, signal_spec_galileo_e1b());
        assert_eq!(e5.metadata.signal, signal_spec_galileo_e5a());
        assert_eq!(e5.cn0_dbhz, 47.0);
        let dual_frequency = check_dual_frequency_observations(&dataset.epochs);
        assert_eq!(dual_frequency.complete_pairs, 1);
        assert_eq!(dual_frequency.e1_e5_pairs, 1);
    }

    #[test]
    fn parse_rinex_3_beidou_b1_b2_observation_epoch() {
        let data = [
            format!(
                "{:<60}{}",
                "     3.04           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"
            ),
            format!("{:<60}{}", "beidou-station", "MARKER NAME"),
            format!("{:<60}{}", "C    5 C2I L2I C7I L7I S7I", "SYS / # / OBS TYPES"),
            format!("{:<60}{}", "", "END OF HEADER"),
            "> 2022 05 14 00 00 00.0000000  0  1".to_string(),
            format!(
                "{:<3}{:>14}  {:>14}  {:>14}  {:>14}  {:>14}",
                "C11", 24_345_678.125, 123_456.250, 24_345_679.875, 123_450.500, 46.5
            ),
        ]
        .join("\n");
        let dataset = parse_rinex_beidou_observation_dataset(&data)
            .expect("parse synthetic RINEX 3 BeiDou B1/B2 data");

        assert_eq!(dataset.observation_channels.len(), 2);
        assert_eq!(dataset.observation_channels[0].band, SignalBand::B1);
        assert_eq!(dataset.observation_channels[0].code, SignalCode::B1I);
        assert_eq!(dataset.observation_channels[1].band, SignalBand::B2);
        assert_eq!(dataset.observation_channels[1].code, SignalCode::B2I);
        assert_eq!(dataset.epochs.len(), 1);
        assert_eq!(dataset.epochs[0].sats.len(), 2);
        let b1 = dataset.epochs[0]
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::B1)
            .expect("B1 observation");
        let b2 = dataset.epochs[0]
            .sats
            .iter()
            .find(|sat| sat.signal_id.band == SignalBand::B2)
            .expect("B2 observation");
        assert_eq!(b1.metadata.signal, signal_spec_beidou_b1i());
        assert_eq!(b2.metadata.signal, signal_spec_beidou_b2i());
        assert_eq!(b2.cn0_dbhz, 46.5);
        let dual_frequency = check_dual_frequency_observations(&dataset.epochs);
        assert_eq!(dual_frequency.complete_pairs, 1);
        assert_eq!(dual_frequency.b1_b2_pairs, 1);
    }

    #[test]
    fn parse_public_rinex_3_mixed_gps_observation_epochs() {
        let data = fixture("glab_gage_20100305.obs");
        let dataset =
            parse_rinex_gps_observation_dataset(&data).expect("parse public RINEX 3 data");

        assert!((dataset.version - 3.01).abs() < 1.0e-12);
        assert_eq!(dataset.marker_name.as_deref(), Some("MRKR"));
        assert_eq!(dataset.interval_s, Some(30.0));
        assert_eq!(dataset.code_bias_status.state, RinexCodeBiasState::Applied);
        assert_eq!(
            dataset.code_bias_status.source.as_deref(),
            Some("G CC2NONCC          p1c1bias.hist @ goby.nrl.navy.mil")
        );
        assert_eq!(dataset.observation_channels.len(), 2);
        assert_eq!(dataset.observation_channels[0].band, SignalBand::L1);
        assert_eq!(dataset.observation_channels[0].code, SignalCode::Ca);
        assert_eq!(dataset.observation_channels[1].band, SignalBand::L2);
        assert_eq!(dataset.observation_channels[1].code, SignalCode::Py);
        assert_eq!(dataset.epochs.len(), 2);
        validate_obs_epochs(&dataset.epochs).expect("imported mixed-band epochs must validate");
        assert!(dataset.epochs[0].sats.len() > 10);
        let g07_l1 = dataset.epochs[0]
            .sats
            .iter()
            .find(|sat| {
                sat.signal_id.sat == SatId { constellation: Constellation::Gps, prn: 7 }
                    && sat.signal_id.band == SignalBand::L1
            })
            .expect("G07 L1 observation");
        assert!((g07_l1.pseudorange_m.0 - 22_227_666.760).abs() < 1.0e-6);
        let dual_frequency = check_dual_frequency_observations(&dataset.epochs);
        assert!(dual_frequency.complete_pairs > 0);
        assert_eq!(dual_frequency.complete_pairs, dual_frequency.l1_l2_pairs);
    }
}
