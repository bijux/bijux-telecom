use std::collections::BTreeMap;
use std::fs::File;
use std::io::Write;
use std::path::Path;

use bijux_gnss_core::api::{
    utc_to_gps, Constellation, Cycles, GpsTime, Hertz, IoError, LeapSeconds, LockFlags, Meters,
    ObsEpoch, ObsMetadata, ObsSatellite, ObservationStatus, ParseError, ReceiverRole, SatId,
    Seconds, SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_signal::api::{
    signal_spec_beidou_b1i, signal_spec_beidou_b2i, signal_spec_galileo_e1b,
    signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l2c,
    signal_spec_gps_l5_i, signal_spec_gps_l5_q,
};
use time::{Date, Month, PrimitiveDateTime, Time};

const OBSERVATION_FIELD_WIDTH: usize = 16;
const OBSERVATIONS_PER_LINE: usize = 5;
const GPS_UNIX_EPOCH_OFFSET_S: f64 = 315_964_800.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RinexObservationTimeSystem {
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

#[derive(Debug, Clone, PartialEq)]
pub struct RinexObservationValue {
    pub value: Option<f64>,
    pub loss_of_lock_indicator: Option<u8>,
    pub signal_strength_indicator: Option<u8>,
}

impl RinexObservationValue {
    fn empty() -> Self {
        Self { value: None, loss_of_lock_indicator: None, signal_strength_indicator: None }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RinexObservationEpochTime {
    pub year: i32,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RinexSatelliteObservation {
    pub system: char,
    pub satellite: SatId,
    pub observations: Vec<RinexObservationValue>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RinexObservationEpoch {
    pub epoch_time: RinexObservationEpochTime,
    pub receive_gps_time: GpsTime,
    pub event_flag: u8,
    pub receiver_clock_offset_s: Option<f64>,
    pub satellites: Vec<RinexSatelliteObservation>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RinexObservationEvent {
    pub event_flag: u8,
    pub records: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum RinexObservationRecord {
    Epoch(RinexObservationEpoch),
    Event(RinexObservationEvent),
}

#[derive(Debug, Clone, PartialEq)]
pub struct RinexObservationDataset {
    pub version: f64,
    pub marker_name: Option<String>,
    pub approx_position_ecef_m: Option<(f64, f64, f64)>,
    pub interval_s: Option<f64>,
    pub time_system: RinexObservationTimeSystem,
    pub observation_types_v2: Option<Vec<String>>,
    pub observation_types_by_system: BTreeMap<char, Vec<String>>,
    pub code_bias_status_by_system: BTreeMap<char, String>,
    pub records: Vec<RinexObservationRecord>,
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
        epoch_time: RinexObservationEpochTime,
        receive_gps_time: GpsTime,
        event_flag: u8,
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

pub fn parse_rinex_observation_dataset(data: &str) -> Result<RinexObservationDataset, ParseError> {
    let lines = data.lines().collect::<Vec<_>>();
    let (header, header_line_count) = parse_rinex_observation_header(data)?;
    let records = if header.version >= 3.0 {
        parse_rinex_3_observation_records(&lines[header_line_count..], &header)?
    } else {
        let observation_types = header.obs_types_v2.as_ref().ok_or_else(|| ParseError {
            message: "RINEX 2 OBS file does not declare observation types".to_string(),
        })?;
        parse_rinex_2_observation_records(
            &lines[header_line_count..],
            observation_types,
            header.time_system,
        )?
    };

    Ok(RinexObservationDataset {
        version: header.version,
        marker_name: header.marker_name,
        approx_position_ecef_m: header.approx_position_ecef_m,
        interval_s: header.interval_s,
        time_system: header.time_system,
        observation_types_v2: header.obs_types_v2,
        observation_types_by_system: header.obs_types_by_system,
        code_bias_status_by_system: header.dcbs_applied_by_system,
        records,
    })
}

pub fn write_rinex_observation_dataset(
    path: &Path,
    dataset: &RinexObservationDataset,
) -> Result<(), IoError> {
    let mut file = File::create(path).map_err(|err| IoError { message: err.to_string() })?;
    file.write_all(format_rinex_observation_dataset(dataset)?.as_bytes())
        .map_err(|err| IoError { message: err.to_string() })
}

pub fn format_rinex_observation_dataset(
    dataset: &RinexObservationDataset,
) -> Result<String, IoError> {
    let mut output = String::new();
    push_header_line(
        &mut output,
        &format!(
            "{:>9.2}           OBSERVATION DATA    {}",
            dataset.version,
            rinex_file_type(dataset)
        ),
        "RINEX VERSION / TYPE",
    );
    push_header_line(&mut output, "bijux-gnss", "PGM / RUN BY / DATE");
    if let Some(marker_name) = &dataset.marker_name {
        push_header_line(&mut output, marker_name, "MARKER NAME");
    }
    if let Some((x_m, y_m, z_m)) = dataset.approx_position_ecef_m {
        push_header_line(
            &mut output,
            &format!("{x_m:14.4}{y_m:14.4}{z_m:14.4}"),
            "APPROX POSITION XYZ",
        );
    }
    if let Some(interval_s) = dataset.interval_s {
        push_header_line(&mut output, &format!("{interval_s:10.3}"), "INTERVAL");
    }
    if dataset.version < 3.0 {
        let observation_types = dataset.observation_types_v2.as_deref().ok_or_else(|| IoError {
            message: "RINEX 2 observation dataset is missing observation types".to_string(),
        })?;
        push_rinex_2_observation_type_header(&mut output, observation_types);
    } else {
        for (system, observation_types) in &dataset.observation_types_by_system {
            push_rinex_3_observation_type_header(&mut output, *system, observation_types);
        }
    }
    for (system, source) in &dataset.code_bias_status_by_system {
        let content =
            if source.starts_with(*system) { source.clone() } else { format!("{system} {source}") };
        push_header_line(&mut output, &content, "SYS / DCBS APPLIED");
    }
    push_header_line(&mut output, "", "TIME OF FIRST OBS");
    push_header_line(&mut output, "", "END OF HEADER");

    for record in &dataset.records {
        match record {
            RinexObservationRecord::Epoch(epoch) => {
                if dataset.version < 3.0 {
                    push_rinex_2_epoch_record(&mut output, dataset, epoch)?;
                } else {
                    push_rinex_3_epoch_record(&mut output, dataset, epoch)?;
                }
            }
            RinexObservationRecord::Event(event) => {
                if dataset.version < 3.0 {
                    output.push_str(&format!(
                        "{:28}{:>1}{:>3}\n",
                        "",
                        event.event_flag,
                        event.records.len()
                    ));
                } else {
                    output.push_str(&format!(
                        "> {:04} {:02} {:02} {:02} {:02} {:>10.7} {:>2} {:>2}\n",
                        1980,
                        1,
                        6,
                        0,
                        0,
                        0.0,
                        event.event_flag,
                        event.records.len()
                    ));
                }
                for line in &event.records {
                    output.push_str(line);
                    output.push('\n');
                }
            }
        }
    }

    Ok(output)
}

fn push_header_line(output: &mut String, content: &str, label: &str) {
    let mut content = content.to_string();
    if content.len() > 60 {
        content.truncate(60);
    }
    output.push_str(&format!("{content:<60}{label}\n"));
}

fn rinex_file_type(dataset: &RinexObservationDataset) -> String {
    let mut systems = dataset
        .records
        .iter()
        .filter_map(|record| match record {
            RinexObservationRecord::Epoch(epoch) => Some(epoch),
            RinexObservationRecord::Event(_) => None,
        })
        .flat_map(|epoch| epoch.satellites.iter().map(|satellite| satellite.system))
        .collect::<Vec<_>>();
    systems.sort_unstable();
    systems.dedup();
    match systems.as_slice() {
        ['G'] => "G (GPS)".to_string(),
        ['R'] => "R (GLO)".to_string(),
        ['E'] => "E (GAL)".to_string(),
        ['C'] => "C (BDS)".to_string(),
        _ => "M (MIXED)".to_string(),
    }
}

fn push_rinex_2_observation_type_header(output: &mut String, observation_types: &[String]) {
    let mut index = 0usize;
    while index < observation_types.len() {
        let count = if index == 0 {
            format!("{:>6}", observation_types.len())
        } else {
            "      ".to_string()
        };
        let mut content = count;
        for observation_type in observation_types.iter().skip(index).take(9) {
            content.push_str(&format!("{observation_type:>6}"));
        }
        push_header_line(output, &content, "# / TYPES OF OBSERV");
        index += 9;
    }
}

fn push_rinex_3_observation_type_header(
    output: &mut String,
    system: char,
    observation_types: &[String],
) {
    let mut index = 0usize;
    while index < observation_types.len() {
        let mut content = if index == 0 {
            format!("{system}  {:>3} ", observation_types.len())
        } else {
            format!("{system}      ")
        };
        for observation_type in observation_types.iter().skip(index).take(13) {
            content.push_str(&format!("{observation_type:>4}"));
        }
        push_header_line(output, &content, "SYS / # / OBS TYPES");
        index += 13;
    }
}

fn push_rinex_2_epoch_record(
    output: &mut String,
    dataset: &RinexObservationDataset,
    epoch: &RinexObservationEpoch,
) -> Result<(), IoError> {
    let observation_count =
        dataset.observation_types_v2.as_ref().map(Vec::len).ok_or_else(|| IoError {
            message: "RINEX 2 observation dataset is missing observation types".to_string(),
        })?;
    let year = epoch.epoch_time.year.rem_euclid(100);
    let mut satellite_tokens = epoch
        .satellites
        .iter()
        .map(|satellite| format_satellite_token(satellite.system, satellite.satellite))
        .collect::<Vec<_>>();
    let first_tokens = satellite_tokens.drain(..satellite_tokens.len().min(12)).collect::<Vec<_>>();
    output.push_str(&format!(
        "{year:>3}{:>3}{:>3}{:>3}{:>3}{:>11.7}  {:>1}{:>3}{}\n",
        epoch.epoch_time.month,
        epoch.epoch_time.day,
        epoch.epoch_time.hour,
        epoch.epoch_time.minute,
        epoch.epoch_time.second,
        epoch.event_flag,
        epoch.satellites.len(),
        first_tokens.join("")
    ));
    while !satellite_tokens.is_empty() {
        let line_tokens =
            satellite_tokens.drain(..satellite_tokens.len().min(12)).collect::<Vec<_>>();
        output.push_str(&format!("{:32}{}\n", "", line_tokens.join("")));
    }
    for satellite in &epoch.satellites {
        push_observation_lines(output, "", &satellite.observations, observation_count, 5)?;
    }
    Ok(())
}

fn push_rinex_3_epoch_record(
    output: &mut String,
    dataset: &RinexObservationDataset,
    epoch: &RinexObservationEpoch,
) -> Result<(), IoError> {
    output.push_str(&format!(
        "> {:04} {:02} {:02} {:02} {:02} {:>10.7} {:>2} {:>2}",
        epoch.epoch_time.year,
        epoch.epoch_time.month,
        epoch.epoch_time.day,
        epoch.epoch_time.hour,
        epoch.epoch_time.minute,
        epoch.epoch_time.second,
        epoch.event_flag,
        epoch.satellites.len()
    ));
    if let Some(receiver_clock_offset_s) = epoch.receiver_clock_offset_s {
        output.push_str(&format!(" {receiver_clock_offset_s:>16.12}"));
    }
    output.push('\n');

    for satellite in &epoch.satellites {
        let observation_count = dataset
            .observation_types_by_system
            .get(&satellite.system)
            .map(Vec::len)
            .ok_or_else(|| IoError {
                message: format!("missing observation types for RINEX system {}", satellite.system),
            })?;
        let prefix = format_satellite_token(satellite.system, satellite.satellite);
        push_observation_lines(output, &prefix, &satellite.observations, observation_count, 4)?;
    }
    Ok(())
}

fn push_observation_lines(
    output: &mut String,
    first_prefix: &str,
    observations: &[RinexObservationValue],
    observation_count: usize,
    first_line_capacity: usize,
) -> Result<(), IoError> {
    if observations.len() > observation_count {
        return Err(IoError {
            message: format!(
                "RINEX satellite record has {} observations but only {observation_count} types",
                observations.len()
            ),
        });
    }
    let mut cells = observations.to_vec();
    cells.resize_with(observation_count, RinexObservationValue::empty);

    let mut index = 0usize;
    let first_take = cells.len().min(first_line_capacity);
    output.push_str(first_prefix);
    for cell in &cells[index..index + first_take] {
        output.push_str(&format_observation_cell(cell)?);
    }
    output.push('\n');
    index += first_take;
    while index < cells.len() {
        let take = (cells.len() - index).min(OBSERVATIONS_PER_LINE);
        for cell in &cells[index..index + take] {
            output.push_str(&format_observation_cell(cell)?);
        }
        output.push('\n');
        index += take;
    }
    Ok(())
}

fn format_observation_cell(cell: &RinexObservationValue) -> Result<String, IoError> {
    let value = cell.value.map(|value| format!("{value:>14.3}")).unwrap_or_else(|| " ".repeat(14));
    let loss_of_lock =
        format_observation_flag(cell.loss_of_lock_indicator, "loss-of-lock indicator")?;
    let signal_strength =
        format_observation_flag(cell.signal_strength_indicator, "signal-strength indicator")?;
    Ok(format!("{value}{loss_of_lock}{signal_strength}"))
}

fn format_observation_flag(value: Option<u8>, name: &str) -> Result<char, IoError> {
    match value {
        Some(value) if value <= 9 => Ok(char::from(b'0' + value)),
        Some(value) => {
            Err(IoError { message: format!("RINEX observation {name} must be 0..9, got {value}") })
        }
        None => Ok(' '),
    }
}

fn format_satellite_token(system: char, satellite: SatId) -> String {
    format!("{system}{:02}", satellite.prn)
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
                epoch_time: _,
                receive_gps_time,
                event_flag: _,
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

fn parse_rinex_2_observation_records(
    lines: &[&str],
    observation_types: &[String],
    time_system: RinexObservationTimeSystem,
) -> Result<Vec<RinexObservationRecord>, ParseError> {
    let mut records = Vec::new();
    let mut line_index = 0usize;

    while line_index < lines.len() {
        let line = lines[line_index];
        if line.trim().is_empty() {
            line_index += 1;
            continue;
        }

        let record = parse_rinex_2_epoch_header(&lines[line_index..], time_system)?;
        match record {
            Rinex2Record::Event { special_record_count } => {
                let event_line_start = line_index + 1;
                let event_line_end = event_line_start + special_record_count;
                let event_lines = lines
                    .get(event_line_start..event_line_end)
                    .ok_or_else(|| ParseError {
                        message: format!(
                            "truncated RINEX 2 event record: expected {special_record_count} records"
                        ),
                    })?
                    .iter()
                    .map(|line| (*line).to_string())
                    .collect();
                let event_flag = parse_u8_field(line, 28, 29, "RINEX 2 OBS event flag")?;
                records.push(RinexObservationRecord::Event(RinexObservationEvent {
                    event_flag,
                    records: event_lines,
                }));
                line_index = event_line_end;
            }
            Rinex2Record::ObservationEpoch {
                epoch_time,
                receive_gps_time,
                event_flag,
                discontinuity: _,
                satellites,
                header_line_count,
            } => {
                line_index += header_line_count;
                let observation_line_count = observation_record_line_count(observation_types.len());
                let mut raw_satellites = Vec::with_capacity(satellites.len());

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
                        parse_observation_cells(record_lines, observation_types.len(), 0)?;
                    line_index += observation_line_count;
                    raw_satellites.push(RinexSatelliteObservation {
                        system: rinex_system_from_constellation(satellite.constellation),
                        satellite,
                        observations,
                    });
                }

                records.push(RinexObservationRecord::Epoch(RinexObservationEpoch {
                    epoch_time,
                    receive_gps_time,
                    event_flag,
                    receiver_clock_offset_s: None,
                    satellites: raw_satellites,
                }));
            }
        }
    }

    Ok(records)
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

fn parse_rinex_3_observation_records(
    lines: &[&str],
    header: &RinexObservationHeader,
) -> Result<Vec<RinexObservationRecord>, ParseError> {
    let mut records = Vec::new();
    let mut line_index = 0usize;

    while line_index < lines.len() {
        let line = lines[line_index];
        if line.trim().is_empty() {
            line_index += 1;
            continue;
        }
        if !line.starts_with('>') {
            return Err(ParseError {
                message: format!("expected RINEX 3/4 epoch header line, found '{}'", line.trim()),
            });
        }

        let (epoch_time, receive_gps_time, event_flag, record_count, receiver_clock_offset_s) =
            parse_rinex_3_epoch_header_raw(line, header.time_system)?;
        line_index += 1;

        if event_flag > 1 {
            let event_lines = lines
                .get(line_index..line_index + record_count)
                .ok_or_else(|| ParseError {
                    message: format!(
                        "truncated RINEX 3/4 event record: expected {record_count} records"
                    ),
                })?
                .iter()
                .map(|line| (*line).to_string())
                .collect();
            records.push(RinexObservationRecord::Event(RinexObservationEvent {
                event_flag,
                records: event_lines,
            }));
            line_index += record_count;
            continue;
        }

        let mut satellites = Vec::with_capacity(record_count);
        for _ in 0..record_count {
            let record_start = lines.get(line_index).ok_or_else(|| ParseError {
                message: "truncated RINEX 3/4 observation record".to_string(),
            })?;
            let satellite_token = parse_satellite_token(record_start.get(..3).unwrap_or_default())?;
            let observation_types = observation_types_for_system(header, satellite_token.system);
            if observation_types.is_empty() {
                return Err(ParseError {
                    message: format!(
                        "missing RINEX 3/4 observation types for {}{:02}",
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
                            "truncated RINEX 3/4 OBS record for satellite {:?}-{:02}",
                            satellite_token.satellite.constellation, satellite_token.satellite.prn
                        ),
                    }
                })?;
            let observations = parse_observation_cells(record_lines, observation_types.len(), 3)?;
            line_index += record_line_count;
            satellites.push(RinexSatelliteObservation {
                system: satellite_token.system,
                satellite: satellite_token.satellite,
                observations,
            });
        }

        records.push(RinexObservationRecord::Epoch(RinexObservationEpoch {
            epoch_time,
            receive_gps_time,
            event_flag,
            receiver_clock_offset_s,
            satellites,
        }));
    }

    Ok(records)
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
    let normalized_year = normalize_rinex_2_year(year);
    let epoch_time =
        RinexObservationEpochTime { year: normalized_year, month, day, hour, minute, second };
    let receive_gps_time =
        rinex_epoch_to_gps_time(normalized_year, month, day, hour, minute, second, time_system)?;
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
        epoch_time,
        receive_gps_time,
        event_flag: flag,
        discontinuity: flag == 1,
        satellites,
        header_line_count,
    })
}

fn parse_rinex_3_epoch_header(
    line: &str,
    time_system: RinexObservationTimeSystem,
) -> Result<(GpsTime, bool, usize), ParseError> {
    let (_epoch_time, receive_gps_time, flag, satellite_count, _receiver_clock_offset_s) =
        parse_rinex_3_epoch_header_raw(line, time_system)?;
    if flag > 1 {
        return Err(ParseError { message: format!("unsupported RINEX 3 epoch flag {flag}") });
    }
    Ok((receive_gps_time, flag == 1, satellite_count))
}

fn parse_rinex_3_epoch_header_raw(
    line: &str,
    time_system: RinexObservationTimeSystem,
) -> Result<(RinexObservationEpochTime, GpsTime, u8, usize, Option<f64>), ParseError> {
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
    let receiver_clock_offset_s =
        fields.get(9).map(|field| parse_rinex_float(field)).transpose()?;
    let epoch_time = RinexObservationEpochTime { year, month, day, hour, minute, second };
    let receive_gps_time =
        rinex_epoch_to_gps_time(year, month, day, hour, minute, second, time_system)?;
    Ok((epoch_time, receive_gps_time, flag, satellite_count, receiver_clock_offset_s))
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
            SignalCode::L5Q,
            signal_spec_gps_l5_q(),
            vec!["C5Q".to_string(), "C5X".to_string(), "C5".to_string()],
            vec!["L5Q".to_string(), "L5X".to_string(), "L5".to_string()],
            vec!["S5Q".to_string(), "S5X".to_string(), "S5".to_string()],
        ),
        (
            SignalBand::L5,
            SignalCode::L5I,
            signal_spec_gps_l5_i(),
            vec!["C5I".to_string()],
            vec!["L5I".to_string()],
            vec!["S5I".to_string()],
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
    Ok(parse_observation_cells(lines, observation_count, prefix_width)?
        .into_iter()
        .map(|cell| cell.value)
        .collect())
}

fn parse_observation_cells(
    lines: &[&str],
    observation_count: usize,
    prefix_width: usize,
) -> Result<Vec<RinexObservationValue>, ParseError> {
    let mut observations = Vec::with_capacity(observation_count);

    for (line_offset, line) in lines.iter().enumerate() {
        let mut field_index = if line_offset == 0 { prefix_width } else { 0usize };
        while field_index < line.len() && observations.len() < observation_count {
            let end = (field_index + OBSERVATION_FIELD_WIDTH).min(line.len());
            let field = line.get(field_index..end).unwrap_or_default();
            let value = field.get(..14).unwrap_or_default().trim();
            let loss_of_lock_indicator = parse_observation_flag(field, 14, "LLI")?;
            let signal_strength_indicator = parse_observation_flag(field, 15, "signal strength")?;
            let value = if value.is_empty() { None } else { Some(parse_rinex_float(value)?) };
            observations.push(RinexObservationValue {
                value,
                loss_of_lock_indicator,
                signal_strength_indicator,
            });
            field_index += OBSERVATION_FIELD_WIDTH;
        }
    }

    while observations.len() < observation_count {
        observations.push(RinexObservationValue::empty());
    }

    Ok(observations)
}

fn parse_observation_flag(
    field: &str,
    index: usize,
    label: &str,
) -> Result<Option<u8>, ParseError> {
    let marker = field.get(index..index + 1).unwrap_or_default().trim();
    if marker.is_empty() {
        return Ok(None);
    }
    let value = marker.parse::<u8>().map_err(|err| ParseError {
        message: format!("invalid RINEX OBS {label} flag '{marker}': {err}"),
    })?;
    if value <= 9 {
        Ok(Some(value))
    } else {
        Err(ParseError { message: format!("invalid RINEX OBS {label} flag {value}") })
    }
}

fn observation_record_line_count(observation_count: usize) -> usize {
    observation_count.div_ceil(OBSERVATIONS_PER_LINE)
}

fn rinex_3_observation_record_line_count(lines: &[&str], observation_count: usize) -> usize {
    let mut line_count = 1usize;
    let max_line_count = rinex_3_observation_record_max_line_count(observation_count);

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

fn rinex_3_observation_record_max_line_count(observation_count: usize) -> usize {
    if observation_count <= 4 {
        1
    } else {
        1 + (observation_count - 4).div_ceil(OBSERVATIONS_PER_LINE)
    }
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
    matches!(line.as_bytes().first(), Some(b'G' | b'R' | b'E' | b'C' | b'S'))
        && parse_satellite_token(line.get(..3).unwrap_or_default()).is_ok()
}

fn rinex_system_from_constellation(constellation: Constellation) -> char {
    match constellation {
        Constellation::Gps => 'G',
        Constellation::Glonass => 'R',
        Constellation::Galileo => 'E',
        Constellation::Beidou => 'C',
        Constellation::Unknown => 'S',
    }
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
#[path = "rinex_obs/tests.rs"]
mod tests;
