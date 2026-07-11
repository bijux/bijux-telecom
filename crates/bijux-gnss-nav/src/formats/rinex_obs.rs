#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::ParseError;

#[derive(Debug, Clone)]
struct RinexObservationHeader {
    version: f64,
    marker_name: Option<String>,
    approx_position_ecef_m: Option<(f64, f64, f64)>,
    interval_s: Option<f64>,
    obs_types_v2: Option<Vec<String>>,
    obs_types_by_system: BTreeMap<char, Vec<String>>,
}

impl RinexObservationHeader {
    fn gps_observation_types(&self) -> Option<&[String]> {
        if self.version >= 3.0 {
            self.obs_types_by_system.get(&'G').map(Vec::as_slice)
        } else {
            self.obs_types_v2.as_deref()
        }
    }
}

fn parse_rinex_observation_header(data: &str) -> Result<(RinexObservationHeader, usize), ParseError> {
    let lines = data.lines().collect::<Vec<_>>();
    let mut header = RinexObservationHeader {
        version: 0.0,
        marker_name: None,
        approx_position_ecef_m: None,
        interval_s: None,
        obs_types_v2: None,
        obs_types_by_system: BTreeMap::new(),
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
                let interval = line
                    .get(..10)
                    .unwrap_or_default()
                    .trim()
                    .parse::<f64>()
                    .map_err(|err| ParseError {
                        message: format!("invalid RINEX OBS interval '{}': {err}", line.trim()),
                    })?;
                header.interval_s = Some(interval);
                line_index += 1;
            }
            "# / TYPES OF OBSERV" => {
                let (obs_types, consumed_lines) = parse_rinex_2_observation_types(&lines[line_index..])?;
                header.obs_types_v2 = Some(obs_types);
                line_index += consumed_lines;
            }
            "SYS / # / OBS TYPES" => {
                let (system, obs_types, consumed_lines) =
                    parse_rinex_3_observation_types(&lines[line_index..])?;
                header.obs_types_by_system.insert(system, obs_types);
                line_index += consumed_lines;
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
        return Err(ParseError {
            message: "invalid or incomplete RINEX OBS header".to_string(),
        });
    }

    Ok((header, line_index))
}

fn rinex_header_label(line: &str) -> &str {
    line.get(60..).unwrap_or_default().trim()
}

fn parse_rinex_version(line: &str) -> Result<f64, ParseError> {
    line.get(..9)
        .unwrap_or_default()
        .trim()
        .parse::<f64>()
        .map_err(|err| ParseError {
            message: format!("invalid RINEX OBS version '{}': {err}", line.trim()),
        })
}

fn parse_rinex_2_observation_types(lines: &[&str]) -> Result<(Vec<String>, usize), ParseError> {
    let first_line = lines.first().ok_or_else(|| ParseError {
        message: "missing RINEX OBS observation types line".to_string(),
    })?;
    let expected_count = first_line
        .get(..6)
        .unwrap_or_default()
        .trim()
        .parse::<usize>()
        .map_err(|err| ParseError {
            message: format!(
                "invalid RINEX 2 OBS type count '{}': {err}",
                first_line.trim()
            ),
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
    let expected_count = first_line
        .get(3..6)
        .unwrap_or_default()
        .trim()
        .parse::<usize>()
        .map_err(|err| ParseError {
            message: format!(
                "invalid RINEX 3 OBS type count '{}': {err}",
                first_line.trim()
            ),
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
        .map_err(|err| ParseError {
            message: format!("invalid {label}: {err}"),
        })?;
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

    use super::parse_rinex_observation_header;

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
        assert_eq!(
            gps_observation_types,
            ["C1", "C2", "C8", "L1", "L2", "L8", "P2"]
        );
    }

    #[test]
    fn parse_rinex_3_observation_type_header() {
        let data = [
            format!(
                "{:<60}{}",
                "     3.04           OBSERVATION DATA    G (GPS)",
                "RINEX VERSION / TYPE"
            ),
            format!(
                "{:<60}{}",
                "G    5 C1C L1C D1C S1C C5Q",
                "SYS / # / OBS TYPES"
            ),
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
}
