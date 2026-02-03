use std::collections::HashMap;

use bijux_gnss_core::{
    AmbiguityId, DoubleDifference, ObsEpoch, ObsSatellite, SignalBand, SingleDifference,
};

fn ambiguity_id_from_sat(sat: &ObsSatellite) -> AmbiguityId {
    AmbiguityId {
        constellation: sat.metadata.signal.constellation,
        prn: sat.prn,
        band: sat.metadata.signal.band,
        signal: match sat.metadata.signal.band {
            SignalBand::L1 => "L1CA".to_string(),
            SignalBand::L2 => "L2".to_string(),
            SignalBand::L5 => "L5".to_string(),
            _ => "Unknown".to_string(),
        },
    }
}

pub fn single_difference(rover: &ObsEpoch, base: &ObsEpoch) -> Vec<SingleDifference> {
    let mut base_map: HashMap<u8, &ObsSatellite> = HashMap::new();
    for sat in &base.sats {
        base_map.insert(sat.prn, sat);
    }
    let mut out = Vec::new();
    for rover_sat in &rover.sats {
        if let Some(base_sat) = base_map.get(&rover_sat.prn) {
            out.push(SingleDifference {
                prn: rover_sat.prn,
                code_m: rover_sat.pseudorange_m - base_sat.pseudorange_m,
                phase_cycles: rover_sat.carrier_phase_cycles - base_sat.carrier_phase_cycles,
                doppler_hz: rover_sat.doppler_hz - base_sat.doppler_hz,
                ambiguity_rover: ambiguity_id_from_sat(rover_sat),
                ambiguity_base: ambiguity_id_from_sat(base_sat),
            });
        }
    }
    out
}

pub fn double_difference(
    single_differences: &[SingleDifference],
    ref_prn: u8,
) -> Vec<DoubleDifference> {
    let mut ref_sd = None;
    for sd in single_differences {
        if sd.prn == ref_prn {
            ref_sd = Some(sd);
            break;
        }
    }
    let Some(ref_sd) = ref_sd else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for sd in single_differences {
        if sd.prn == ref_prn {
            continue;
        }
        out.push(DoubleDifference {
            ref_prn,
            prn: sd.prn,
            code_m: sd.code_m - ref_sd.code_m,
            phase_cycles: sd.phase_cycles - ref_sd.phase_cycles,
            doppler_hz: sd.doppler_hz - ref_sd.doppler_hz,
            canceled: vec![
                ref_sd.ambiguity_rover.clone(),
                ref_sd.ambiguity_base.clone(),
                sd.ambiguity_rover.clone(),
                sd.ambiguity_base.clone(),
            ],
        });
    }
    out
}
