#![allow(missing_docs)]

use std::collections::HashMap;

use bijux_gnss_core::{
    AmbiguityId, DoubleDifference, Hertz, Meters, ObsEpoch, ObsSatellite, SigId, SignalBand,
    SingleDifference,
};

fn ambiguity_id_from_sat(sat: &ObsSatellite) -> AmbiguityId {
    AmbiguityId {
        sig: sat.signal_id,
        signal: match sat.metadata.signal.band {
            SignalBand::L1 => "L1CA".to_string(),
            SignalBand::L2 => "L2".to_string(),
            SignalBand::L5 => "L5".to_string(),
            _ => "Unknown".to_string(),
        },
    }
}

pub fn single_difference(rover: &ObsEpoch, base: &ObsEpoch) -> Vec<SingleDifference> {
    let mut base_map: HashMap<bijux_gnss_core::SatId, &ObsSatellite> = HashMap::new();
    for sat in &base.sats {
        base_map.insert(sat.signal_id.sat, sat);
    }
    let mut out = Vec::new();
    for rover_sat in &rover.sats {
        if let Some(base_sat) = base_map.get(&rover_sat.signal_id.sat) {
            out.push(SingleDifference {
                sig: rover_sat.signal_id,
                code_m: Meters(rover_sat.pseudorange_m.0 - base_sat.pseudorange_m.0),
                phase_cycles: bijux_gnss_core::Cycles(
                    rover_sat.carrier_phase_cycles.0 - base_sat.carrier_phase_cycles.0,
                ),
                doppler_hz: Hertz(rover_sat.doppler_hz.0 - base_sat.doppler_hz.0),
                ambiguity_rover: ambiguity_id_from_sat(rover_sat),
                ambiguity_base: ambiguity_id_from_sat(base_sat),
            });
        }
    }
    out
}

pub fn double_difference(
    single_differences: &[SingleDifference],
    ref_sig: SigId,
) -> Vec<DoubleDifference> {
    let mut ref_sd = None;
    for sd in single_differences {
        if sd.sig == ref_sig {
            ref_sd = Some(sd);
            break;
        }
    }
    let Some(ref_sd) = ref_sd else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for sd in single_differences {
        if sd.sig == ref_sig {
            continue;
        }
        out.push(DoubleDifference {
            ref_sig: ref_sd.sig,
            sig: sd.sig,
            code_m: Meters(sd.code_m.0 - ref_sd.code_m.0),
            phase_cycles: bijux_gnss_core::Cycles(sd.phase_cycles.0 - ref_sd.phase_cycles.0),
            doppler_hz: Hertz(sd.doppler_hz.0 - ref_sd.doppler_hz.0),
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
