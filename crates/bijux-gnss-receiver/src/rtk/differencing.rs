#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Cycles, DoubleDifference, Hertz, Meters, ObsEpoch, SigId, SingleDifference,
};

use super::core::build_sd;

pub fn single_difference(rover: &ObsEpoch, base: &ObsEpoch) -> Vec<SingleDifference> {
    build_sd(base, rover)
        .into_iter()
        .map(|observation| SingleDifference {
            sig: observation.sig,
            code_m: Meters(observation.code_m),
            phase_cycles: Cycles(observation.phase_cycles),
            doppler_hz: Hertz(observation.doppler_hz),
            ambiguity_rover: observation.ambiguity_rover,
            ambiguity_base: observation.ambiguity_base,
        })
        .collect()
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
            phase_cycles: bijux_gnss_core::api::Cycles(sd.phase_cycles.0 - ref_sd.phase_cycles.0),
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
