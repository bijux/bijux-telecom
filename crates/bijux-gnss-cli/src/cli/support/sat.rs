fn format_sat(sat: SatId) -> String {
    format!("{:?}-{}", sat.constellation, sat.prn)
}

fn prns_to_sats(prns: &[u8]) -> Vec<SatId> {
    prns.iter()
        .map(|&prn| SatId {
            constellation: Constellation::Gps,
            prn,
        })
        .collect()
}
