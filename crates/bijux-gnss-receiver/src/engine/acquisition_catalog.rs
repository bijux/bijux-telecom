#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, GlonassSlot, SatId};

pub(crate) fn default_acquisition_satellites(constellation: Constellation) -> Vec<SatId> {
    match constellation {
        Constellation::Gps => sat_ids_for_prn_range(constellation, 1..=32),
        Constellation::Galileo => sat_ids_for_prn_range(constellation, 1..=50),
        Constellation::Glonass => {
            sat_ids_for_prn_range(constellation, GlonassSlot::MIN..=GlonassSlot::MAX)
        }
        Constellation::Beidou => sat_ids_for_prn_range(constellation, 1..=37),
        Constellation::Unknown => Vec::new(),
    }
}

fn sat_ids_for_prn_range(
    constellation: Constellation,
    prns: std::ops::RangeInclusive<u8>,
) -> Vec<SatId> {
    prns.map(|prn| SatId { constellation, prn }).collect()
}

#[cfg(test)]
mod tests {
    use super::default_acquisition_satellites;
    use bijux_gnss_core::api::{Constellation, GlonassSlot};

    #[test]
    fn default_acquisition_satellites_match_supported_catalog_bounds() {
        assert_eq!(default_acquisition_satellites(Constellation::Gps).len(), 32);
        assert_eq!(default_acquisition_satellites(Constellation::Galileo).len(), 50);
        assert_eq!(
            default_acquisition_satellites(Constellation::Glonass).len(),
            usize::from(GlonassSlot::MAX - GlonassSlot::MIN + 1)
        );
        assert_eq!(default_acquisition_satellites(Constellation::Beidou).len(), 37);
        assert!(default_acquisition_satellites(Constellation::Unknown).is_empty());
    }
}
