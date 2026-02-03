use crate::{GpsEphemeris, GpsWord};
use bijux_gnss_core::{Constellation, SatId};

pub(crate) fn parse_subframe1(words: &[GpsWord]) -> Option<EphemerisPart> {
    if words.len() < 10 {
        return None;
    }
    let w3 = words[2].data;
    let w7 = words[6].data;
    let w8 = words[7].data;
    let w9 = words[8].data;
    let w10 = words[9].data;

    let week = get_bits(w3, 1, 10) as u16;
    let iodc_msb = get_bits(w3, 23, 2) as u16;
    let iodc_lsb = get_bits(w8, 1, 8) as u16;
    let iodc = (iodc_msb << 8) | iodc_lsb;
    let tgd = signed(get_bits(w7, 17, 8), 8) as f64 * 2f64.powi(-31);
    let toc = get_bits(w8, 9, 16) as f64 * 16.0;
    let af2 = signed(get_bits(w9, 1, 8), 8) as f64 * 2f64.powi(-55);
    let af1 = signed(get_bits(w9, 9, 16), 16) as f64 * 2f64.powi(-43);
    let af0 = signed(get_bits(w10, 1, 22), 22) as f64 * 2f64.powi(-31);

    Some(EphemerisPart {
        iodc: Some(iodc),
        iode: None,
        week: Some(week),
        toe_s: None,
        toc_s: Some(toc),
        sqrt_a: None,
        e: None,
        i0: None,
        idot: None,
        omega0: None,
        omegadot: None,
        w: None,
        m0: None,
        delta_n: None,
        cuc: None,
        cus: None,
        crc: None,
        crs: None,
        cic: None,
        cis: None,
        af0: Some(af0),
        af1: Some(af1),
        af2: Some(af2),
        tgd: Some(tgd),
    })
}

pub(crate) fn parse_subframe2(words: &[GpsWord]) -> Option<EphemerisPart> {
    if words.len() < 10 {
        return None;
    }
    let w3 = words[2].data;
    let w4 = words[3].data;
    let w5 = words[4].data;
    let w6 = words[5].data;
    let w7 = words[6].data;
    let w8 = words[7].data;
    let w9 = words[8].data;
    let w10 = words[9].data;

    let iode = get_bits(w3, 1, 8) as u8;
    let crs = signed(get_bits(w3, 9, 16), 16) as f64 * 2f64.powi(-5);
    let delta_n = signed(get_bits(w4, 1, 16), 16) as f64 * 2f64.powi(-43) * std::f64::consts::PI;
    let m0 = signed((get_bits(w4, 17, 8) << 24) | get_bits(w5, 1, 24), 32) as f64
        * 2f64.powi(-31)
        * std::f64::consts::PI;
    let cuc = signed(get_bits(w6, 1, 16), 16) as f64 * 2f64.powi(-29);
    let e = (get_bits(w6, 17, 8) << 24 | get_bits(w7, 1, 24)) as f64 * 2f64.powi(-33);
    let cus = signed(get_bits(w8, 1, 16), 16) as f64 * 2f64.powi(-29);
    let sqrt_a = (get_bits(w8, 17, 8) << 24 | get_bits(w9, 1, 24)) as f64 * 2f64.powi(-19);
    let toe = get_bits(w10, 1, 16) as f64 * 16.0;

    Some(EphemerisPart {
        iodc: None,
        iode: Some(iode),
        week: None,
        toe_s: Some(toe),
        toc_s: None,
        sqrt_a: Some(sqrt_a),
        e: Some(e),
        i0: None,
        idot: None,
        omega0: None,
        omegadot: None,
        w: None,
        m0: Some(m0),
        delta_n: Some(delta_n),
        cuc: Some(cuc),
        cus: Some(cus),
        crc: None,
        crs: Some(crs),
        cic: None,
        cis: None,
        af0: None,
        af1: None,
        af2: None,
        tgd: None,
    })
}

pub(crate) fn parse_subframe3(words: &[GpsWord]) -> Option<EphemerisPart> {
    if words.len() < 10 {
        return None;
    }
    let w3 = words[2].data;
    let w4 = words[3].data;
    let w5 = words[4].data;
    let w6 = words[5].data;
    let w7 = words[6].data;
    let w8 = words[7].data;
    let w9 = words[8].data;
    let w10 = words[9].data;

    let cic = signed(get_bits(w3, 1, 16), 16) as f64 * 2f64.powi(-29);
    let omega0 = signed((get_bits(w3, 17, 8) << 24) | get_bits(w4, 1, 24), 32) as f64
        * 2f64.powi(-31)
        * std::f64::consts::PI;
    let cis = signed(get_bits(w5, 1, 16), 16) as f64 * 2f64.powi(-29);
    let i0 = signed((get_bits(w5, 17, 8) << 24) | get_bits(w6, 1, 24), 32) as f64
        * 2f64.powi(-31)
        * std::f64::consts::PI;
    let crc = signed(get_bits(w7, 1, 16), 16) as f64 * 2f64.powi(-5);
    let w = signed((get_bits(w7, 17, 8) << 24) | get_bits(w8, 1, 24), 32) as f64
        * 2f64.powi(-31)
        * std::f64::consts::PI;
    let omegadot = signed(get_bits(w9, 1, 24), 24) as f64 * 2f64.powi(-43) * std::f64::consts::PI;
    let iode = get_bits(w10, 1, 8) as u8;
    let idot = signed(get_bits(w10, 9, 14), 14) as f64 * 2f64.powi(-43) * std::f64::consts::PI;

    Some(EphemerisPart {
        iodc: None,
        iode: Some(iode),
        week: None,
        toe_s: None,
        toc_s: None,
        sqrt_a: None,
        e: None,
        i0: Some(i0),
        idot: Some(idot),
        omega0: Some(omega0),
        omegadot: Some(omegadot),
        w: Some(w),
        m0: None,
        delta_n: None,
        cuc: None,
        cus: None,
        crc: Some(crc),
        crs: None,
        cic: Some(cic),
        cis: Some(cis),
        af0: None,
        af1: None,
        af2: None,
        tgd: None,
    })
}

pub(crate) fn get_bits(data: u32, start: usize, len: usize) -> u32 {
    let shift = 24 - (start - 1) - len;
    (data >> shift) & ((1_u32 << len) - 1)
}

fn signed(value: u32, bits: usize) -> i32 {
    let sign_bit = 1_u32 << (bits - 1);
    if value & sign_bit != 0 {
        (value as i32) - (1_i32 << bits)
    } else {
        value as i32
    }
}

#[derive(Debug, Default, Clone)]
pub(crate) struct EphemerisPart {
    iodc: Option<u16>,
    iode: Option<u8>,
    week: Option<u16>,
    toe_s: Option<f64>,
    toc_s: Option<f64>,
    sqrt_a: Option<f64>,
    e: Option<f64>,
    i0: Option<f64>,
    idot: Option<f64>,
    omega0: Option<f64>,
    omegadot: Option<f64>,
    w: Option<f64>,
    m0: Option<f64>,
    delta_n: Option<f64>,
    cuc: Option<f64>,
    cus: Option<f64>,
    crc: Option<f64>,
    crs: Option<f64>,
    cic: Option<f64>,
    cis: Option<f64>,
    af0: Option<f64>,
    af1: Option<f64>,
    af2: Option<f64>,
    tgd: Option<f64>,
}

#[derive(Debug, Default, Clone)]
pub(crate) struct EphemerisBuilder {
    prn: u8,
    iodc: Option<u16>,
    iode: Option<u8>,
    week: Option<u16>,
    toe_s: Option<f64>,
    toc_s: Option<f64>,
    sqrt_a: Option<f64>,
    e: Option<f64>,
    i0: Option<f64>,
    idot: Option<f64>,
    omega0: Option<f64>,
    omegadot: Option<f64>,
    w: Option<f64>,
    m0: Option<f64>,
    delta_n: Option<f64>,
    cuc: Option<f64>,
    cus: Option<f64>,
    crc: Option<f64>,
    crs: Option<f64>,
    cic: Option<f64>,
    cis: Option<f64>,
    af0: Option<f64>,
    af1: Option<f64>,
    af2: Option<f64>,
    tgd: Option<f64>,
}

impl EphemerisBuilder {
    pub(crate) fn merge(&mut self, part: EphemerisPart) {
        if let Some(value) = part.iodc {
            self.iodc = Some(value);
        }
        if let Some(value) = part.iode {
            self.iode = Some(value);
        }
        if let Some(value) = part.week {
            self.week = Some(value);
        }
        if let Some(value) = part.toe_s {
            self.toe_s = Some(value);
        }
        if let Some(value) = part.toc_s {
            self.toc_s = Some(value);
        }
        if let Some(value) = part.sqrt_a {
            self.sqrt_a = Some(value);
        }
        if let Some(value) = part.e {
            self.e = Some(value);
        }
        if let Some(value) = part.i0 {
            self.i0 = Some(value);
        }
        if let Some(value) = part.idot {
            self.idot = Some(value);
        }
        if let Some(value) = part.omega0 {
            self.omega0 = Some(value);
        }
        if let Some(value) = part.omegadot {
            self.omegadot = Some(value);
        }
        if let Some(value) = part.w {
            self.w = Some(value);
        }
        if let Some(value) = part.m0 {
            self.m0 = Some(value);
        }
        if let Some(value) = part.delta_n {
            self.delta_n = Some(value);
        }
        if let Some(value) = part.cuc {
            self.cuc = Some(value);
        }
        if let Some(value) = part.cus {
            self.cus = Some(value);
        }
        if let Some(value) = part.crc {
            self.crc = Some(value);
        }
        if let Some(value) = part.crs {
            self.crs = Some(value);
        }
        if let Some(value) = part.cic {
            self.cic = Some(value);
        }
        if let Some(value) = part.cis {
            self.cis = Some(value);
        }
        if let Some(value) = part.af0 {
            self.af0 = Some(value);
        }
        if let Some(value) = part.af1 {
            self.af1 = Some(value);
        }
        if let Some(value) = part.af2 {
            self.af2 = Some(value);
        }
        if let Some(value) = part.tgd {
            self.tgd = Some(value);
        }
    }

    pub(crate) fn try_build(&self) -> Option<GpsEphemeris> {
        Some(GpsEphemeris {
            sat: SatId {
                constellation: Constellation::Gps,
                prn: self.prn,
            },
            iodc: self.iodc?,
            iode: self.iode?,
            week: self.week?,
            toe_s: self.toe_s?,
            toc_s: self.toc_s?,
            sqrt_a: self.sqrt_a?,
            e: self.e?,
            i0: self.i0?,
            idot: self.idot?,
            omega0: self.omega0?,
            omegadot: self.omegadot?,
            w: self.w?,
            m0: self.m0?,
            delta_n: self.delta_n?,
            cuc: self.cuc?,
            cus: self.cus?,
            crc: self.crc?,
            crs: self.crs?,
            cic: self.cic?,
            cis: self.cis?,
            af0: self.af0?,
            af1: self.af1?,
            af2: self.af2?,
            tgd: self.tgd?,
        })
    }
}

pub fn decode_subframe_hex(hex: &str) -> Option<Vec<u32>> {
    let bytes = hex::decode(hex).ok()?;
    if bytes.len() != 30 {
        return None;
    }
    let mut bits = Vec::with_capacity(240);
    for byte in bytes {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1);
        }
    }
    let mut words = Vec::new();
    for w in 0..10 {
        let mut data = 0_u32;
        for b in 0..24 {
            let bit = bits[w * 24 + b];
            data = (data << 1) | bit as u32;
        }
        words.push(data);
    }
    Some(words)
}

pub fn decode_rawephem_hex(prn: u8, sub1: &str, sub2: &str, sub3: &str) -> Option<GpsEphemeris> {
    let w1 = decode_subframe_hex(sub1)?;
    let w2 = decode_subframe_hex(sub2)?;
    let w3 = decode_subframe_hex(sub3)?;
    let mut builder = EphemerisBuilder {
        prn,
        ..Default::default()
    };
    let words1: Vec<GpsWord> = w1
        .into_iter()
        .map(|data| GpsWord {
            data,
            parity_ok: true,
            d29_star: 0,
            d30_star: 0,
        })
        .collect();
    let words2: Vec<GpsWord> = w2
        .into_iter()
        .map(|data| GpsWord {
            data,
            parity_ok: true,
            d29_star: 0,
            d30_star: 0,
        })
        .collect();
    let words3: Vec<GpsWord> = w3
        .into_iter()
        .map(|data| GpsWord {
            data,
            parity_ok: true,
            d29_star: 0,
            d30_star: 0,
        })
        .collect();
    builder.merge(parse_subframe1(&words1)?);
    builder.merge(parse_subframe2(&words2)?);
    builder.merge(parse_subframe3(&words3)?);
    builder.try_build()
}
