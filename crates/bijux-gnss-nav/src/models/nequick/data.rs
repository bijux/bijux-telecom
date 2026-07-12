#![allow(dead_code)]

use std::sync::OnceLock;

const MODIP_ROWS: usize = 39;
const MODIP_COLS: usize = 39;
const FOF2_TERM_COUNT: usize = 13 * 76 * 2;
const M3000_TERM_COUNT: usize = 9 * 49 * 2;

#[derive(Debug)]
pub(crate) struct CcirMonthData {
    pub fof2_terms: Vec<f64>,
    pub m3000_terms: Vec<f64>,
}

#[derive(Debug)]
pub(crate) struct SupportData {
    pub modip_grid: [[f64; MODIP_COLS]; MODIP_ROWS],
    pub ccir_months: [CcirMonthData; 12],
}

static SUPPORT_DATA: OnceLock<SupportData> = OnceLock::new();

pub(crate) fn support_data() -> &'static SupportData {
    SUPPORT_DATA.get_or_init(load_support_data)
}

fn load_support_data() -> SupportData {
    SupportData {
        modip_grid: parse_modip_grid(include_str!("data/modip.txt")),
        ccir_months: [
            parse_ccir_month(include_str!("data/ccir11.txt")),
            parse_ccir_month(include_str!("data/ccir12.txt")),
            parse_ccir_month(include_str!("data/ccir13.txt")),
            parse_ccir_month(include_str!("data/ccir14.txt")),
            parse_ccir_month(include_str!("data/ccir15.txt")),
            parse_ccir_month(include_str!("data/ccir16.txt")),
            parse_ccir_month(include_str!("data/ccir17.txt")),
            parse_ccir_month(include_str!("data/ccir18.txt")),
            parse_ccir_month(include_str!("data/ccir19.txt")),
            parse_ccir_month(include_str!("data/ccir20.txt")),
            parse_ccir_month(include_str!("data/ccir21.txt")),
            parse_ccir_month(include_str!("data/ccir22.txt")),
        ],
    }
}

fn parse_modip_grid(contents: &str) -> [[f64; MODIP_COLS]; MODIP_ROWS] {
    let values = parse_f64_values(contents);
    assert_eq!(
        values.len(),
        MODIP_ROWS * MODIP_COLS,
        "unexpected MODIP grid size: {}",
        values.len()
    );
    let mut grid = [[0.0; MODIP_COLS]; MODIP_ROWS];
    for (index, value) in values.into_iter().enumerate() {
        grid[index / MODIP_COLS][index % MODIP_COLS] = value;
    }
    grid
}

fn parse_ccir_month(contents: &str) -> CcirMonthData {
    let values = parse_f64_values(contents);
    assert_eq!(
        values.len(),
        FOF2_TERM_COUNT + M3000_TERM_COUNT,
        "unexpected CCIR term count: {}",
        values.len()
    );
    let (fof2_terms, m3000_terms) = values.split_at(FOF2_TERM_COUNT);
    CcirMonthData { fof2_terms: fof2_terms.to_vec(), m3000_terms: m3000_terms.to_vec() }
}

fn parse_f64_values(contents: &str) -> Vec<f64> {
    contents
        .split_whitespace()
        .map(|value| value.parse::<f64>().expect("support data float"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::{support_data, FOF2_TERM_COUNT, M3000_TERM_COUNT, MODIP_COLS, MODIP_ROWS};

    #[test]
    fn support_data_modip_grid_has_expected_shape() {
        let data = support_data();

        assert_eq!(data.modip_grid.len(), MODIP_ROWS);
        assert!(data.modip_grid.iter().all(|row| row.len() == MODIP_COLS));
        assert!(data.modip_grid[1].iter().all(|value| (*value + 90.0).abs() < 1.0e-12));
        assert!(data.modip_grid[37].iter().all(|value| (*value - 90.0).abs() < 1.0e-12));
        let min_value = data
            .modip_grid
            .iter()
            .flatten()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let max_value = data
            .modip_grid
            .iter()
            .flatten()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        assert_eq!(min_value, -90.0);
        assert_eq!(max_value, 90.0);
    }

    #[test]
    fn support_data_ccir_months_have_expected_term_counts() {
        let data = support_data();

        assert_eq!(data.ccir_months.len(), 12);
        assert!(data
            .ccir_months
            .iter()
            .all(|month| month.fof2_terms.len() == FOF2_TERM_COUNT));
        assert!(data
            .ccir_months
            .iter()
            .all(|month| month.m3000_terms.len() == M3000_TERM_COUNT));
    }
}
