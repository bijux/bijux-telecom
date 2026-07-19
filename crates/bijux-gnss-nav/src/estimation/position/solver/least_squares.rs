#[derive(Debug, Clone)]
pub(super) struct WeightedLeastSquaresSolution {
    pub(super) delta: Vec<f64>,
    pub(super) covariance: Vec<Vec<f64>>,
    pub(super) rank: usize,
    pub(super) condition_number: Option<f64>,
}

#[derive(Debug, Clone)]
pub(super) struct PivotedQr {
    pub(super) q_columns: Vec<Vec<f64>>,
    pub(super) r: Vec<Vec<f64>>,
    pub(super) pivots: Vec<usize>,
    pub(super) rank: usize,
    pub(super) condition_number: Option<f64>,
}

pub(super) fn solve_weighted_least_squares(
    h: &[Vec<f64>],
    v: &[f64],
    w: &[f64],
) -> Option<WeightedLeastSquaresSolution> {
    let dimension = h.first()?.len();
    if h.len() != v.len() || h.len() < dimension {
        return None;
    }
    let mut weighted_residuals = vec![0.0_f64; h.len()];
    for row_index in 0..h.len() {
        let weight = w.get(row_index).copied().unwrap_or(1.0);
        if !weight.is_finite() || weight < 0.0 || !v[row_index].is_finite() {
            return None;
        }
        weighted_residuals[row_index] = v[row_index] * weight.sqrt();
    }

    let decomposition = decompose_weighted_design(h, w)?;
    if decomposition.rank < dimension {
        return None;
    }
    let pivoted_delta = solve_qr_coordinates(&decomposition, &weighted_residuals)?;
    let delta = unpivot_vector(&pivoted_delta, &decomposition.pivots);
    let covariance = covariance_from_upper_triangular(&decomposition.r, &decomposition.pivots)?;

    Some(WeightedLeastSquaresSolution {
        delta,
        covariance,
        rank: decomposition.rank,
        condition_number: decomposition.condition_number,
    })
}

pub(super) fn decompose_weighted_design(h: &[Vec<f64>], w: &[f64]) -> Option<PivotedQr> {
    let dimension = h.first()?.len();
    if h.len() < dimension {
        return None;
    }
    let mut weighted_design = vec![vec![0.0_f64; dimension]; h.len()];
    for (row_index, row) in h.iter().enumerate() {
        if row.len() != dimension {
            return None;
        }
        let weight = w.get(row_index).copied().unwrap_or(1.0);
        if !weight.is_finite() || weight < 0.0 {
            return None;
        }
        let scale = weight.sqrt();
        for (col, value) in row.iter().enumerate().take(dimension) {
            if !value.is_finite() {
                return None;
            }
            weighted_design[row_index][col] = value * scale;
        }
    }
    pivoted_qr(&weighted_design)
}

fn pivoted_qr(a: &[Vec<f64>]) -> Option<PivotedQr> {
    let row_count = a.len();
    let dimension = a.first()?.len();
    if dimension == 0 || a.iter().any(|row| row.len() != dimension) {
        return None;
    }
    let mut columns = vec![vec![0.0_f64; row_count]; dimension];
    for (row_index, row) in a.iter().enumerate() {
        for (col, value) in row.iter().enumerate().take(dimension) {
            columns[col][row_index] = *value;
        }
    }
    let mut pivots = (0..dimension).collect::<Vec<_>>();
    let mut q_columns = vec![vec![0.0_f64; row_count]; dimension];
    let mut r = vec![vec![0.0_f64; dimension]; dimension];
    let mut column_norms = columns.iter().map(|column| dot(column, column)).collect::<Vec<_>>();
    let max_initial_norm = column_norms.iter().copied().fold(0.0_f64, f64::max).sqrt();
    if !max_initial_norm.is_finite() || max_initial_norm <= 0.0 {
        return None;
    }
    let rank_tolerance = (row_count.max(dimension) as f64) * max_initial_norm * 1.0e-12;
    let mut rank = 0;
    let mut max_diagonal = 0.0_f64;
    let mut min_diagonal = f64::INFINITY;

    for col in 0..dimension {
        let pivot_col = (col..dimension).max_by(|left, right| {
            column_norms[*left]
                .partial_cmp(&column_norms[*right])
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        if pivot_col != col {
            columns.swap(col, pivot_col);
            column_norms.swap(col, pivot_col);
            pivots.swap(col, pivot_col);
            for row in r.iter_mut().take(col) {
                row.swap(col, pivot_col);
            }
        }

        let mut vector = columns[col].clone();
        for previous_col in 0..col {
            let projection = dot(&q_columns[previous_col], &vector);
            r[previous_col][col] += projection;
            axpy(&mut vector, &q_columns[previous_col], -projection);
        }
        for previous_col in 0..col {
            let projection = dot(&q_columns[previous_col], &vector);
            r[previous_col][col] += projection;
            axpy(&mut vector, &q_columns[previous_col], -projection);
        }
        let diagonal = dot(&vector, &vector).sqrt();
        r[col][col] = diagonal;
        if diagonal <= rank_tolerance || !diagonal.is_finite() {
            break;
        }
        max_diagonal = max_diagonal.max(diagonal);
        min_diagonal = min_diagonal.min(diagonal);
        for (row, value) in vector.iter().enumerate().take(row_count) {
            q_columns[col][row] = value / diagonal;
        }
        rank += 1;
        for trailing_col in (col + 1)..dimension {
            column_norms[trailing_col] = dot(&columns[trailing_col], &columns[trailing_col]);
        }
    }

    let condition_number = (rank == dimension && min_diagonal.is_finite() && min_diagonal > 0.0)
        .then_some(max_diagonal / min_diagonal);
    Some(PivotedQr { q_columns, r, pivots, rank, condition_number })
}

fn solve_qr_coordinates(qr: &PivotedQr, b: &[f64]) -> Option<Vec<f64>> {
    let dimension = qr.r.len();
    if qr.rank < dimension || qr.q_columns.iter().any(|column| column.len() != b.len()) {
        return None;
    }
    let mut qtb = vec![0.0_f64; dimension];
    for (col, qtb_value) in qtb.iter_mut().enumerate().take(dimension) {
        *qtb_value = dot(&qr.q_columns[col], b);
    }
    solve_upper_triangular(&qr.r, &qtb)
}

fn solve_upper_triangular(r: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let dimension = r.len();
    if dimension == 0 || b.len() != dimension || r.iter().any(|row| row.len() != dimension) {
        return None;
    }
    let mut x = vec![0.0_f64; dimension];
    for row in (0..dimension).rev() {
        let diagonal = r[row][row];
        if !diagonal.is_finite() || diagonal.abs() <= 1.0e-12 {
            return None;
        }
        let mut sum = b[row];
        for (col, x_value) in x.iter().enumerate().take(dimension).skip(row + 1) {
            sum -= r[row][col] * x_value;
        }
        x[row] = sum / diagonal;
    }
    Some(x)
}

pub(super) fn covariance_from_upper_triangular(
    r: &[Vec<f64>],
    pivots: &[usize],
) -> Option<Vec<Vec<f64>>> {
    let dimension = r.len();
    if pivots.len() != dimension || r.iter().any(|row| row.len() != dimension) {
        return None;
    }
    let mut inverse_r = vec![vec![0.0_f64; dimension]; dimension];
    for basis_col in 0..dimension {
        let mut basis = vec![0.0_f64; dimension];
        basis[basis_col] = 1.0;
        let solution = solve_upper_triangular(r, &basis)?;
        for (row, solution_value) in solution.iter().enumerate().take(dimension) {
            inverse_r[row][basis_col] = *solution_value;
        }
    }
    let mut covariance_pivoted = vec![vec![0.0_f64; dimension]; dimension];
    for row in 0..dimension {
        for col in 0..dimension {
            covariance_pivoted[row][col] =
                (0..dimension).map(|k| inverse_r[row][k] * inverse_r[col][k]).sum();
        }
    }
    let mut covariance = vec![vec![0.0_f64; dimension]; dimension];
    for pivoted_row in 0..dimension {
        for pivoted_col in 0..dimension {
            covariance[pivots[pivoted_row]][pivots[pivoted_col]] =
                covariance_pivoted[pivoted_row][pivoted_col];
        }
    }
    Some(covariance)
}

fn unpivot_vector(vector: &[f64], pivots: &[usize]) -> Vec<f64> {
    let mut out = vec![0.0_f64; vector.len()];
    for (pivoted_index, original_index) in pivots.iter().copied().enumerate() {
        out[original_index] = vector[pivoted_index];
    }
    out
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter().zip(right).map(|(left, right)| left * right).sum()
}

fn axpy(out: &mut [f64], x: &[f64], alpha: f64) {
    for (out, x) in out.iter_mut().zip(x) {
        *out += alpha * x;
    }
}
