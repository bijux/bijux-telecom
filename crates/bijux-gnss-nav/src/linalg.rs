#![allow(missing_docs)]

#[derive(Debug, Clone)]
pub struct Matrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl Matrix {
    pub fn new(rows: usize, cols: usize, value: f64) -> Self {
        Self {
            rows,
            cols,
            data: vec![value; rows * cols],
        }
    }

    pub fn identity(n: usize) -> Self {
        let mut m = Self::new(n, n, 0.0);
        for i in 0..n {
            m[(i, i)] = 1.0;
        }
        m
    }

    pub fn rows(&self) -> usize {
        self.rows
    }

    pub fn cols(&self) -> usize {
        self.cols
    }

    pub fn transpose(&self) -> Self {
        let mut out = Matrix::new(self.cols, self.rows, 0.0);
        for r in 0..self.rows {
            for c in 0..self.cols {
                out[(c, r)] = self[(r, c)];
            }
        }
        out
    }

    pub fn add(&self, rhs: &Self) -> Self {
        let mut out = self.clone();
        for r in 0..self.rows {
            for c in 0..self.cols {
                out[(r, c)] += rhs[(r, c)];
            }
        }
        out
    }

    pub fn sub(&self, rhs: &Self) -> Self {
        let mut out = self.clone();
        for r in 0..self.rows {
            for c in 0..self.cols {
                out[(r, c)] -= rhs[(r, c)];
            }
        }
        out
    }

    pub fn mul(&self, rhs: &Self) -> Self {
        let mut out = Matrix::new(self.rows, rhs.cols, 0.0);
        for r in 0..self.rows {
            for c in 0..rhs.cols {
                let mut sum = 0.0;
                for k in 0..self.cols {
                    sum += self[(r, k)] * rhs[(k, c)];
                }
                out[(r, c)] = sum;
            }
        }
        out
    }

    pub fn invert(&self) -> Option<Self> {
        if self.rows != self.cols {
            return None;
        }
        let n = self.rows;
        let mut a = self.clone();
        let mut inv = Matrix::identity(n);
        for i in 0..n {
            let mut pivot = i;
            let mut max = a[(i, i)].abs();
            for r in (i + 1)..n {
                if a[(r, i)].abs() > max {
                    max = a[(r, i)].abs();
                    pivot = r;
                }
            }
            if max < 1e-12 {
                return None;
            }
            if pivot != i {
                for c in 0..n {
                    a.data.swap(i * n + c, pivot * n + c);
                    inv.data.swap(i * n + c, pivot * n + c);
                }
            }
            let diag = a[(i, i)];
            for c in 0..n {
                a[(i, c)] /= diag;
                inv[(i, c)] /= diag;
            }
            for r in 0..n {
                if r == i {
                    continue;
                }
                let factor = a[(r, i)];
                for c in 0..n {
                    a[(r, c)] -= factor * a[(i, c)];
                    inv[(r, c)] -= factor * inv[(i, c)];
                }
            }
        }
        Some(inv)
    }

    pub fn submatrix(&self, rows: &[usize], cols: &[usize]) -> Self {
        let mut out = Matrix::new(rows.len(), cols.len(), 0.0);
        for (i, &r) in rows.iter().enumerate() {
            for (j, &c) in cols.iter().enumerate() {
                out[(i, j)] = self[(r, c)];
            }
        }
        out
    }

    pub fn cholesky(&self) -> Option<Self> {
        if self.rows != self.cols {
            return None;
        }
        let n = self.rows;
        let mut l = Matrix::new(n, n, 0.0);
        for i in 0..n {
            for j in 0..=i {
                let mut sum = self[(i, j)];
                for k in 0..j {
                    sum -= l[(i, k)] * l[(j, k)];
                }
                if i == j {
                    if sum <= 0.0 {
                        return None;
                    }
                    l[(i, j)] = sum.sqrt();
                } else {
                    if l[(j, j)] == 0.0 {
                        return None;
                    }
                    l[(i, j)] = sum / l[(j, j)];
                }
            }
        }
        Some(l)
    }
}

impl std::ops::Index<(usize, usize)> for Matrix {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.data[index.0 * self.cols + index.1]
    }
}

impl std::ops::IndexMut<(usize, usize)> for Matrix {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.data[index.0 * self.cols + index.1]
    }
}
