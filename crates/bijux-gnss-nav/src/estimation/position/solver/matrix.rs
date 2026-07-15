pub fn invert_4x4(a: [[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let mut m = [[0.0_f64; 8]; 4];
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] = a[i][j];
        }
        m[i][i + 4] = 1.0;
    }
    for i in 0..4 {
        let mut pivot = i;
        let mut max = m[i][i].abs();
        for (r, row) in m.iter().enumerate().skip(i + 1) {
            if row[i].abs() > max {
                max = row[i].abs();
                pivot = r;
            }
        }
        if max < 1e-12 {
            return None;
        }
        if pivot != i {
            m.swap(i, pivot);
        }
        let inv_pivot = 1.0 / m[i][i];
        let mut j = i;
        while j < 8 {
            m[i][j] *= inv_pivot;
            j += 1;
        }
        for r in 0..4 {
            if r == i {
                continue;
            }
            let factor = m[r][i];
            let mut j = i;
            while j < 8 {
                m[r][j] -= factor * m[i][j];
                j += 1;
            }
        }
    }
    let mut inv = [[0.0_f64; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            inv[i][j] = m[i][j + 4];
        }
    }
    Some(inv)
}
