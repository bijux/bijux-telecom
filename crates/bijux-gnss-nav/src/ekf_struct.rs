#[derive(Debug, Clone)]
pub struct Ekf {
    pub x: Vec<f64>,
    pub p: Matrix,
    pub config: EkfConfig,
    pub health: EkfHealth,
    pub labels: Vec<String>,
}

