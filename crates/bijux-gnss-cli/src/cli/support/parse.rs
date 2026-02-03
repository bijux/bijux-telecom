
fn parse_ecef(text: &str) -> Result<[f64; 3]> {
    let parts: Vec<&str> = text.split(',').collect();
    if parts.len() != 3 {
        bail!("invalid ECEF format, expected x,y,z");
    }
    Ok([
        parts[0].trim().parse()?,
        parts[1].trim().parse()?,
        parts[2].trim().parse()?,
    ])
}
