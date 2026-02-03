
fn parse_sweep(values: &[String]) -> Result<Vec<(String, Vec<String>)>> {
    let mut out = Vec::new();
    for item in values {
        let Some((key, vals)) = item.split_once('=') else {
            bail!("invalid sweep format: {item}");
        };
        let vals: Vec<String> = vals.split(',').map(|v| v.trim().to_string()).collect();
        if vals.is_empty() {
            bail!("sweep values empty for {key}");
        }
        out.push((key.trim().to_string(), vals));
    }
    Ok(out)
}

fn expand_sweep(spec: &[(String, Vec<String>)]) -> Vec<Vec<(String, String)>> {
    fn expand(
        idx: usize,
        spec: &[(String, Vec<String>)],
        current: &mut Vec<(String, String)>,
        out: &mut Vec<Vec<(String, String)>>,
    ) {
        if idx == spec.len() {
            out.push(current.clone());
            return;
        }
        let (key, vals) = &spec[idx];
        for val in vals {
            current.push((key.clone(), val.clone()));
            expand(idx + 1, spec, current, out);
            current.pop();
        }
    }
    let mut out = Vec::new();
    expand(0, spec, &mut Vec::new(), &mut out);
    if out.is_empty() {
        out.push(Vec::new());
    }
    out
}
