use super::*;

pub(crate) fn handle_cacode(command: GnssCommand) -> Result<()> {
    let GnssCommand::CaCode {
        prn,
        start_chip,
        count,
        with_reference,
        with_autocorrelation,
        cross_correlation_prn,
    } = command
    else {
        bail!("invalid command for handler");
    };

    let assignment = ca_code_assignment(Prn(prn))
        .map_err(|err| eyre!("failed to load C/A code assignment for PRN {prn}: {err}"))?;
    let autocorrelation = if with_autocorrelation {
        Some(ca_code_autocorrelation_summary(Prn(prn)).map_err(|err| {
            eyre!("failed to compute C/A autocorrelation summary for PRN {prn}: {err}")
        })?)
    } else {
        None
    };
    let cross_correlation = if let Some(other_prn) = cross_correlation_prn {
        Some(
            ca_code_cross_correlation_summary(Prn(prn), Prn(other_prn)).map_err(|err| {
                eyre!(
                    "failed to compute C/A cross-correlation summary for PRNs {prn} and {other_prn}: {err}"
                )
            })?,
        )
    } else {
        None
    };
    let wrapped_start_chip = start_chip % CA_CODE_PERIOD_CHIPS;
    let chip_count = wrapped_start_chip.saturating_add(count);
    let code = generate_ca_code_chips(Prn(prn), chip_count)
        .map_err(|err| eyre!("failed to generate C/A code for PRN {prn}: {err}"))?;

    if with_reference {
        println!("prn: {prn}");
        println!("period_chips: {CA_CODE_PERIOD_CHIPS}");
        println!("start_chip: {start_chip}");
        println!("wrapped_start_chip: {wrapped_start_chip}");
        println!("g2_taps: {} {}", assignment.g2_taps.0, assignment.g2_taps.1);
        println!("g2_delay_chips: {}", assignment.g2_delay_chips);
        println!("first_ten_chips_octal: {:04}", assignment.first_ten_chips_octal);
    }

    if let Some(autocorrelation) = autocorrelation {
        println!("autocorrelation_peak: {}", autocorrelation.peak);
        println!("autocorrelation_max_nonzero_abs: {}", autocorrelation.max_nonzero_abs);
        println!(
            "autocorrelation_nonzero_values: {}",
            autocorrelation
                .unique_nonzero_values
                .iter()
                .map(i16::to_string)
                .collect::<Vec<_>>()
                .join(",")
        );
    }

    if let Some(cross_correlation) = cross_correlation {
        let other_prn = cross_correlation_prn.expect("cross-correlation PRN");
        println!("cross_correlation_base_prn: {prn}");
        println!("cross_correlation_other_prn: {other_prn}");
        println!("cross_correlation_max_abs: {}", cross_correlation.max_abs);
        println!(
            "cross_correlation_values: {}",
            cross_correlation
                .unique_values
                .iter()
                .map(i16::to_string)
                .collect::<Vec<_>>()
                .join(",")
        );
    }

    for chip in code.iter().skip(wrapped_start_chip).take(count) {
        print!("{chip} ");
    }
    println!();

    Ok(())
}
