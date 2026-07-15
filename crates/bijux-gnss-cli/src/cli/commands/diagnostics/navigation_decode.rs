pub(super) fn handle_nav(command: GnssCommand) -> Result<()> {
    let GnssCommand::Nav { command } = command else {
        bail!("invalid command for handler");
    };
    let common = match &command {
        NavCommand::Decode { common, .. } => common,
    };
    let profile = load_config(common)?;
    let dataset = load_dataset(common)?;

    match command {
        NavCommand::Decode { common, track, prn, reference_week } => {
            let _ = runtime_config_from_env(&common, None);
            let rows = read_tracking_dump(&track)?;
            let target = SatId { constellation: Constellation::Gps, prn };
            let reference_week = nav_reference_week(reference_week, dataset.as_ref())?;
            let mut sorted: Vec<_> = rows.into_iter().filter(|r| r.sat == target).collect();
            sorted.sort_by_key(|r| r.epoch_idx);
            let prompt = sorted.iter().map(|row| row.prompt_i).collect::<Vec<_>>();
            let demodulation =
                bijux_gnss_infra::api::nav::demodulate_gps_l1ca_navigation_bits(&prompt);
            let decoded_stream =
                bijux_gnss_infra::api::nav::decode_gps_l1ca_lnav_from_prompt(&prompt);
            let parity_word_count = decoded_stream
                .subframes
                .iter()
                .map(|subframe| subframe.parity.word_count)
                .sum::<usize>();
            let parity_failed_words = decoded_stream
                .subframes
                .iter()
                .map(|subframe| subframe.parity.failed_word_indexes.len())
                .sum::<usize>();
            let bit_signs = demodulation.bits.iter().map(|bit| bit.sign).collect::<Vec<_>>();
            let (mut ephs, stats) =
                bijux_gnss_infra::api::nav::decode_subframes(&bit_signs, reference_week);
            for eph in ephs.iter_mut() {
                eph.sat = target;
            }
            let report = NavDecodeReport {
                sat: target,
                reference_week,
                bit_start_ms: demodulation.bit_start_ms,
                bit_signs,
                aligned_subframes: decoded_stream
                    .subframes
                    .iter()
                    .map(|subframe| subframe.alignment.clone())
                    .collect(),
                decoded_subframes: decoded_stream.subframes,
                ephemeris_rejections: stats.ephemeris_rejections.clone(),
                parity_word_count,
                parity_failed_words,
                preamble_hits: stats.preamble_hits,
                parity_pass_rate: stats.parity_pass_rate,
                ephemerides: ephs.clone(),
            };
            emit_report(&common, "nav_decode", &report)?;
            write_ephemeris(&common, &ephs, &profile, dataset.as_ref())?;
            write_manifest(&common, "nav_decode", &profile, dataset.as_ref(), &report)?;
        }
    }

    Ok(())
}

fn nav_reference_week(
    explicit_reference_week: Option<u32>,
    dataset: Option<&bijux_gnss_infra::api::DatasetEntry>,
) -> Result<Option<u32>> {
    if let Some(reference_week) = explicit_reference_week {
        return Ok(Some(reference_week));
    }

    let Some(capture_start_utc) = dataset.and_then(|entry| entry.capture_start_utc.as_deref())
    else {
        return Ok(None);
    };

    Ok(Some(reference_week_from_capture_start_utc(capture_start_utc)?))
}

fn reference_week_from_capture_start_utc(capture_start_utc: &str) -> Result<u32> {
    let utc = time::OffsetDateTime::parse(
        capture_start_utc,
        &time::format_description::well_known::Rfc3339,
    )
    .map_err(|err| eyre!("failed to parse capture_start_utc {capture_start_utc:?}: {err}"))?;
    let utc = bijux_gnss_infra::api::core::UtcTime {
        unix_s: utc.unix_timestamp_nanos() as f64 / 1_000_000_000.0,
    };
    Ok(bijux_gnss_infra::api::nav::gps_time_from_utc(utc).week)
}
