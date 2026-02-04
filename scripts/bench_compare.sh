#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACTS_DIR="${ROOT_DIR}/artifacts"
BASELINE="${ROOT_DIR}/benchmarks/bencher_baseline.txt"
CURRENT="${ROOT_DIR}/benchmarks/bencher_current.txt"
STRICT="${BENCH_STRICT:-0}"

mkdir -p "${ARTIFACTS_DIR}"
mkdir -p "${ROOT_DIR}/benchmarks"

echo "Running benchmarks (bencher output)..."
cargo bench -p bijux-gnss-receiver --bench bench_correlator --bench bench_acquisition_fft --bench bench_tracking_update -- --output-format bencher | tee "${ARTIFACTS_DIR}/benchmarks.txt"
cargo bench -p bijux-gnss-nav --bench bench_ekf_update -- --output-format bencher | tee -a "${ARTIFACTS_DIR}/benchmarks.txt"

rg '^test ' "${ARTIFACTS_DIR}/benchmarks.txt" \
  | sed -E 's/^test ([^ ]+) .* bench: *([0-9,]+) ns\\/iter$/\\1 \\2/' \
  | tr -d ',' \
  | sort \
  > "${CURRENT}"

if [[ ! -f "${BASELINE}" ]]; then
  echo "No benchmark baseline found at ${BASELINE}. Skipping regression check."
  exit 0
fi

echo "Comparing benchmarks to baseline..."
awk -v strict="${STRICT}" '
  NR==FNR { base[$1]=$2; next }
  {
    name=$1; cur=$2; basev=base[name];
    if (basev == "") { next }
    ratio = cur / basev;
    if (ratio > 1.10) {
      printf("Regression: %s current=%s ns baseline=%s ns (%.2fx)\n", name, cur, basev, ratio);
      if (strict == "1") { exit 1 }
    }
  }
' "${BASELINE}" "${CURRENT}"

echo "Benchmark comparison complete."
