#!/usr/bin/env bash
# 无 PPS 延时批处理估计：Monte Carlo（可复现）
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_bin}"
CXX="${CXX:-g++}"
OUT="$BIN_DIR/gnss_delay_batch"
CSV="${1:-$ROOT/../TandemVec-EKF-Report/sim-data/gnss-delay-batch-mc-2026-10-22.csv}"
mkdir -p "$BIN_DIR" "$(dirname "$CSV")"
$CXX -std=c++17 -O2 -I"$ROOT/include" -I"$ROOT/lib/eigen/src" \
  "$ROOT/test_host/test_gnss_delay_batch.cpp" -o "$OUT"
BIN="$OUT"
[ -f "$OUT.exe" ] && BIN="$OUT.exe"
echo "true_delay_ms,seed,best_tau_ms,error_ms,cost_m" > "$CSV"
for tau in 0.02 0.05 0.08; do
  for seed in $(seq 1 100); do
    line="$("$BIN" 1 "$seed" "$tau" 1.5 0.196 | grep '^best_tau')"
    best="$(echo "$line" | sed -n 's/.*best_tau=\([0-9.]*\) ms.*/\1/p')"
    err="$(echo "$line" | sed -n 's/.*error \([-0-9.]*\) ms.*/\1/p')"
    cost="$(echo "$line" | sed -n 's/.*cost=\([0-9.]*\) m.*/\1/p')"
    echo "$(awk -v t="$tau" 'BEGIN{printf "%.0f", t*1000}'),$seed,$best,$err,$cost" >> "$CSV"
  done
done
echo "written: $CSV"
