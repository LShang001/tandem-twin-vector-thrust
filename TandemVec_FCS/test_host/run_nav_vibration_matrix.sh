#!/usr/bin/env bash
# 卫惯组合导航：振动 × 机动 矩阵（可复现）
# 输出 CSV：mode,vib_g,vib_mode,gyro_vib_dps,seed,pos3d_m,vel_mps,improve_pct
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_bin}"
CXX="${CXX:-g++}"
OUT="$BIN_DIR/ekf_nav_matrix"
mkdir -p "$BIN_DIR"
OUT_CSV="${1:-$ROOT/../TandemVec-EKF-Report/sim-data/nav-vibration-maneuver-2026-10-22.csv}"

INC="-I$ROOT/include -I$ROOT/lib/eigen/src -I$ROOT/src -I$ROOT/lib/navigation-main/src -I$ROOT/lib/units/src"
$CXX -std=c++17 -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES $INC \
  "$ROOT/test_host/test_ekf_highspeed_realistic.cpp" \
  "$ROOT/lib/units/src/convang.cpp" \
  "$ROOT/lib/units/src/convangacc.cpp" \
  "$ROOT/lib/units/src/convangvel.cpp" \
  -o "$OUT"
BIN="$OUT"
[ -f "$OUT.exe" ] && BIN="$OUT.exe"

mkdir -p "$(dirname "$OUT_CSV")"
echo "mode,vib_g,vib_mode,gyro_vib_dps,seed,pos3d_m,vel_mps,improve_pct" > "$OUT_CSV"

for mode in 0 1 2; do
  for vib in 0 0.1 0.3 0.5 1.0 2.0; do
    for seed in 1 2 3; do
      line="$("$BIN" 30 0.05 0.25 0.0015 0.00524 1.5 0.05 "$seed" "$mode" 0.01 "$vib" 0 0 0.5 20 1e9 1e9 | tail -1)"
      pos="$(echo "$line" | sed -n 's/.*pos3d=\([0-9.]*\).*/\1/p')"
      vel="$(echo "$line" | sed -n 's/.*vel=\([0-9.]*\).*/\1/p')"
      imp="$(echo "$line" | sed -n 's/.*improve=\([-0-9.]*\)%.*/\1/p')"
      echo "$mode,$vib,0,0.5,$seed,$pos,$vel,$imp" >> "$OUT_CSV"
    done
  done
done

echo "written: $OUT_CSV"
