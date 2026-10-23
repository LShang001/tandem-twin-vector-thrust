#!/usr/bin/env bash
# 卫惯组合导航验收：延时补偿 / 杆臂补偿 / 振动鲁棒性
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_bin}"
CXX="${CXX:-g++}"
OUT="$BIN_DIR/nav_accept"
mkdir -p "$BIN_DIR"

INC="-I$ROOT/include -I$ROOT/lib/eigen/src -I$ROOT/src -I$ROOT/lib/navigation-main/src -I$ROOT/lib/units/src"
$CXX -std=c++17 -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES $INC \
  "$ROOT/test_host/test_ekf_highspeed_realistic.cpp" \
  "$ROOT/lib/units/src/convang.cpp" \
  "$ROOT/lib/units/src/convangacc.cpp" \
  "$ROOT/lib/units/src/convangvel.cpp" \
  -o "$OUT"
BIN="$OUT"
[ -f "$OUT.exe" ] && BIN="$OUT.exe"

fail=0
run_check() {
  local name="$1" limit="$2"; shift 2
  local line pos3d
  line="$("$BIN" "$@" | tail -1)"
  pos3d="$(echo "$line" | sed -n 's/.*pos3d=\([0-9.]*\).*/\1/p')"
  echo "$name: $line"
  if [ -z "$pos3d" ] || ! awk -v p="$pos3d" -v l="$limit" 'BEGIN { exit !(p <= l) }'; then
    echo "  FAIL: pos3d=${pos3d:-?} > ${limit}"
    fail=1
  else
    echo "  PASS: pos3d=${pos3d} <= ${limit}"
  fi
}

# 参数顺序：
# speed latency accel_std gyro_std gyro_markov gnss_pos gnss_vel seed maneuver
# accel_markov vib_g adaptive vib_mode gyro_vib_dps vib_hz outage_start outage_end lever_m lever_comp jitter
run_check "delay-comp nominal" 0.30 \
  30 0.05 0.25 0.0015 0.00524 1.5 0.05 1 0 0.01 0 0 0 0.5 20 1e9 1e9 0 0 0
run_check "yaw + lever comp" 0.20 \
  30 0.05 0.25 0.0015 0.00524 1.5 0.05 1 3 0.01 0 0 0 0.5 20 1e9 1e9 1.0 1 0
run_check "vibration 0.5g" 1.10 \
  30 0.05 0.25 0.0015 0.00524 1.5 0.05 1 0 0.01 0.5 0 0 0.5 20 1e9 1e9 0 0 0
run_check "vibration 2g" 5.00 \
  30 0.05 0.25 0.0015 0.00524 1.5 0.05 1 0 0.01 2.0 0 0 0.5 20 1e9 1e9 0 0 0

if [ "$fail" -ne 0 ]; then
  echo "NAV acceptance FAILED"
  exit 1
fi
echo "NAV acceptance PASSED (delay/lever/vibration)"
