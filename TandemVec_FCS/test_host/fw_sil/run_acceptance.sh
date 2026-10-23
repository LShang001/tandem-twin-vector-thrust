#!/usr/bin/env bash
# SIL 验收：真实固件 + 真实 EKF 定高闭环（3 seeds）
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil}"
CXX="${CXX:-g++}"
mkdir -p "$BIN_DIR"

INC="-Itest_host/fw_sil/stub -Iinclude -Isrc -Itest_host/stub"
INC="$INC -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink -Ilib/eigen/src"
for d in $(find lib -maxdepth 2 -type d); do INC="$INC -I$d"; done

OUT_BASE="$BIN_DIR/fw_env"
$CXX -std=c++17 -O2 -DTANDEMVEC_FIRMWARE \
  -Wno-address-of-packed-member -Wno-psabi \
  $INC \
  src/flight_control.cpp src/state_data.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp test_host/fw_sil/fw_sil_env_main.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  lib/units/src/convang.cpp lib/units/src/convangacc.cpp lib/units/src/convangvel.cpp \
  -o "$OUT_BASE"
OUT="$OUT_BASE"
[ -f "$OUT_BASE.exe" ] && OUT="$OUT_BASE.exe"

fail=0
for s in 1 2 3; do
  line="$("$OUT" --seed "$s" --ekf | tail -1)"
  alt="$(echo "$line" | sed -n 's/.*final_alt=\([-0-9.]*\).*/\1/p')"
  echo "seed$s: $line"
  case "$alt" in
    ""|nan|-nan|*[!0-9.-]*) echo "FAIL seed$s: non-numeric altitude"; fail=1 ;;
    *)
      awk -v a="$alt" 'BEGIN { if (a < 8.0 || a > 11.0) exit 1 }' \
        || { echo "FAIL seed$s: altitude $alt outside 8..11 m"; fail=1; }
      ;;
  esac
done

if [ "$fail" -ne 0 ]; then
  echo "SIL acceptance FAILED"
  exit 1
fi
echo "SIL acceptance PASSED (real firmware + real EKF, 3 seeds, 8..11 m)"
