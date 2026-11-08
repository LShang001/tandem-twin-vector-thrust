#!/usr/bin/env bash
# SIL 验收：真实固件 + 真实 EKF 定高闭环（3 seeds）
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil}"
CXX="${CXX:-g++}"
mkdir -p "$BIN_DIR"

INC="-Itest_host/fw_sil/stub -Iinclude -Isrc -Ilib/ImuAbstraction/src -Itest_host/stub"
INC="$INC -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink -Ilib/eigen/src"
for d in $(find lib -maxdepth 2 -type d); do INC="$INC -I$d"; done

OUT_BASE="$BIN_DIR/fw_env"
$CXX -std=c++17 -O2 -DTANDEMVEC_FIRMWARE -DFW_SIL_REAL_NAV -DARDUINO \
  -Wno-address-of-packed-member -Wno-psabi \
  -Ilib/ublox-main/src $INC \
  src/flight_control.cpp src/state_data.cpp src/navigation_task.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp test_host/fw_sil/fw_sil_env_main.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
  lib/ublox-main/src/ubx.cpp lib/ublox-main/src/minmea.c \
  lib/units/src/convang.cpp lib/units/src/convangacc.cpp lib/units/src/convangvel.cpp \
  -o "$OUT_BASE"
OUT="$OUT_BASE"
[ -f "$OUT_BASE.exe" ] && OUT="$OUT_BASE.exe"

ALT_LO="${ALT_LO:-8.0}"
ALT_HI="${ALT_HI:-11.0}"
fail=0
for s in 1 2 3; do
  line="$("$OUT" --seed "$s" --ekf | tail -1)"
  alt="$(echo "$line" | sed -n 's/.*final_alt=\([-0-9.]*\).*/\1/p')"
  echo "seed$s: $line"
  case "$alt" in
    ""|nan|-nan|*[!0-9.-]*) echo "FAIL seed$s: non-numeric altitude"; fail=1 ;;
    *)
      # ★ 2026-11-08：阈值提成变量，提示文案跟着变量走。
      #   此前文案硬编码「8..11」——改判据时提示会撒谎（验证时实测过：
      #   把阈值收紧到 9.99..10.01，报错仍写「outside 8..11 m」）。
      awk -v a="$alt" -v lo="$ALT_LO" -v hi="$ALT_HI" \
        'BEGIN { if (a < lo || a > hi) exit 1 }' \
        || { echo "FAIL seed$s: altitude $alt outside ${ALT_LO}..${ALT_HI} m"; fail=1; }
      ;;
  esac
done

if [ "$fail" -ne 0 ]; then
  echo "SIL acceptance FAILED"
  exit 1
fi
echo "SIL acceptance PASSED (real firmware + real EKF, 3 seeds, ${ALT_LO}..${ALT_HI} m)"
