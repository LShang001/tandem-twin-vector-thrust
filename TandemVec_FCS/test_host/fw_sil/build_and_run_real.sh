#!/usr/bin/env bash
# 真实固件 + 真实导航任务 + UBX 解析 + 2 kHz 传感器 SIL
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil_real}"
CXX="${CXX:-g++}"
CC="${CC:-gcc}"
OPT="${OPT:--O2}"
mkdir -p "$BIN_DIR"

# 真实 ubx.h 必须排在 SIL 的 ubx 桩之前；-DARDUINO 让它使用 SIL 的 Arduino 桩。
INC="-Ilib/ublox-main/src -Itest_host/fw_sil/stub -Iinclude -Isrc -Itest_host/stub"
INC="$INC -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink -Ilib/eigen/src"
for d in $(find lib -maxdepth 2 -type d); do INC="$INC -I$d"; done

$CC -std=c11 -O2 -Ilib/ublox-main/src -c lib/ublox-main/src/minmea.c -o "$BIN_DIR/minmea.o"

OUT_BASE="$BIN_DIR/fw_real"
$CXX -std=c++17 $OPT ${EXTRA_CXXFLAGS:-} -DTANDEMVEC_FIRMWARE -DFW_SIL_REAL_NAV -DARDUINO=300 \
  -DBFS_EKF_BARO_ALTITUDE_UPDATE \
  -Wno-address-of-packed-member -Wno-psabi \
  $INC \
  src/flight_control.cpp \
  src/state_data.cpp \
  src/navigation_task.cpp \
  lib/ublox-main/src/ubx.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp \
  test_host/fw_sil/fw_sil_env_main.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
  lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  lib/units/src/convang.cpp lib/units/src/convangacc.cpp lib/units/src/convangvel.cpp \
  "$BIN_DIR/minmea.o" \
  -o "$OUT_BASE"

OUT="$OUT_BASE"
[ -f "$OUT_BASE.exe" ] && OUT="$OUT_BASE.exe"
if [ "${1:-}" = "--build-only" ]; then
  echo "built: $OUT"
  exit 0
fi
"$OUT" "$@"
