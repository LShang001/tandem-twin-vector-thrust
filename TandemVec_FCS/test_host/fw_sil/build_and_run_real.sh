#!/usr/bin/env bash
# 真实固件 + 真实导航任务 + UBX 解析 + 2 kHz 传感器 SIL
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil_real}"
CXX="${CXX:-g++}"
CC="${CC:-gcc}"
OPT="${OPT:--O2}"
# 宿主 SIL 固定标量 Eigen 路径（理由见 build_incremental.sh）：避免混合核
# 机器上边际 seed 按核出现双吸引子；固件 Cortex-M7 本身无 NEON。
EIGEN_SIL_FLAGS="${EIGEN_SIL_FLAGS:--DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE}"
mkdir -p "$BIN_DIR"

# 真实 ubx.h 必须排在 SIL 的 ubx 桩之前；-DARDUINO 让它使用 SIL 的 Arduino 桩。
INC="-Ilib/ublox-main/src -Itest_host/fw_sil/stub -Iinclude -Isrc -Ilib/ImuAbstraction/src -Itest_host/stub"
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
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
  "$BIN_DIR/minmea.o" \
  -o "$OUT_BASE"

OUT="$OUT_BASE"
[ -f "$OUT_BASE.exe" ] && OUT="$OUT_BASE.exe"
if [ "${1:-}" = "--build-only" ]; then
  echo "built: $OUT"
  exit 0
fi
"$OUT" "$@"
