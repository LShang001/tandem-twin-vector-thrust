#!/usr/bin/env bash
# 共轴双桨 VTVL：真实固件 + CoaxialVtvControl + 共轴近似 6DOF plant。
# 与 build_and_run_real.sh 同一套导航/SIL，仅多定义 BFS_COAXIAL_VTVL /
# FW_SIL_COAXIAL；用于验证固件机型分支与输出映射，不是实机/台架证据。
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil_coax}"
CXX="${CXX:-g++}"
CC="${CC:-gcc}"
OPT="${OPT:--O2}"
# 宿主 SIL 固定标量 Eigen 路径（与 build_incremental.sh 同口径）。
EIGEN_SIL_FLAGS="${EIGEN_SIL_FLAGS:--DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE}"
mkdir -p "$BIN_DIR"

INC="-Ilib/ublox-main/src -Itest_host/fw_sil/stub -Iinclude -Isrc -Ilib/ImuAbstraction/src -Itest_host/stub"
INC="$INC -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink -Ilib/eigen/src"
for d in $(find lib -maxdepth 2 -type d); do INC="$INC -I$d"; done

$CC -std=c11 -O2 -Ilib/ublox-main/src -c lib/ublox-main/src/minmea.c -o "$BIN_DIR/minmea.o"

OUT_BASE="$BIN_DIR/fw_coax"
$CXX -std=c++17 $OPT ${EXTRA_CXXFLAGS:-} $EIGEN_SIL_FLAGS \
  -DTANDEMVEC_FIRMWARE -DFW_SIL_REAL_NAV \
  -DFW_SIL_COAXIAL -DBFS_COAXIAL_VTVL -DARDUINO=300 \
  -DBFS_EKF_BARO_ALTITUDE_UPDATE \
  -Wno-address-of-packed-member -Wno-psabi \
  $INC \
  src/flight_control.cpp \
  src/state_data.cpp \
  src/navigation_task.cpp \
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
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
