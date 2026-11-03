#!/usr/bin/env bash
# 真实固件 + 6DOF + 传感器/环境 SIL
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil}"
CXX="${CXX:-g++}"
mkdir -p "$BIN_DIR"

INC="-Itest_host/fw_sil/stub -Iinclude -Isrc -Ilib/ImuAbstraction/src -Itest_host/stub"
INC="$INC -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink -Ilib/eigen/src"
for d in $(find lib -maxdepth 2 -type d); do INC="$INC -I$d"; done
# --real-nav 必须用真实 UBX 驱动（不能落在 test_host stub 的简化 ubx.h 上），
# 否则 navigation_task 的历元/计数接口无法匹配；真实头文件置于 stub 之前。
INC="-Ilib/ublox-main/src $INC"

OUT_BASE="$BIN_DIR/fw_env"
$CXX -std=c++17 -O2 -DTANDEMVEC_FIRMWARE -DFW_SIL_REAL_NAV -DARDUINO \
  -Wno-address-of-packed-member -Wno-psabi \
  $INC \
  src/flight_control.cpp \
  src/state_data.cpp \
  src/navigation_task.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp \
  test_host/fw_sil/fw_sil_env_main.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
  lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
  lib/ublox-main/src/ubx.cpp \
  lib/ublox-main/src/minmea.c \
  lib/units/src/convang.cpp lib/units/src/convangacc.cpp lib/units/src/convangvel.cpp \
  -o "$OUT_BASE"

OUT="$OUT_BASE"
[ -f "$OUT_BASE.exe" ] && OUT="$OUT_BASE.exe"
"$OUT" "$@"
