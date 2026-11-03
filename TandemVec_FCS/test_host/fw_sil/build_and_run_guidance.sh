#!/usr/bin/env bash
# ============================================================
# 制导算法联合仿真：真实固件 SIL + UDP 双向桥接
#
# 链接：src/flight_control.cpp（真实固件控制律）+ src/state_data.cpp
# 入口：test_host/fw_sil/fw_sil_guidance_main.cpp（6DOF plant + UDP）
#
# 用法（仓库根目录）：
#   BIN_DIR=T:/fw_sil bash test_host/fw_sil/build_and_run_guidance.sh [rx_port] [tx_port]
# ============================================================
set -eu

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil}"
mkdir -p "$BIN_DIR"

INC="-Itest_host/fw_sil/stub -Iinclude -Isrc -Ilib/ImuAbstraction/src -Itest_host/stub"
INC="$INC -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink"
for d in $(find lib -maxdepth 2 -type d); do
  INC="$INC -I$d"
done

CXX="${CXX:-g++}"
OUT_BASE="$BIN_DIR/fw_sil_guidance"

echo "== 编译制导联合仿真 SIL =="
# 推力/油门按现役线性 ESC 口径，不启用平方曲线宏。
$CXX -std=c++17 -O2 -DTANDEMVEC_FIRMWARE \
  -Wno-address-of-packed-member -Wno-psabi \
  $INC \
  src/flight_control.cpp \
  src/state_data.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp \
  test_host/fw_sil/fw_sil_guidance_main.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
  lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
  -lws2_32 \
  -lwinmm \
  -o "$OUT_BASE"

OUT="$OUT_BASE"
if [ -f "$OUT_BASE.exe" ]; then OUT="$OUT_BASE.exe"; fi

echo "== 运行制导联合仿真（60 s，200 Hz；UDP rx=${1:-7001} tx=${2:-7002}） =="
"$OUT" "${1:-7001}" "${2:-7002}"
