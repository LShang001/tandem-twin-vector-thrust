#!/usr/bin/env bash
# ============================================================
# 全链路数字孪生 · 阶段 P4：真实参数页（ano_params）运行期验证
#
# 链接的真实固件源码（一行不改）：
#   src/ano_params.cpp    169 条参数注册表、按名查找、读写与非法值拒绝
#   src/state_data.cpp    全局状态与 kFlightCtrlParams 可变实例
#   src/flight_control.cpp  参数写入后的 applyFlightCtrlParams() 同步链
#
# 用法（仓库根目录）：
#   bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_params.sh
# ============================================================
set -eu

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT"

BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/unignc_fullstack}"
export BIN_DIR

INC="-Itest_host/fw_sil/stub -Iinclude -Isrc -Itest_host/stub"
for d in $(find lib -maxdepth 2 -type d); do
  INC="$INC -I$d"
done
export INC

CXX="${CXX:-g++}"
CXXFLAGS="-std=c++17 -O0 -g -DTANDEMVEC_FIRMWARE -DBFS_REAL_UBX -DARDUINO=10819 \
 -Wno-address-of-packed-member -Wno-psabi \
 -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE \
 -DBFS_TASK_FRACTIONAL_INTERVALS"
export CXXFLAGS

echo "== 编译参数页验证（增量）=="
OUT="$(bash test_host/fw_sil/fullstack/build_incremental.sh fs_params \
  src/state_data.cpp \
  src/ano_params.cpp \
  src/flight_control.cpp \
  lib/ublox-main/src/ubx.cpp \
  lib/ublox-main/src/minmea.c \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
  lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp \
  test_host/fw_sil/fullstack/fs_params_main.cpp)"

echo "== 运行 =="
if command -v timeout >/dev/null 2>&1; then
  timeout 60 "$OUT"
else
  "$OUT"
fi