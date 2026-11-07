#!/usr/bin/env bash
# ============================================================
# 单涵道四燃气舵：真实固件控制代码的宿主机 SIL 构建/运行
#
# 链接的是：
#   src/flight_control.cpp   （真实固件控制律，走 BFS_SINGLE_EDF_4VANE 分支）
#   src/state_data.cpp       （真实固件全局状态/参数）
#
# 只替换：STM32/Arduino 外设、传感器、调度器、执行器输出。
# plant 为 single_edf_plant.h（1 路涵道推力 + 4 片燃气舵力矩，RK4 6DOF）。
# 本脚本不是生产固件构建，也不替代 pio run。
#
# 用法（TandemVec_FCS 目录）：
#   bash test_host/fw_sil/build_and_run_singleedf.sh            # 定高 10 m
#   bash test_host/fw_sil/build_and_run_singleedf.sh --manual   # roll/pitch 阶跃
# ============================================================
set -eu

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil_sedf}"
mkdir -p "$BIN_DIR"

INC="-Itest_host/fw_sil/stub -Iinclude -Isrc -Ilib/ImuAbstraction/src -Itest_host/stub"
INC="$INC -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink"
for d in $(find lib -maxdepth 2 -type d); do
  INC="$INC -I$d"
done

CXX="${CXX:-g++}"
OUT_BASE="$BIN_DIR/fw_sedf"

echo "== 编译真实固件 SIL（单涵道） =="
$CXX -std=c++17 -O2 -DTANDEMVEC_FIRMWARE -DBFS_SINGLE_EDF_4VANE \
  -Wno-address-of-packed-member -Wno-psabi \
  $INC \
  src/flight_control.cpp \
  src/state_data.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp \
  test_host/fw_sil/fw_sil_singleedf_main.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
  lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
  -o "$OUT_BASE"

OUT="$OUT_BASE"
if [ -f "$OUT_BASE.exe" ]; then OUT="$OUT_BASE.exe"; fi

echo "== 运行真实固件 SIL（单涵道，40 s，200 Hz） =="
"$OUT" "$@"
