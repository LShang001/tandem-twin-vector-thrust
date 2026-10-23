#!/usr/bin/env bash
# ============================================================
# 真实固件控制代码的宿主机 SIL 构建/运行
#
# 链接的是：
#   src/flight_control.cpp   （真实固件控制律）
#   src/state_data.cpp       （真实固件全局状态/参数）
#
# 只替换：STM32/Arduino 外设、传感器、调度器、执行器输出。
# 本脚本不是生产固件构建，也不替代 pio run。
# ============================================================
set -eu

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil}"
mkdir -p "$BIN_DIR"

INC="-Itest_host/fw_sil/stub -Iinclude -Isrc -Itest_host/stub"
INC="$INC -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink"
for d in $(find lib -maxdepth 2 -type d); do
  INC="$INC -I$d"
done

CXX="${CXX:-g++}"
OUT_BASE="$BIN_DIR/fw_sil"

echo "== 编译真实固件 SIL =="
$CXX -std=c++17 -O2 -DTANDEMVEC_FIRMWARE \
  -Wno-address-of-packed-member -Wno-psabi \
  $INC \
  src/flight_control.cpp \
  src/state_data.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp \
  test_host/fw_sil/fw_sil_main.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
  lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  -o "$OUT_BASE"

OUT="$OUT_BASE"
if [ -f "$OUT_BASE.exe" ]; then OUT="$OUT_BASE.exe"; fi

echo "== 运行真实固件 SIL（40 s，200 Hz） =="
"$OUT" "$@"
