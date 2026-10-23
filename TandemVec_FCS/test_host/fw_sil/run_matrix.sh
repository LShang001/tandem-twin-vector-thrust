#!/usr/bin/env bash
# 真实固件 SIL 可复现矩阵：编译一次，跑多种模式/种子
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil}"
CXX="${CXX:-g++}"
mkdir -p "$BIN_DIR"

# 先构建一次（不跑默认场景）
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

run_case() {
  local name="$1"; shift
  echo "---- $name ----"
  "$OUT" "$@" | tail -1
}

for s in 1 2 3; do
  run_case "direct seed$s" --seed "$s"
  run_case "ekf seed$s" --seed "$s" --ekf
  run_case "ekf+flow0 seed$s" --seed "$s" --ekf --flow --flow-delay 0
  run_case "ekf+flow1 seed$s" --seed "$s" --ekf --flow --flow-delay 1
  run_case "ekf+F3 seed$s" --seed "$s" --ekf --fault 3
  run_case "ekf+F4 seed$s" --seed "$s" --ekf --fault 4
done
