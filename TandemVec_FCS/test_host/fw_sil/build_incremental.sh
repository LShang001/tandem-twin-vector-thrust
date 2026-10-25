#!/usr/bin/env bash
# 增量构建：真实固件 + 真实导航任务 + UBX 解析 + 2 kHz 传感器 SIL。
# 与 build_and_run_real.sh 同源、同宏，但按 TU 编译并保留 .o/.d；
# 首次全量，之后只重编改动源文件及其头依赖。用法：
#   BIN_DIR=/tmp/inc bin/bash test_host/fw_sil/build_incremental.sh [--build-only] [SIL args...]
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil_inc}"
OBJ_DIR="$BIN_DIR/obj"
CXX="${CXX:-g++}"
CC="${CC:-gcc}"
OPT="${OPT:--O2}"
mkdir -p "$OBJ_DIR"

# 真实 ubx.h 必须排在 SIL 的 ubx 桩之前；-DARDUINO 让它使用 SIL 的 Arduino 桩。
INC="-Ilib/ublox-main/src -Itest_host/fw_sil/stub -Iinclude -Isrc -Itest_host/stub"
INC="$INC -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink -Ilib/eigen/src"
for d in $(find lib -maxdepth 2 -type d); do INC="$INC -I$d"; done

COMMON_CXX="-std=c++17 $OPT ${EXTRA_CXXFLAGS:-} -DTANDEMVEC_FIRMWARE -DFW_SIL_REAL_NAV -DARDUINO=300 -DBFS_EKF_BARO_ALTITUDE_UPDATE -Wno-address-of-packed-member -Wno-psabi"

obj_name() {
  # 稳定、无目录的对象名
  printf '%s' "$1" | tr '/.\\' '___'
}

is_stale() {
  # $1=src $2=obj $3=dep
  [ ! -f "$2" ] && return 0
  [ "$1" -nt "$2" ] && return 0
  [ -f "$3" ] || return 0
  local deps
  deps=$(sed -e 's/\\$//' -e 's/^[^:]*://' "$3" | tr ' ' '\n' | grep -v '^$' || true)
  local d
  for d in $deps; do
    [ -f "$d" ] || continue
    [ "$d" -nt "$2" ] && return 0
  done
  return 1
}

MINMEA_SRC="lib/ublox-main/src/minmea.c"
MINMEA_OBJ="$OBJ_DIR/$(obj_name "$MINMEA_SRC").o"
if is_stale "$ROOT/$MINMEA_SRC" "$MINMEA_OBJ" "$MINMEA_OBJ.d"; then
  echo "[inc] minmea"
  $CC -std=c11 -O2 -Ilib/ublox-main/src -MMD -MF "$MINMEA_OBJ.d" \
    -c "$ROOT/$MINMEA_SRC" -o "$MINMEA_OBJ"
fi

SOURCES="
src/flight_control.cpp
src/state_data.cpp
src/navigation_task.cpp
lib/ublox-main/src/ubx.cpp
test_host/fw_sil/fw_sil_stubs.cpp
test_host/fw_sil/fw_sil_env_main.cpp
lib/DPS310-Pressure-Sensor/src/Dps310.cpp
lib/DPS310-Pressure-Sensor/src/DpsClass.cpp
lib/units/src/convang.cpp
lib/units/src/convangacc.cpp
lib/units/src/convangvel.cpp
"
OBJS=""
for src in $SOURCES; do
  obj="$OBJ_DIR/$(obj_name "$src").o"
  dep="$obj.d"
  OBJS="$OBJS $obj"
  if is_stale "$ROOT/$src" "$obj" "$dep"; then
    echo "[inc] $src"
    $CXX $COMMON_CXX $INC -MMD -MF "$dep" -c "$ROOT/$src" -o "$obj"
  fi
done

OUT="$BIN_DIR/fw_real"
echo "[inc] link"
$CXX $OPT $OBJS "$MINMEA_OBJ" -o "$OUT"
[ -f "$OUT.exe" ] && OUT="$OUT.exe"
echo "[inc] built: $OUT"
if [ "${1:-}" = "--build-only" ]; then
  exit 0
fi
exec "$OUT" "$@"
