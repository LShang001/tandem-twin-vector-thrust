#!/usr/bin/env bash
# 断链失控保护 + 模式重入/速率环积分 SIL（编译真实固件源码，非重写控制律）
#
# 用途：验证 2026-10-31 审计修复的两组行为：
#   ① CRSF 断链：空中保持解锁 + AUTO_ALTITUDE 定高（P1-2/P1-3）；
#   ② AUTO_POSITION 重入重锁当前点（P1-1）、RATE_MODE 速率环积分不被每拍清零（P1-4）。
#
# 用法（TandemVec_FCS 目录）：
#   BIN_DIR=/c/Users/<user>/AppData/Local/Temp/tv_fcs_sil bash test_host/fw_sil/build_and_run_failsafe.sh
#   BIN_DIR=... bash test_host/fw_sil/build_and_run_failsafe.sh --legacy   # 断链差分对照（预期失败）
#
# 退出码：0 = 全部通过；非零 = 有失败项。
set -eu

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_sil_failsafe}"
CXX="${CXX:-g++}"
OPT="${OPT:--O2}"
EXE_SUFFIX=""
case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*) EXE_SUFFIX=".exe" ;;
esac
mkdir -p "$BIN_DIR"

cd "$ROOT"
INC="-Iinclude -Isrc -Itest_host/fw_sil/stub -Itest_host/stub"
for d in $(find lib -maxdepth 3 -type d); do INC="$INC -I$d"; done

FW_SRC="src/flight_control.cpp src/state_data.cpp test_host/fw_sil/fw_sil_stubs.cpp"
FW_LIBS="lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
         lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
         lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp"
CFLAGS="-std=c++17 $OPT -DTANDEMVEC_FIRMWARE -Wno-address-of-packed-member -Wno-psabi"

echo "---- 编译 fw_failsafe ----"
# shellcheck disable=SC2086
$CXX $CFLAGS $INC $FW_SRC test_host/fw_sil/fw_sil_failsafe_main.cpp $FW_LIBS \
  -o "$BIN_DIR/fw_failsafe$EXE_SUFFIX"
echo "---- 运行 fw_failsafe ----"
"$BIN_DIR/fw_failsafe$EXE_SUFFIX" "$@"

echo "---- 编译 fw_mode ----"
# shellcheck disable=SC2086
$CXX $CFLAGS $INC $FW_SRC test_host/fw_sil/fw_sil_mode_main.cpp $FW_LIBS \
  -o "$BIN_DIR/fw_mode$EXE_SUFFIX"
echo "---- 运行 fw_mode ----"
"$BIN_DIR/fw_mode$EXE_SUFFIX"

echo "=== 断链/模式 SIL 全部通过 ==="
