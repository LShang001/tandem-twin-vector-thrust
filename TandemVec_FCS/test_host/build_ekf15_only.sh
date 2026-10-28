#!/usr/bin/env bash
# 只编译+运行 15 状态 EKF 宿主回归（迭代 T24 用，避免每次跑全量 run_all）
set -u
ROOT="/d/纵列双发矢量推力飞行器/TandemVec_FCS"
BIN_DIR="${BIN_DIR:-/tmp/host_bin_ekf}"
mkdir -p "$BIN_DIR"
INCLUDES="-I$ROOT/include -I$ROOT/lib/eigen/src"
g++ -std=c++17 -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES \
  $INCLUDES -I$ROOT/src -I$ROOT/lib/navigation-main/src -I$ROOT/lib/units/src \
  "$ROOT/test_host/test_ekf_15state.cpp" \
  "$ROOT/lib/units/src/convang.cpp" \
  "$ROOT/lib/units/src/convangacc.cpp" \
  "$ROOT/lib/units/src/convangvel.cpp" \
  -o "$BIN_DIR/ekf15.exe" || { echo "BUILD FAIL"; exit 2; }
"$BIN_DIR/ekf15.exe"
echo "RC=$?"
