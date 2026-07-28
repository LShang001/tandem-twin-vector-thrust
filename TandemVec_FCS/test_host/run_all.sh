#!/usr/bin/env bash
# 宿主机平台无关算法回归测试运行器
# 用途：在不依赖 STM32 硬件的前提下，用 g++ 编译运行 include/ 下三个平台无关算法的回归测试。
# 对应 AGENTS.md「修改这三个文件后，可以也应该在宿主机上用 g++ 单独验证」。
#
# 用法（在仓库根目录）：
#   bash test_host/run_all.sh
# 退出码：0 表示全部通过，非 0 表示有失败项。
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN_DIR="$ROOT/test_host/bin"
mkdir -p "$BIN_DIR"

INCLUDES="-I$ROOT/include -I$ROOT/lib/eigen/src"
STUB_INCLUDE="-I$ROOT/test_host/stub"
CXX=${CXX:-g++}
STD=${STD:-c++17}
EXE_SUFFIX=""
case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*) EXE_SUFFIX=".exe" ;;
esac

fail=0

run_one() {
  local src="$1" name="$2" extra_includes="$3"
  local out="$BIN_DIR/$name$EXE_SUFFIX"
  echo "---- 编译 $name ----"
  if ! $CXX -std=$STD $INCLUDES $extra_includes "$src" -o "$out" 2>&1; then
    echo "[BUILD FAIL] $name"
    fail=1
    return
  fi
  echo "---- 运行 $name ----"
  if ! "$out"; then
    echo "[RUN FAIL] $name"
    fail=1
  fi
  echo
}

# 平台无关算法（include/ins_*.h，不依赖 Arduino）：
run_one "$ROOT/test_host/test_gnss_dynamic_weight.cpp" gdw ""
run_one "$ROOT/test_host/test_static_aid_profile.cpp"  sap ""
run_one "$ROOT/test_host/test_static_detector.cpp"     sd  ""
run_one "$ROOT/test_host/test_altitude_reference.cpp"  ar  ""
run_one "$ROOT/test_host/test_gnss_epoch_timing.cpp"    get ""

# 平台相关算法（依赖 Arduino.h，通过 test_host/stub/Arduino.h 桩在宿主机编译）：
run_one "$ROOT/test_host/test_vertical_kf.cpp"         vkf "$STUB_INCLUDE"

# 纵列双发矢量推力飞行器控制分配（纯平台无关头文件，无 Arduino 依赖）：
run_one "$ROOT/test_host/test_tandemvec_allocation.cpp" ta ""
run_one "$ROOT/test_host/test_tandemvec_cascade.cpp"    tc ""
run_one "$ROOT/test_host/test_tandemvec_sim.cpp"        ts ""

if [ "$fail" -eq 0 ]; then
  echo "=== 全部宿主机回归测试通过 ==="
  exit 0
else
  echo "=== 存在失败项 ==="
  exit 1
fi
