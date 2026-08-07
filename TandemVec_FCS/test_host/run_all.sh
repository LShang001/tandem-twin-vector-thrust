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
# 输出目录可用环境变量覆盖：MSYS/bash 下 g++ 无法写中文路径（Invalid argument），
# PowerShell 环境请设 BIN_DIR 到英文路径（如 $env:TEMP\tv_fcs_bin）。
BIN_DIR="${BIN_DIR:-$ROOT/test_host/bin}"
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
# ★ 2026-08-08 临时停用：CascadeCtrl 半成品架构已废弃（头文件已删），
#   cascade/sim 两测试待正式删除（回归清理，等并行改动稳定后执行）
# run_one "$ROOT/test_host/test_tandemvec_cascade.cpp"    tc ""
# run_one "$ROOT/test_host/test_tandemvec_sim.cpp"        ts ""

# flight_control.cpp 姿态环闭环仿真（轴序 FRD 修复验证）：
# 按源码公式独立实现，驱动刚体角动力学 + 推进力矩闭环
run_one "$ROOT/test_host/test_flight_control_axis.cpp"  fca ""

# PositionPID v3 回归测试（无 Arduino 依赖，纯平台无关）：
run_one "$ROOT/test_host/test_position_pid.cpp" pp ""

# 在线辨识（RLS + 自适应增益调度），纯平台无关：
run_one "$ROOT/test_host/test_online_id.cpp"            oi ""

# 全阶闭环仿真与灵敏度分析。依赖 PositionPID / ComplementaryFilter
# （后者 include Arduino.h），故需 stub：
run_one "$ROOT/test_host/test_advanced_theory.cpp"      at  "$STUB_INCLUDE"
run_one "$ROOT/test_host/test_robustness.cpp"           rb  "$STUB_INCLUDE"
run_one "$ROOT/test_host/test_qual_analysis.cpp"        qa  "$STUB_INCLUDE"
run_one "$ROOT/test_host/test_comprehensive_sim.cpp"    cs  "$STUB_INCLUDE"

# 15 状态 EKF 宿主回归（ekf_15_state.h 为 header-only，但依赖 bfs::convang 系列
# 与 src/ 下的快速协方差块；需 -DEKF_HOST_REGRESSION 打开测试接口、-O2 控制
# 模板展开段数）：
echo "---- 编译 ekf15 ----"
if ! $CXX -std=$STD -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES \
     $INCLUDES -I$ROOT/src -I$ROOT/lib/navigation-main/src -I$ROOT/lib/units/src \
     "$ROOT/test_host/test_ekf_15state.cpp" \
     "$ROOT/lib/units/src/convang.cpp" \
     "$ROOT/lib/units/src/convangacc.cpp" \
     "$ROOT/lib/units/src/convangvel.cpp" \
     -o "$BIN_DIR/ekf15$EXE_SUFFIX" 2>&1; then
  echo "[BUILD FAIL] ekf15"
  fail=1
else
  echo "---- 运行 ekf15 ----"
  if ! "$BIN_DIR/ekf15$EXE_SUFFIX"; then
    echo "[RUN FAIL] ekf15"
    fail=1
  fi
  echo
fi

if [ "$fail" -eq 0 ]; then
  echo "=== 全部宿主机回归测试通过 ==="
  exit 0
else
  echo "=== 存在失败项 ==="
  exit 1
fi
