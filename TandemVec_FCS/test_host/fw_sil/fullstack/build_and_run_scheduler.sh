#!/usr/bin/env bash
# ============================================================
# 全链路数字孪生 · 阶段 P1：真实任务调度器上宿主
#
# 链接的真实固件源码（一行不改）：
#   src/task_scheduler.cpp   setupTimer / timerCallback / addTaskNamed / taskExecutor / 统计
#   src/state_data.cpp       全局状态、参数实例、TaskTimer 构造
#
# 替换的唯一一层：谁按 156.25 µs 触发 timerCallback。
#   固件 = TIM8 硬件溢出中断；此处 = 主程序按仿真时间显式调用。
#   因此本目标验证调度算法本体，不代表中断延迟与抖动量级。
#
# 用法（仓库根目录）：
#   bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_scheduler.sh
#   BIN_DIR=<ascii 路径> bash .../build_and_run_scheduler.sh   # 中文路径下 MinGW ld 需要
# ============================================================
set -eu

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT"

BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/unignc_fullstack}"
export BIN_DIR
mkdir -p "$BIN_DIR"

INC="-Itest_host/fw_sil/stub -Iinclude -Isrc -Itest_host/stub"
INC="$INC -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink"
for d in $(find lib -maxdepth 2 -type d); do
  INC="$INC -I$d"
done

CXX="${CXX:-g++}"

echo "== 编译全链路 P1（真实 task_scheduler.cpp + state_data.cpp）=="
$CXX -std=c++17 -O2 -g \
  -DTANDEMVEC_FIRMWARE \
  -Wno-address-of-packed-member -Wno-psabi \
  -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE \
  -DBFS_TASK_FRACTIONAL_INTERVALS \
  $INC \
  src/state_data.cpp \
  src/task_scheduler.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp \
  test_host/fw_sil/fullstack/fs_scheduler_main.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
  lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
  -o "$BIN_DIR/fs_sched"

OUT="$BIN_DIR/fs_sched"
if [ -f "$BIN_DIR/fs_sched.exe" ]; then OUT="$BIN_DIR/fs_sched.exe"; fi

echo "== 运行 =="
"$OUT"