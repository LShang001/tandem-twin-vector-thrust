#!/usr/bin/env bash
# ============================================================
# 全链路数字孪生 · 阶段 P3：真实传感器模块 initImu() 上链路
#
# 链接的真实固件源码（一行不改）：
#   src/sensor_imu.cpp       initImu / initMagnetometer / readIMUData / handleImuTask
#   src/imu_factory.cpp      型号装配
#   src/state_data.cpp       全局状态与参数实例
#   lib/ICM42688/src/*.cpp   真实芯片驱动
#   lib/MagSensor + 磁芯片驱动
#   lib/MadgwickAHRS         AHRS（sensor_imu 直接使用）
#
# 替换的唯一一层：SPI/I2C 总线（寄存器级替身）+ 其他外设空桩。
#
# ★本脚本走增量构建（build_incremental.sh）：首次全量较慢，之后只重编改动单元。
#
# 用法（仓库根目录）：
#   bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_sensor.sh
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
# ★用 -O0：本目标是"接线验证"，不评估性能；-O0 让 Eigen 模板实例化快一个量级，
#   把单次全量编译从 5 分钟压到 1 分钟内。若将来做逐拍对拍，两条构建必须同级别。
CXXFLAGS="-std=c++17 -O0 -g -DTANDEMVEC_FIRMWARE -DFW_SIL_REAL_IMU \
 -Wno-address-of-packed-member -Wno-psabi \
 -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE \
 -DBFS_TASK_FRACTIONAL_INTERVALS"
export CXXFLAGS

echo "== 编译全链路 sensor_imu 接线验证（增量）=="
OUT="$(bash test_host/fw_sil/fullstack/build_incremental.sh fs_sensor \
  src/state_data.cpp \
  src/sensor_imu.cpp \
  src/imu_factory.cpp \
  lib/ICM42688/src/ICM42688.cpp \
  lib/ICM42688/src/ICM42688_ImuDevice.cpp \
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
  lib/MagSensor/MagSensor.cpp \
  lib/IST8310/IST8310.cpp \
  lib/QMC5883L/QMC5883L.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
  lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp \
  test_host/fw_sil/fullstack/fs_sensor_main.cpp)"

echo "== 运行 =="
# ★必须带超时：真实 initMagnetometer/驱动里存在"探测失败后重试"的等待循环，
#   宿主桩下这些循环可能因外设恒失败而不退出。没有超时会把验证变成无限挂起。
if command -v timeout >/dev/null 2>&1; then
  timeout 30 "$OUT"
  rc=$?
  if [ "$rc" -eq 124 ]; then
    echo "[FAIL] 运行超过 30 s 未结束——疑似外设重试循环不退出（见脚本注释）" >&2
    exit 124
  fi
  exit "$rc"
else
  "$OUT"
fi