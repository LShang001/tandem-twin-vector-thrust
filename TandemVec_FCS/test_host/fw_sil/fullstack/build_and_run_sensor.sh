#!/usr/bin/env bash
# ============================================================
# 全链路数字孪生 · 阶段 P3：真实传感器模块 initImu() 上链路
#
# 链接的真实固件源码（一行不改）：
#   src/sensor_imu.cpp       initImu / initMagnetometer / readIMUData / handleImuTask
#   src/imu_factory.cpp      型号装配
#   src/state_data.cpp       全局状态与参数实例
#   lib/ICM42688/src/*.cpp   真实芯片驱动
#   lib/MadgwickAHRS         AHRS（sensor_imu 直接使用）
#
# 替换的唯一一层：SPI 总线（寄存器级替身）+ 其他外设空桩。
#
# 用法（仓库根目录）：
#   bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_sensor.sh
# ============================================================
set -eu

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT"

BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/unignc_fullstack}"
export BIN_DIR
mkdir -p "$BIN_DIR"

INC="-Itest_host/fw_sil/stub -Iinclude -Isrc -Itest_host/stub"
for d in $(find lib -maxdepth 2 -type d); do
  INC="$INC -I$d"
done

CXX="${CXX:-g++}"

echo "== 编译全链路 sensor_imu 接线验证 =="
$CXX -std=c++17 -O2 -g \
  -DTANDEMVEC_FIRMWARE \
  -DFW_SIL_REAL_IMU \
  -Wno-address-of-packed-member -Wno-psabi \
  -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE \
  -DBFS_TASK_FRACTIONAL_INTERVALS \
  $INC \
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
  test_host/fw_sil/fullstack/fs_sensor_main.cpp \
  -o "$BIN_DIR/fs_sensor"

OUT="$BIN_DIR/fs_sensor"
if [ -f "$BIN_DIR/fs_sensor.exe" ]; then OUT="$BIN_DIR/fs_sensor.exe"; fi

echo "== 运行 =="
"$OUT"