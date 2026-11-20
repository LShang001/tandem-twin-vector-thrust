#!/usr/bin/env bash
# ============================================================
# 全链路数字孪生 · 阶段 P2：真实 main.cpp 初始化与主循环
#
# 链接 11 个固件源文件（含 main.cpp / navigation_task.cpp / sensor_imu.cpp /
# communication.cpp 等），只替换最底层 SPI 与时钟。
#
# 用法（仓库根目录）：
#   bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_main.sh
# ============================================================
set -u
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT" || exit 2

export BIN_DIR="${BIN_DIR:-/tmp/unignc_fullstack}"
export OBJ_NAME=fs_main
export INC="-Itest_host/fw_sil/stub -Iinclude -Isrc -Itest_host/stub"
for d in $(find lib -maxdepth 2 -type d); do
  INC="$INC -I$d"
done
export INC
export CXXFLAGS="-std=c++17 -O0 -g -DTANDEMVEC_FIRMWARE -DBFS_REAL_UBX -DFW_SIL_REAL_IMU -DFW_SIL_REAL_LIBS \
 -DARDUINO=10819 -Wno-address-of-packed-member -Wno-psabi \
 -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE \
 -DBFS_TASK_FRACTIONAL_INTERVALS -DBFS_DISABLE_IWDG"

OUTPUT=$(bash test_host/fw_sil/fullstack/build_incremental.sh fs_main \
  src/main.cpp \
  src/state_data.cpp \
  src/task_scheduler.cpp \
  src/navigation_task.cpp \
  src/sensor_imu.cpp \
  src/imu_factory.cpp \
  src/sensor_peripheral.cpp \
  src/communication.cpp \
  src/ano_params.cpp \
  src/ano_vars.cpp \
  src/flight_control.cpp \
  src/ubx_config.cpp \
  src/mavlink_bridge.cpp \
  lib/ublox-main/src/ubx.cpp \
  lib/ublox-main/src/minmea.c \
  lib/AnoComProtocol/AnoComProtocol.cpp \
  lib/CrsfSerial/CrsfSerial.cpp \
  lib/crsf/crsf.cpp \
  lib/crsf/common_rc.cpp \
  lib/units/src/convang.cpp \
  lib/W25N01GV/src/W25N01GV.cpp \
  lib/W25N01GV/src/W25N01GV_log.cpp \
  lib/ICM42688/src/ICM42688.cpp \
  lib/ICM42688/src/ICM42688_ImuDevice.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
  lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
  lib/MagSensor/MagSensor.cpp \
  lib/IST8310/IST8310.cpp \
  lib/QMC5883L/QMC5883L.cpp \
  lib/LQS48_Flow/LQS48_Flow.cpp \
  lib/MTF02P/MTF02P.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp \
  test_host/fw_sil/fullstack/fs_peripheral_stubs.cpp \
  test_host/fw_sil/fullstack/fs_main_main.cpp)

if command -v timeout >/dev/null 2>&1; then timeout 120 "$OUTPUT"; else "$OUTPUT"; fi