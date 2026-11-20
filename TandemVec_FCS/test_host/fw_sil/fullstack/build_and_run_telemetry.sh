#!/usr/bin/env bash
# ============================================================
# 全链路数字孪生 · 阶段 P4：遥测帧字节级验证
#
# 链接真实 communication.cpp，调用 sendPositionVelocityData() 组帧，
# 从串口桩捕获写出的字节，再按固件声明的布局解析回原值对拍。
#
# 用法（仓库根目录）：
#   bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_telemetry.sh
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
CXXFLAGS="-std=c++17 -O0 -g -DTANDEMVEC_FIRMWARE -DBFS_REAL_UBX -DFW_SIL_REAL_IMU -DFW_SIL_REAL_LIBS -DARDUINO=10819 \
 -Wno-address-of-packed-member -Wno-psabi \
 -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE \
 -DBFS_TASK_FRACTIONAL_INTERVALS"
export CXXFLAGS

echo "== 编译遥测帧验证（增量）=="
OUT="$(bash test_host/fw_sil/fullstack/build_incremental.sh fs_telemetry \
  src/state_data.cpp \
  src/communication.cpp \
  src/ano_params.cpp \
  src/ano_vars.cpp \
  src/flight_control.cpp \
  src/task_scheduler.cpp \
  src/sensor_peripheral.cpp \
  src/ubx_config.cpp \
  src/mavlink_bridge.cpp \
  src/sensor_imu.cpp \
  src/imu_factory.cpp \
  lib/ICM42688/src/ICM42688.cpp \
  lib/ICM42688/src/ICM42688_ImuDevice.cpp \
  lib/ublox-main/src/ubx.cpp \
  lib/ublox-main/src/minmea.c \
  lib/AnoComProtocol/AnoComProtocol.cpp \
  lib/units/src/convang.cpp \
  lib/W25N01GV/src/W25N01GV.cpp \
  lib/W25N01GV/src/W25N01GV_log.cpp \
  lib/DPS310-Pressure-Sensor/src/Dps310.cpp \
  lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
  lib/CrsfSerial/CrsfSerial.cpp \
  lib/crsf/crsf.cpp \
  lib/crsf/common_rc.cpp \
  lib/LQS48_Flow/LQS48_Flow.cpp \
  lib/MTF02P/MTF02P.cpp \
  lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp \
  lib/MagSensor/MagSensor.cpp \
  lib/IST8310/IST8310.cpp \
  lib/QMC5883L/QMC5883L.cpp \
  test_host/fw_sil/fw_sil_stubs.cpp \
  test_host/fw_sil/fullstack/fs_peripheral_stubs.cpp \
  test_host/fw_sil/fullstack/fs_telemetry_main.cpp)"

echo "== 运行 =="
if command -v timeout >/dev/null 2>&1; then
  timeout 60 "$OUT"
else
  "$OUT"
fi