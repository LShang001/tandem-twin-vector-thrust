#!/usr/bin/env bash
# ============================================================
# 全链路数字孪生 · 阶段 P2/P3 入口：真实 IMU 驱动 begin() 上宿主
#
# 链接的真实固件源码（一行不改）：
#   src/imu_factory.cpp                 按编译期宏装配芯片后端
#   lib/ICM42688/src/*.cpp              真实 ICM 芯片驱动（含 delay+回读校验）
#   lib/ImuAbstraction                  型号无关接口
#
# 替换的唯一一层：SPI 总线。宿主桩开启寄存器级替身后按真实读写协议
# 模拟 128 字节寄存器文件，驱动内部的 reset/WHO_AM_I/FSR/ODR/LPF 全部照常执行。
#
# 用法（仓库根目录）：
#   bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_imu.sh
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

echo "== 编译全链路 IMU 驱动验证 =="
$CXX -std=c++17 -O2 -g \
  -DTANDEMVEC_FIRMWARE \
  -Wno-address-of-packed-member -Wno-psabi \
  -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE \
  $INC \
  src/imu_factory.cpp \
  lib/ICM42688/src/ICM42688.cpp \
  lib/ICM42688/src/ICM42688_ImuDevice.cpp \
  test_host/fw_sil/fullstack/fs_imu_begin_main.cpp \
  -o "$BIN_DIR/fs_imu_begin"

OUT="$BIN_DIR/fs_imu_begin"
if [ -f "$BIN_DIR/fs_imu_begin.exe" ]; then OUT="$BIN_DIR/fs_imu_begin.exe"; fi

echo "== 运行 =="
"$OUT"