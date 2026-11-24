#!/usr/bin/env bash
# ============================================================
# 全链路数字孪生 · 阶段 P5：ubx_config 快路径 / 全量路径验证
#
# 链接真实 src/ubx_config.cpp，串口对端换成「替身接收机」：
# 它按 UBX 协议解析固件发出的每一帧，回 ACK / poll 响应，维护 CFG 配置
# 数据库（VALSET/VALGET），并按自身配置状态决定是否输出 NAV-PVT/EOE。
#
# 三个场景：
#   S1 冷机（9600 + 只有 NMEA + 空配置库）→ 必须走全量并配到目标；
#   S2 已配置（921600 + PVT/EOE 在出 + 键值全对）→ 必须命中快路径且 0 配置帧；
#   S3 半配置（PVT/EOE 在出但 NMEA 输出未关、dynModel 低一档）→ 快路径必须拒绝。
#
# 用法（仓库根目录）：
#   bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_ubxcfg.sh
# 调试（打印替身收到的每条命令）：
#   SIM_UBX_DEBUG=1 bash .../build_and_run_ubxcfg.sh
# ============================================================
set -u

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT" || exit 2

export BIN_DIR="${BIN_DIR:-/tmp/unignc_fullstack}"
export OBJ_NAME=fs_ubxcfg
export INC="-Itest_host/fw_sil/stub -Iinclude -Isrc"
export CXXFLAGS="-std=c++17 -O0 -g -DTANDEMVEC_FIRMWARE -DARDUINO=10819 \
 -Wno-address-of-packed-member -Wno-psabi"

echo "== 编译 UBX 配置验证（增量）=="
OUT="$(bash test_host/fw_sil/fullstack/build_incremental.sh fs_ubxcfg \
  src/ubx_config.cpp \
  test_host/fw_sil/fullstack/fs_ubxcfg_main.cpp)"

echo "== 运行 =="
if command -v timeout >/dev/null 2>&1; then
  timeout 120 "$OUT"
else
  "$OUT"
fi