#!/bin/bash
# §4.24w W 组：子集"每历元配额"门（默认 1 次）的同二进制 A/B。
# 先建 32 g 档新二进制（含配额门），再跑 配额1 / 配额255 × 两段 seeds。
set -uo pipefail
cd /d/纵列双发矢量推力飞行器/TandemVec_FCS
BIN_DIR=/tmp/tv_32g_w2 EXTRA_CXXFLAGS="-DBFS_ICM42688_ACCEL_FS_G=32.0f -DBFS_ICM42688_ACCEL_RANGE_MPS2=313.8f" \
  /bin/bash test_host/fw_sil/build_incremental.sh --build-only > /tmp/build_w.log 2>&1
BUILD_RC=$?
echo "BUILD_RC=$BUILD_RC"
# ★ 构建失败必须硬停：否则会拿目录里残留的旧二进制跑完整个矩阵，
#   而"等价性检查"照样能通过（旧二进制当然等于旧归档）⇒ 静默验证了个寂寞。
if [ "$BUILD_RC" -ne 0 ] || [ ! -f /tmp/tv_32g_w2/fw_real.exe ]; then
  echo "BUILD FAILED / 缺二进制，中止矩阵"; tail -5 /tmp/build_w.log; exit 1
fi
md5sum /tmp/tv_32g_w2/fw_real.exe | cut -c1-12
cd /d/纵列双发矢量推力飞行器/TandemVec-EKF-Report
EXE="C:/Users/12631/AppData/Local/Temp/tv_32g_w2/fw_real.exe"
run() {
  local tag="$1"; local start="$2"; shift 2
  echo "### ARM $tag (seeds $start..$((start+49))) : $*"
  py -3.12 sim/run_305g_onset_scan.py --exe "$EXE" --seeds 50 --seed-start "$start" \
    --bind-core 5 --ladder 23900 --workers 8 --tag "2026-10-28_${tag}" \
    --extra-args "$*" | tail -4
}
run w_q1_s51   51 "--gnss-vel-subset 1 --gnss-rate-hz 20"
run w_q255_s51 51 "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-maxfires 255"
run w_q1_s1    1  "--gnss-vel-subset 1 --gnss-rate-hz 20"
run w_q255_s1  1  "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-maxfires 255"
echo "### VERDICT W1-W3"
py -3.12 sim/check_vel_subset_quota.py
echo "W_RC=$?"
