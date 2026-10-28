#!/bin/bash
# §4.24r Y 组：15 m 触发上限在独立 seeds 51–100 上的泛化与阈值平坦性矩阵。
# 七臂全部跑在同一个 32 g 二进制（tv_32g_q1，含 --gnss-vel-subset-posres 运行时开关），
# 只差运行时参数 ⇒ 同二进制 A/B。判据 Y1–Y4 写在 sim/check_vel_subset_capval.py 头部（跑前已固定）。
# 用法：/bin/bash sim/run_vel_subset_capval_matrix.sh
set -uo pipefail
cd "$(dirname "$0")/.."
EXE="C:/Users/12631/AppData/Local/Temp/tv_32g_q1/fw_real.exe"
echo "exe md5: $(md5sum /tmp/tv_32g_q1/fw_real.exe | cut -c1-8)"

run() {
  local tag="$1"; shift
  echo "### ARM $tag : $*"
  py -3.12 sim/run_305g_onset_scan.py --exe "$EXE" --seeds 50 --seed-start 51 \
    --bind-core 5 --ladder 23900 --workers 8 --tag "2026-10-28_${tag}" \
    --extra-args "$*" | tail -18
}

run y20_off   "--gnss-rate-hz 20"
run y20_nocap "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres 1e6"
run y20_c12   "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres 12"
run y20_c15   "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres 15"
run y20_c21   "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres 21"
run y10_nocap "--gnss-vel-subset 1 --gnss-rate-hz 10 --gnss-vel-subset-posres 1e6"
run y10_c15   "--gnss-vel-subset 1 --gnss-rate-hz 10 --gnss-vel-subset-posres 15"

echo "### VERDICT Y1-Y4"
py -3.12 sim/check_vel_subset_capval.py
VERDICT_RC=$?
echo "Y_GROUP_RC=$VERDICT_RC"
