#!/bin/bash
# §4.24s Y5 边界映射：把 15 m 单点量成平台区间 [L, U)。
# 复用 §4.24r 已跑的 y20_c12/c15/c21 与 §4.24q 的 q20（=1–50 段 cap15），本轮只补四个新臂。
# 用法：/bin/bash sim/run_vel_subset_capband_matrix.sh
set -uo pipefail
cd "$(dirname "$0")/.."
EXE="C:/Users/12631/AppData/Local/Temp/tv_32g_q1/fw_real.exe"
echo "exe md5: $(md5sum /tmp/tv_32g_q1/fw_real.exe | cut -c1-8)"

run() {
  local tag="$1"; local start="$2"; shift 2
  echo "### ARM $tag (seeds $start..$((start+49))) : $*"
  py -3.12 sim/run_305g_onset_scan.py --exe "$EXE" --seeds 50 --seed-start "$start" \
    --bind-core 5 --ladder 23900 --workers 8 --tag "2026-10-28_${tag}" \
    --extra-args "$*" | tail -4
}

run y20_c13 51 "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres 13"
run y20_c14 51 "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres 14"
run y20_c18 51 "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres 18"
run y20_c25 51 "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres 25"
run p20_c25 1  "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres 25"

echo "### VERDICT Y5a-Y5c"
py -3.12 sim/check_vel_subset_capband.py
VERDICT_RC=$?
echo "CAPBAND_RC=$VERDICT_RC"
