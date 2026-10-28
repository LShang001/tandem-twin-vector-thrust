#!/bin/bash
# §4.24t Y6：量触发上限的上边界（cap 25/30/35 × 两段 seeds）。
# cap25 重跑一遍用于 Y6a 的"与 §4.24s 归档逐 seed 相同"参照一致性检查（顺带证运行间确定性）。
# 用法：/bin/bash sim/run_vel_subset_capupper_matrix.sh
set -uo pipefail
cd "$(dirname "$0")/.."
EXE="C:/Users/12631/AppData/Local/Temp/tv_32g_q1/fw_real.exe"
echo "exe md5: $(md5sum /tmp/tv_32g_q1/fw_real.exe | cut -c1-8)"

run() {
  local tag="$1"; local start="$2"; local cap="$3"
  echo "### ARM $tag (seeds $start..$((start+49))) cap=$cap"
  py -3.12 sim/run_305g_onset_scan.py --exe "$EXE" --seeds 50 --seed-start "$start" \
    --bind-core 5 --ladder 23900 --workers 8 --tag "2026-10-28_${tag}" \
    --extra-args "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres ${cap}" | tail -4
}

for cap in 25 30 35; do
  run "u${cap}_s51" 51 "$cap"
  run "u${cap}_s1" 1 "$cap"
done

echo "### VERDICT Y6a-Y6c"
py -3.12 sim/check_vel_subset_capupper.py
VERDICT_RC=$?
echo "CAPUPPER_RC=$VERDICT_RC"
