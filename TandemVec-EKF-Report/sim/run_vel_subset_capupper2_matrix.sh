#!/bin/bash
# §4.24t Y7：把上界继续往上顶（cap 40/50 × 两段 seeds），判据沿用 Y6b 的定义。
# 用法：/bin/bash sim/run_vel_subset_capupper2_matrix.sh
set -uo pipefail
cd "$(dirname "$0")/.."
EXE="C:/Users/12631/AppData/Local/Temp/tv_32g_q1/fw_real.exe"
echo "exe md5: $(md5sum /tmp/tv_32g_q1/fw_real.exe | cut -c1-8)"

run() {
  local tag="$1"; local start="$2"; local cap="$3"
  echo "### ARM $tag (seeds $start..$((start+49))) cap=$cap"
  py -3.12 sim/run_305g_onset_scan.py --exe "$EXE" --seeds 50 --seed-start "$start" \
    --bind-core 5 --ladder 23900 --workers 8 --tag "2026-10-28_${tag}" \
    --extra-args "--gnss-vel-subset 1 --gnss-rate-hz 20 --gnss-vel-subset-posres ${cap}" | tail -3
}

for cap in 40 50; do
  run "v${cap}_s51" 51 "$cap"
  run "v${cap}_s1" 1 "$cap"
done

echo "### VERDICT Y7a-Y7c"
py -3.12 sim/check_vel_subset_capupper2.py
VERDICT_RC=$?
echo "CAPUPPER2_RC=$VERDICT_RC"
