#!/usr/bin/env bash
# ============================================================
# L4 姿态传递 · 频率扫描
#
# 回答一个问题：这个数字孪生在**多快的动态下还准**。
# 单点（0.2 Hz）只能说明"慢变时一致"，看不出滤波带宽与相位滞后；
# 扫一组频率，才能给出"误差随激励频率怎么长"的量级曲线。
#
# 做法：同一 plant、同一驱动链路、同一判据，只改激励频率。
# 每个频率的仿真时长按「覆盖收敛（约 10 s）+ 至少 3 个周期」取。
#
# 用法（仓库根目录）：
#   bash TandemVec_FCS/test_host/fw_sil/fullstack/run_sweep.sh
# 退出码：0 = 全部频率都跑出结果；1 = 至少一个频率失败。
# ============================================================
set -u

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT" || exit 2

FS="test_host/fw_sil/fullstack"
export BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/unignc_fullstack}"

# 频率(Hz) 与对应仿真时长(s)：时长 ≥ 覆盖 ~10 s 收敛 + 3 个周期
CASES=(
  "0.05 70"
  "0.20 30"
  "0.50 27"
  "1.00 25"
  "2.00 24"
)

fail=0
# NOISE=1 时开启传感器噪声与零偏（默认关＝纯机理传递误差）
EXTRA_ARGS=""
if [ "${NOISE:-0}" = "1" ]; then
  EXTRA_ARGS="--noise"
  printf '传感器误差：已开启（acc σ=0.004 g / gyr σ=0.12 dps + 三轴零偏，假设值）\n\n'
fi

printf '%-8s %-10s %-10s %-10s %-10s %-8s %-6s\n' \
  "freq(Hz)" "roll峰(°)" "pitch峰(°)" "稳态roll(°)" "稳态pitch(°)" "收敛(s)" "判据"
printf '%s\n' "-------------------------------------------------------------------------------"

rows=()
for c in "${CASES[@]}"; do
  set -- $c
  freq="$1"
  dur="$2"
  log="/tmp/sweep_${freq}.log"
  # ★不看退出码：高频下判据本就该 FAIL（误差随频率增长），但那是**要的数据**，
  #   不是"运行失败"。只有跑不出 SWEEP 行才算真失败。
  bash "$FS/build_and_run_closedloop.sh" --freq "$freq" --dur "$dur" $EXTRA_ARGS > "$log" 2>&1
  rc=$?
  line=$(grep -E '^SWEEP ' "$log" | tail -1)
  if [ -z "$line" ]; then
    printf '%-8s %s\n' "$freq" "无 SWEEP 输出（rc=$rc，见 $log）"
    fail=$((fail + 1))
    continue
  fi
  verdict="PASS"
  [ "$rc" -ne 0 ] && verdict="超阈"
  rp=$(printf '%s\n' "$line" | sed -n 's/.*roll_peak=\([0-9.]*\).*/\1/p')
  pp=$(printf '%s\n' "$line" | sed -n 's/.*pitch_peak=\([0-9.]*\).*/\1/p')
  sr=$(printf '%s\n' "$line" | sed -n 's/.*steady_r=\([0-9.]*\).*/\1/p')
  sp=$(printf '%s\n' "$line" | sed -n 's/.*steady_p=\([0-9.]*\).*/\1/p')
  cv=$(printf '%s\n' "$line" | sed -n 's/.*conv=\([-0-9.]*\).*/\1/p')
  printf '%-8s %-10s %-10s %-10s %-10s %-8s %-6s\n' \
    "$freq" "$rp" "$pp" "$sr" "$sp" "$cv" "$verdict"
  rows+=("$freq|$rp|$pp|$sr|$sp|$cv|$verdict")
done

printf '\n-- 判读口径 --\n'
printf '  峰值列 = 收敛后（首次进入 ±2° 之后）的最大偏差，反映动态跟踪能力；\n'
printf '  稳态列 = 16 s 之后的残差，反映静态精度；\n'
printf '  频率升高时峰值变大属预期（滤波相位滞后），关键是看它在哪个频点越过 3°。\n'
printf '  边界：理想传感器（无噪声/零偏/总线延迟），不含实机噪声传播。\n'

printf '\n-- 机读汇总 --\n'
for r in "${rows[@]}"; do printf 'SWEEP_ROW %s\n' "$r"; done

if [ "$fail" -eq 0 ]; then
  printf '\n=== 频率扫描完成：%d 个频点全部有结果 ===\n' "${#CASES[@]}"
  exit 0
else
  printf '\n=== 频率扫描有 %d 个频点失败 ===\n' "$fail"
  exit 1
fi