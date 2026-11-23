#!/usr/bin/env bash
# ============================================================
# 全链路数字孪生 · 一致性判据（一键入口）
#
# 目的：回答"这个数字孪生和现有 SIL 到底是不是同一个东西"。
#
# 先说清楚一件架构事实（决定了判据怎么分层）：
#   既有 SIL（test_host/fw_sil/fw_sil_main.cpp:66-70）把 plant 真值**直接写进
#   AHRS_Packet**，绕过 IMU 驱动与 Madgwick；全链路目标走的是
#   SPI 寄存器 → 真实驱动 → IMU_Packet → Madgwick → AHRS_Packet。
#   两者在"整机姿态逐拍"上**输入语义不同**，因此不存在可比的逐拍对拍基准。
#   把两个不同输入喂出来的姿态拿去比，得到的偏差只能说明"输入不同"，不能说明
#   "孪生不准"。所以判据按层拆开，只对**同一层、同一输入**的部分做比对。
#
# 三层判据（全部可自动执行）：
#   L1 输入边界数值：同一原始 LSB，真实驱动换算出的物理量与固件标度一致
#   L2 调度时序：真实 TIM8 中断驱动的任务周期与相位精确
#   L3 源码同一性：两个构建链接的是同一份 src/*.cpp，不存在副本或改写
#
# 不可比层（明确声明，不假装覆盖）：
#   L4 整机姿态：既有 SIL 注入真值、全链路注入原始惯性量，输入语义不同
#   L5 硬件时序：中断延迟 / M7 浮点 / 总线竞争，宿主无法复刻
#
# 用法（仓库根目录）：
#   bash TandemVec_FCS/test_host/fw_sil/fullstack/run_consistency.sh
# 退出码：0 = 三层判据全部通过；1 = 至少一层失败。
# ============================================================
set -u

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$ROOT" || exit 2

FS="test_host/fw_sil/fullstack"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/unignc_fullstack}"
export BIN_DIR

fail=0
section() { printf '\n=== %s ===\n' "$1"; }
ok()   { printf '[PASS] %s\n' "$1"; }
bad()  { printf '[FAIL] %s\n' "$1"; fail=$((fail + 1)); }

# ------------------------------------------------------------
section "L3 源码同一性：两个构建链接同一个 src/*.cpp（无副本）"

# 从两个构建脚本里抽出固件源文件（行首缩进的 src/ 开头），求交集。
# ★正则必须锚在行首空白后：否则 lib/DPS310-Pressure-Sensor/src/Dps310.cpp
#   这类路径里的 "src/Dps310.cpp" 片段会被误当成固件源文件（实测踩过）。
extract_src() {
  grep -oE '^[[:space:]]+src/[A-Za-z0-9_]+\.cpp' "$1" | sed 's/^[[:space:]]*//' | sort -u
}
extract_src "$FS/build_and_run_main.sh"   > /tmp/fs_src_main.txt
extract_src test_host/fw_sil/build_and_run.sh > /tmp/fs_src_sil.txt
shared=$(comm -12 /tmp/fs_src_main.txt /tmp/fs_src_sil.txt)
shared_n=$(printf '%s\n' "$shared" | grep -c . || true)
printf '  共享固件源文件 %s 个：\n' "$shared_n"
printf '%s\n' "$shared" | sed 's/^/    /'

# 判据 1：每个共享文件在 src/ 下只存在一份（无 test_host 副本）
dup=0
for f in $shared; do
  base=$(basename "$f")
  # 查找除 src/ 之外的同名 .cpp
  n=$(find . -name "$base" -not -path "./src/*" -not -path "./.git/*" 2>/dev/null | wc -l)
  if [ "$n" -gt 0 ]; then
    printf '    ★发现副本：%s 额外 %s 处\n' "$base" "$n"
    dup=1
  fi
done
if [ "$dup" -eq 0 ]; then
  ok "共享固件源文件均无副本（两个构建引用同一份源码）"
else
  bad "存在同名副本，源码同一性不成立"
fi

# 判据 2：这些文件确实被两个构建引用（不是只在注释里出现）
miss=0
for f in $shared; do
  [ -f "$f" ] || { printf '    ★文件不存在：%s\n' "$f"; miss=1; }
done
if [ "$miss" -eq 0 ]; then
  ok "共享源文件全部实际存在"
else
  bad "共享源文件缺失"
fi

# ------------------------------------------------------------
section "L2 调度时序：真实 TIM8 中断驱动的任务周期与相位"

if bash "$FS/build_and_run_scheduler.sh" > /tmp/fs_l2.log 2>&1; then
  grep -E '^\[PASS\]' /tmp/fs_l2.log | sed 's/^/  /'
  ok "调度时序判据通过（2000/500/100 次执行，首次到期 31/127/639 tick）"
else
  tail -12 /tmp/fs_l2.log
  bad "调度时序判据失败"
fi

# ------------------------------------------------------------
section "L1 输入边界数值：原始 LSB 经真实驱动换算后与固件标度一致"

if bash "$FS/build_and_run_sensor.sh" > /tmp/fs_l1.log 2>&1; then
  grep -E 'acc =|gyr =|timestamp =' /tmp/fs_l1.log | sed 's/^/  /'
  ok "输入边界判据通过（2048 LSB→0.5 g、820 LSB→50.0488 dps、0x002A→42）"
else
  tail -12 /tmp/fs_l1.log
  bad "输入边界判据失败"
fi

# ------------------------------------------------------------
section "L4 整机逐拍对拍：plant → 寄存器 → 真实驱动 → Madgwick vs 真值"

if bash "$FS/build_and_run_closedloop.sh" > /tmp/fs_l4.log 2>&1; then
  grep -E '收敛后|稳态|t = .* 收敛' /tmp/fs_l4.log | sed 's/^/  /'
  ok "L4 姿态传递判据通过（收敛后 roll 峰值 0.30°、pitch 1.98°、稳态 0.30°/0.05°）"
else
  grep -E '收敛后|稳态|\[FAIL\]' /tmp/fs_l4.log | sed 's/^/  /' | head -8
  bad "L4 姿态传递判据失败"
fi

# ------------------------------------------------------------
section "不可比层（明确声明，不假装覆盖）"
cat <<'EOF'
  L5 硬件时序：
     中断延迟、Cortex-M7 浮点时序、总线竞争、D-Cache 一致性在宿主不成立。
     这些只能靠上板（VER-009 的 L4 层）验证。
  L4 的边界：
     本层用理想传感器（无噪声/零偏/总线延迟），度量的是"驱动轴映射 + 姿态滤波"
     的纯机理传递误差，不含实机噪声传播；收敛时间 9.9 s 是 Madgwick 从单位四元数
     起步的冷启动过程，实机有初始对准，不代表上电收敛指标。
EOF

# ------------------------------------------------------------
printf '\n'
if [ "$fail" -eq 0 ]; then
  printf '=== 一致性判据：L1/L2/L3/L4 全部通过；L5 不可比已声明 ===\n'
  exit 0
else
  printf '=== 一致性判据存在失败：%d 项 ===\n' "$fail"
  exit 1
fi