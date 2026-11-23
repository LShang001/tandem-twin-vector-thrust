#!/usr/bin/env bash
# ============================================================================
# verify-ekf-lambda-fix.sh —— 验证 ekf_15_state.h:3839 悬垂引用的修复
#
# 用途：修完 `phi_at_time` 的返回类型之后，一条命令跑完全部验收。
#       不修也能跑——它会如实告诉你"还没修"。
#
# 背景（定位过程见 README §7.6 与提交 db766a4b）：
#   const auto phi_at_time = [&](NavCovScalar tau) {
#     const Eigen::Matrix<NavCovScalar,15,15> f_tau = fs_d * tau;
#     return identity_d + f_tau + 0.5*tau...;   // 返回惰性表达式，持有 f_tau 引用
#   };
#   lambda 返回后 f_tau 销毁，调用方求值时读到失效栈内存。
#   ASan 报 stack-use-after-scope，调用链落在
#   handleNavigationSystem → TimeUpdateTwoSample → PropagateStateFromIncrements。
#
# 判据三步：
#   1) ASan 干净     —— 不再出现 stack-use-after-scope
#   2) 常规跑批稳定   —— N 次连跑成功率 100%
#   3) 稳态数值不变   —— 成功样本的稳态高度与修前一致（防止"改对了但改偏了"）
#
# 用法：
#   bash test_host/fw_sil/fullstack/verify-ekf-lambda-fix.sh [次数，默认 5]
# ============================================================================
set -uo pipefail

RUNS="${1:-5}"
DUR="${DUR:-40}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
CLOSEDLOOP="$SCRIPT_DIR/build_and_run_closedloop.sh"

cd "$REPO_ROOT" || exit 1

echo "=== 1/3 ASan 检查（$DUR s）==="
ASAN_LOG="$(mktemp)"
BIN_DIR=/tmp/unignc_asan_verify \
EXTRA_FLAGS="-O2 -fsanitize=address -fno-omit-frame-pointer -g" \
  bash "$CLOSEDLOOP" --hover --dur "$DUR" >"$ASAN_LOG" 2>&1

if grep -qE 'stack-use-after-(return|scope)|AddressSanitizer: [a-z-]+ on address' "$ASAN_LOG"; then
  echo "  [FAIL] ASan 仍报内存错误："
  grep -E 'ERROR: AddressSanitizer|SUMMARY: AddressSanitizer' "$ASAN_LOG" | head -4 | sed 's/^/         /'
  echo "         → 说明 3839 那行还没修，或修法没生效（检查是否真的加了显式返回类型）"
  ASAN_OK=0
else
  echo "  [PASS] ASan 未报 stack-use-after-* 或其它内存错误"
  ASAN_OK=1
fi
rm -f "$ASAN_LOG"

echo ""
echo "=== 2/3 常规构建连跑 $RUNS 次 ==="
PASS=0
FAIL=0
MEANS=""
for i in $(seq 1 "$RUNS"); do
  LOG="$(mktemp)"
  bash "$CLOSEDLOOP" --hover --dur "$DUR" >"$LOG" 2>&1
  LINE="$(grep -E '^  HOVER' "$LOG" | head -1)"
  MEAN="$(printf '%s' "$LINE" | sed -n 's/.*alt_mean=\([-0-9.nan]*\).*/\1/p')"
  if printf '%s' "$LINE" | grep -q 'conv_t=[0-9]' && \
     awk -v m="$MEAN" 'BEGIN{exit !(m>9.0 && m<11.0)}'; then
    PASS=$((PASS+1))
    MEANS="$MEANS $MEAN"
    printf '  run%-2s PASS  alt_mean=%s\n' "$i" "$MEAN"
  else
    FAIL=$((FAIL+1))
    printf '  run%-2s FAIL  %s\n' "$i" "${LINE:-（无 HOVER 输出）}"
  fi
  rm -f "$LOG"
done
echo "  通过 $PASS / $RUNS"

echo ""
echo "=== 3/3 稳态数值一致性 ==="
# 修前实测（--hover --dur 40，理想传感器）稳态 10.525 m。
# 修法只改求值时机，不应改变稳态；偏离说明改动动到了数学。
REF_MEAN="10.525"
if [ -n "$MEANS" ]; then
  echo "  本次成功样本稳态:$MEANS"
  echo "  修前基准:          $REF_MEAN"
  BAD="$(for m in $MEANS; do awk -v a="$m" -v b="$REF_MEAN" 'BEGIN{d=a-b; if(d<0)d=-d; if(d>0.05) print a}'; done)"
  if [ -n "$BAD" ]; then
    echo "  [WARN] 下列样本与基准偏差 >0.05 m：$BAD"
    echo "         → 求值时机不该改变数值，先确认修法是否只加了返回类型"
    NUM_OK=0
  else
    echo "  [PASS] 稳态与修前基准一致（偏差 ≤0.05 m）"
    NUM_OK=1
  fi
else
  echo "  [SKIP] 没有成功样本，无法比对"
  NUM_OK=0
fi

echo ""
echo "--------------------------------------------------------------------------"
if [ "$ASAN_OK" = 1 ] && [ "$FAIL" = 0 ] && [ "$NUM_OK" = 1 ]; then
  echo "=== 全部通过：ASan 干净、$RUNS/$RUNS 成功、稳态未漂 ==="
  exit 0
else
  echo "=== 未通过：ASan=${ASAN_OK} 成功=${PASS}/${RUNS} 数值=${NUM_OK} ==="
  exit 1
fi