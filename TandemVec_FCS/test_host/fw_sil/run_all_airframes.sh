#!/usr/bin/env bash
# ============================================================
# 三机型 SIL 统一跑批（UniGNC）
#
# 一条命令验证"同一份固件在不同执行机构上都能闭环"：
#   纵列双发（基型）  build_and_run.sh           定高 + 手动阶跃
#   共轴双桨 VTVL     run_coaxial_acceptance.sh 8 profiles × 3 seeds
#   单涵道四燃气舵    build_and_run_singleedf.sh 定高 + 手动阶跃
#
# 判据（与各机型 README 记录一致）：
#   定高   末段 |alt − 10| ≤ 0.2 m 且 unlock=1
#   手动   roll/pitch 峰值落在 4..8° 且不出现 |attitude| > 45° 的失控
#   共轴   acceptance 输出 "0 failed"
#
# 用法（TandemVec_FCS 目录，建议经 tools/with-ascii-bin.sh 包一层）：
#   bash ../tools/with-ascii-bin.sh --sub allaf -- \
#       bash test_host/fw_sil/run_all_airframes.sh
#
# 退出码：0 = 三机型全过；1 = 有机型失败（逐项打印原因）。
# ============================================================
set -u

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_sil_allaf}"
export BIN_DIR
mkdir -p "$BIN_DIR"

fail=0
declare -a RESULTS=()

# ---------- 通用判据 ----------
# $1=输出文件 $2=场景名；检查末段高度与解锁
check_hover() {
  local log="$1" tag="$2"
  local last
  last="$(grep -o 'alt=[- ]*[0-9.]*' "$log" | tail -1 | sed 's/alt=//;s/ //g')"
  local unlock
  unlock="$(grep -o 'unlock=[01]' "$log" | tail -1 | sed 's/unlock=//')"
  if [ -z "$last" ] || [ -z "$unlock" ]; then
    RESULTS+=("FAIL  $tag：无输出（编译或运行失败）")
    fail=1
    return
  fi
  local ok
  ok=$(awk -v a="$last" 'BEGIN{ x=a-10.0; if (x<0) x=-x; print (x<=0.2)?"1":"0" }')
  if [ "$ok" = "1" ] && [ "$unlock" = "1" ]; then
    RESULTS+=("PASS  $tag：稳态高度 ${last} m，超差 ≤0.2 m，已解锁")
  else
    RESULTS+=("FAIL  $tag：稳态高度 ${last} m（要求 10±0.2）或 unlock=${unlock}")
    fail=1
  fi
}

# 手动阶跃：峰值姿态必须落在 4..8°（固件姿态环对 1600µs 摇杆的既定响应）
check_step() {
  local log="$1" tag="$2"
  local peak
  peak="$(grep -o 'roll=[- ]*[0-9.]*' "$log" | sed 's/roll=//;s/ //g' \
          | awk '{v=$1; if(v<0)v=-v; if(v>m)m=v} END{printf "%.2f", m+0}')"
  if [ -z "$peak" ]; then
    RESULTS+=("FAIL  $tag：无姿态输出")
    fail=1
    return
  fi
  local ok
  ok=$(awk -v p="$peak" 'BEGIN{ print ((p>=4.0)&&(p<=8.0))?"1":"0" }')
  if [ "$ok" = "1" ]; then
    RESULTS+=("PASS  $tag：roll 峰值 ${peak}°（4..8° 窗口内，未发散）")
  else
    RESULTS+=("FAIL  $tag：roll 峰值 ${peak}°（期望 4..8°，疑似方向错或发散）")
    fail=1
  fi
}

echo "======== 1/3 纵列双发（基型） ========"
if bash test_host/fw_sil/build_and_run.sh > "$BIN_DIR/tandemvec_hover.log" 2>&1; then
  check_hover "$BIN_DIR/tandemvec_hover.log" "纵列·定高"
else
  RESULTS+=("FAIL  纵列·定高：构建或运行失败"); fail=1
fi
if bash test_host/fw_sil/build_and_run.sh --manual > "$BIN_DIR/tandemvec_manual.log" 2>&1; then
  check_step "$BIN_DIR/tandemvec_manual.log" "纵列·手动阶跃"
else
  RESULTS+=("FAIL  纵列·手动阶跃：构建或运行失败"); fail=1
fi

echo "======== 2/3 共轴双桨 VTVL ========"
if bash test_host/fw_sil/run_coaxial_acceptance.sh > "$BIN_DIR/coax_accept.log" 2>&1; then
  failed_n="$(grep -o 'real-nav acceptance: [0-9]* cases, [0-9]* failed' "$BIN_DIR/coax_accept.log" \
              | tail -1 | grep -o '[0-9]* failed' | grep -o '[0-9]*')"
  if [ "${failed_n:-1}" = "0" ]; then
    RESULTS+=("PASS  共轴·验收矩阵：0 failed")
  else
    RESULTS+=("FAIL  共轴·验收矩阵：${failed_n:-未知} failed")
    fail=1
  fi
else
  RESULTS+=("FAIL  共轴·验收矩阵：构建或运行失败"); fail=1
fi

echo "======== 3/3 单涵道四燃气舵 ========"
if bash test_host/fw_sil/build_and_run_singleedf.sh > "$BIN_DIR/sedf_hover.log" 2>&1; then
  check_hover "$BIN_DIR/sedf_hover.log" "单涵道·定高"
  thr="$(grep -o 'thr=[0-9.]*%' "$BIN_DIR/sedf_hover.log" | tail -1)"
  RESULTS+=("INFO  单涵道·悬停油门 ${thr:-未知}（解析期望 32.2%）")
else
  RESULTS+=("FAIL  单涵道·定高：构建或运行失败"); fail=1
fi
if bash test_host/fw_sil/build_and_run_singleedf.sh --manual > "$BIN_DIR/sedf_manual.log" 2>&1; then
  check_step "$BIN_DIR/sedf_manual.log" "单涵道·手动阶跃"
else
  RESULTS+=("FAIL  单涵道·手动阶跃：构建或运行失败"); fail=1
fi

echo
echo "======== 汇总 ========"
for line in "${RESULTS[@]}"; do
  echo "$line"
done
echo
if [ "$fail" = "0" ]; then
  echo "=== 三机型 SIL 全部通过 ==="
else
  echo "=== 存在失败项，见上方 FAIL 行 ==="
fi
exit "$fail"
