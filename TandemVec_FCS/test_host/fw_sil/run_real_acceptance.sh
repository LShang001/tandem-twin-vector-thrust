#!/usr/bin/env bash
# 真实导航任务 SIL 多配置验收：先构建，再跑 clean/nominal/degraded × 3 seeds。
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil_real}"
OPT="${OPT:--O2}"
CXX="${CXX:-g++}"
CC="${CC:-gcc}"
mkdir -p "$BIN_DIR"

BIN_DIR="$BIN_DIR" OPT="$OPT" CXX="$CXX" CC="$CC" \
  /bin/bash "$ROOT/test_host/fw_sil/build_and_run_real.sh" --build-only

BIN="$BIN_DIR/fw_real"
[ -f "$BIN.exe" ] && BIN="$BIN.exe"
# Python 解释器选择（与 run_failsafe_matrix.sh 同口径）：
# Windows 上优先 py -3.12，WSL/Git Bash 回退 python3/python。
PYTHON_BIN="${PYTHON_BIN:-}"
if [ -z "$PYTHON_BIN" ]; then
  if command -v py >/dev/null 2>&1; then
    PYTHON_BIN="py -3.12"
  elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
  elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN="python"
  else
    echo "需要 python（或设置 PYTHON_BIN=...）" >&2
    exit 1
  fi
fi
exec $PYTHON_BIN "$ROOT/test_host/fw_sil/run_real_acceptance.py" --bin "$BIN" "$@"
