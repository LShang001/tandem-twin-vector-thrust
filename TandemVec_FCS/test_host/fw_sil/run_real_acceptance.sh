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
exec py -3.12 "$ROOT/test_host/fw_sil/run_real_acceptance.py" --bin "$BIN" "$@"
