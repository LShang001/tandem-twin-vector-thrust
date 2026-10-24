#!/usr/bin/env bash
# CCAEGT 实飞参考制导 + 本机推进模型：20/30/40 m 回收包线。
# 仅参考测试，不接入固件飞行模式；参数对齐 CCAEGT M0=16.5 kg。
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_bin}"
CXX="${CXX:-g++}"
mkdir -p "$BIN_DIR"
BIN="$BIN_DIR/lrccaegt_sweep"
"$CXX" -std=c++17 -O2 -I"$ROOT/include" -I"$ROOT/test_host/reference/ccaegt" \
  "$ROOT/test_host/test_liquid_ccaegt_reference.cpp" \
  "$ROOT/test_host/reference/ccaegt/CCAEGT.cpp" -o "$BIN"
EXE="$BIN"
[ -f "$BIN.exe" ] && EXE="$BIN.exe"
fail=0
for h in 20 30 40; do
  if "$EXE" --h0 "$h" --prop 4.0 >/tmp/lrccaegt_case.log 2>&1; then
    printf '[PASS] h0=%s  %s\n' "$h" "$(grep 'CCAEGT-SIL' /tmp/lrccaegt_case.log | tail -1)"
  else
    printf '[FAIL] h0=%s\n' "$h"
    cat /tmp/lrccaegt_case.log
    fail=1
  fi
done
exit "$fail"
