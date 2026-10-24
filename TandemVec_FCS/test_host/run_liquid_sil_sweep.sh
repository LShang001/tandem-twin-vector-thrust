#!/usr/bin/env bash
# 液体火箭闭环 SIL 参数扫描：初始高度/推进剂/初始姿态 9 组。
# L1 证据：模型+喷管几何+简单制导，不是实机/台架。
set -eu
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_bin}"
CXX="${CXX:-g++}"
mkdir -p "$BIN_DIR"
BIN="$BIN_DIR/lrsil_sweep"
[ -f "$BIN.exe" ] && EXE="$BIN.exe" || EXE="$BIN"
"$CXX" -std=c++17 -O2 -I"$ROOT/include" -I"$ROOT/src" \
  "$ROOT/test_host/test_liquid_rocket_sil.cpp" -o "$BIN"
fail=0
for h in 20 30 40; do
  for p in 2.5 3.0 4.0; do
    if "$EXE" --h0 "$h" --prop "$p" --roll0 3 --pitch0 -2 >/tmp/lrsil_case.log 2>&1; then
      printf '[PASS] h0=%s prop=%s  %s\n' "$h" "$p" "$(grep -o 'v=[0-9.]*' /tmp/lrsil_case.log | head -1)"
    else
      printf '[FAIL] h0=%s prop=%s\n' "$h" "$p"
      cat /tmp/lrsil_case.log
      fail=1
    fi
  done
done
exit "$fail"
