#!/usr/bin/env bash
# 共轴 VTVL：真实固件（BFS_COAXIAL_VTVL）+ 近似 6DOF plant 验收。
# 只验证固件机型分支/输出映射/姿态与定高；不是共轴台架或实机证据。
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil_coax}"
mkdir -p "$BIN_DIR"

BIN_DIR="$BIN_DIR" /bin/bash "$ROOT/test_host/fw_sil/build_and_run_coaxial.sh" --build-only
BIN="$BIN_DIR/fw_coax"
[ -f "$BIN.exe" ] && BIN="$BIN.exe"
exec py -3.12 "$ROOT/test_host/fw_sil/run_real_acceptance.py" --bin "$BIN" \
  --profiles mag_clean,mag_nominal,nomag_degraded "$@"
