#!/usr/bin/env bash
# 共轴 VTVL：真实固件（BFS_COAXIAL_VTVL）+ 近似 6DOF plant 验收。
# 只验证固件机型分支/输出映射/姿态与定高；不是共轴台架或实机证据。
#
# 用法（在 TandemVec_FCS 目录下）：
#   bash test_host/fw_sil/run_coaxial_acceptance.sh            # 8 profiles × 3 seeds
#   bash test_host/fw_sil/run_coaxial_acceptance.sh --full     # 8 profiles × 5 seeds
#   bash test_host/fw_sil/run_coaxial_acceptance.sh --profiles mag_clean,mag_nominal
#   bash test_host/fw_sil/run_coaxial_acceptance.sh --seeds 1,2,3,4,5 --seconds 40
#   bash test_host/fw_sil/run_coaxial_acceptance.sh --bin /path/fw_coax.exe  # 复用已构建二进制
#   bash test_host/fw_sil/run_coaxial_acceptance.sh --append-args=--coax-realistic
# 其余参数原样透传给 run_real_acceptance.py（如 --seeds/--profiles/--seconds/
# --gain-scale 等；同名参数以命令行后者为准）。
#
# 8 个 profile 在共轴二进制（-DBFS_COAXIAL_VTVL -DFW_SIL_REAL_NAV）中的语义：
#   mag_clean / mag_nominal   磁航向融合 + 位置保持（主验收；标题里的“磁基线”）。
#   mag_disturbed             磁干扰下门控拒绝（mag_gate>=1）+ 高度/姿态有界；
#                             不判位置保持（同时失去绝对航向与水平辅助时不可保）。
#   nomag_degraded            无磁主动降级 AUTO_POSITION→AUTO_ALTITUDE
#                             （要求 pos_degraded=1），只判高度/姿态。
#   imu_stuck / imu_nan       IMU 卡死 / NaN 注入（10–11 s），只验证检测隔离
#                             （imu_detected=1，skip_motion），不判故障后飞行品质。
#   imu_drop                  IMU 10–11 s 丢包，同样只验证检测隔离。
#   gnss_spoof                GNSS 0.5 m/s 斜坡欺骗 + 一致性拒绝，只验证
#                             gnss_crej>=1（skip_motion）。
# 这些故障/门控路径都在固件共用的导航与传感器注入层，与纵列/共轴机型分支无关；
# 2026-10-30 单 seed 实测 8 个 profile 的 SIL_REAL_JSON 均能在共轴二进制中解析，
# 且 mag_gate / pos_degraded / imu_detected / gnss_crej 按上述语义置位，故 8 个
# 全部保留，无移出项。未纳入默认矩阵的相邻 profile 及原因：
#   mag_clean_flow  光流只做过纵列诊断，共轴未接光流验收（避免伪造覆盖）；
#   mag_gate        与 mag_disturbed 的降级门控场景重叠，纵列单列、共轴不重复；
#   clean/nominal/degraded  无磁基线仅用于观测性诊断，不作验收（见 README）。
#
# 说明：本脚本历史上默认仅 3 profiles × 3 seeds；现按纵列同级扩展为 8×3，
# --full 为 8×5。验收门阈值由 run_real_acceptance.py 统一给出，本脚本不改阈值。
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil_coax}"
mkdir -p "$BIN_DIR"

PROFILES_DEFAULT="mag_clean,mag_nominal,mag_disturbed,nomag_degraded,imu_stuck,imu_nan,imu_drop,gnss_spoof"

bin_path=""
full=0
passthrough=()

usage() {
  sed -n '4,17p' "$0" | sed 's/^# \{0,1\}//'
}

while [ $# -gt 0 ]; do
  case "$1" in
    --full) full=1; shift ;;
    --bin) bin_path="${2:?--bin 需要路径}"; shift 2 ;;
    --bin=*) bin_path="${1#--bin=}"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) passthrough+=("$1"); shift ;;
  esac
done

if [ -z "$bin_path" ]; then
  echo "[coax-acceptance] 构建共轴 SIL 二进制（BIN_DIR=$BIN_DIR）..."
  BIN_DIR="$BIN_DIR" /bin/bash "$ROOT/test_host/fw_sil/build_and_run_coaxial.sh" --build-only
  bin_path="$BIN_DIR/fw_coax"
fi
[ -f "$bin_path.exe" ] && bin_path="$bin_path.exe"
if [ ! -f "$bin_path" ]; then
  echo "[FAIL] 二进制不存在：$bin_path" >&2
  exit 2
fi

args=(py -3.12 "$ROOT/test_host/fw_sil/run_real_acceptance.py"
      --bin "$bin_path"
      --profiles "$PROFILES_DEFAULT"
      --seeds "1,2,3")
if [ "$full" -eq 1 ]; then
  # --full：核心与扩展 profile 一律 5 seeds（8×5=40 例）
  args+=(--seeds "1,2,3,4,5")
fi
# 用户显式参数放在最后，argparse 后者覆盖前者。
args+=(${passthrough[@]+"${passthrough[@]}"})

echo "[coax-acceptance] binary=$bin_path"
echo "[coax-acceptance] 默认 8 profiles × 3 seeds（--full 为 5 seeds）；显式 --profiles/--seeds 覆盖默认"
exec "${args[@]}"
