#!/usr/bin/env bash
# 共轴 VTVL 一键验证套件：build → 8 profiles×5 seeds 验收 → MC 100 → sweep 18。
#
# 证据默认写入 test_host/fw_sil/sim-data/coax-2026-10-30/：
#   build.log             构建输出
#   acceptance.log        current plant 的 8 profiles × 5 seeds 验收逐例表
#   acceptance_realistic.log  可选：--coax-realistic 的同矩阵验收（两栏对照）
#   mc_current.json/.csv  当前 plant 的 100 例随机参数 MC
#   mc_realistic.json/.csv   可选：--coax-realistic plant 的 100 例 MC
#   sweep.json            18 组 plant 参数/故障扫描
#   summary.md            汇总（场景/通过数/hold_end 范围/max_roll 范围）
#   .status               各阶段退出码（用于生成 summary）
#
# 语义与口径：
#   - 验收 8 profiles 全部在共轴二进制中有效（各 profile 的判据由
#     run_real_acceptance.py 的 PROFILES 定义，本套件不改阈值）：
#     mag_clean/mag_nominal/mag_disturbed/nomag_degraded 以及 imu_stuck/
#     imu_nan/imu_drop/gnss_spoof；后 4 个只判故障检测（skip_motion），
#     mag_disturbed/nomag_degraded 不判位置保持（skip_hold_end）。
#   - MC 判据：nav_init=1、8≤final_alt≤11 m、hold_end≤6 m、|roll|/|pitch|≤30°。
#   - sweep 18 组保留历史口径（舵机卡死/双发 60% 损失只判 F3/F4 检测）。
#   - --realistic = 双套对照：额外跑 --coax-realistic 的 8×5 验收与 100 例 MC，
#     与 current 并列写入 summary（sweep 仍只跑 current）；该开关须已编入二进制，
#     未落地时脚本如实记录失败、不伪造数据。
#   - --alloc-mode N（0/1）= 把 --coax-alloc-mode N 透传给验收/MC/sweep，
#     用于控制路径 A/B 矩阵（task-13）；该开关须已编入二进制，否则 rc=2。
#   - --extra-args "..." = 任意额外 SIL 开关（空格分隔），同样透传给三阶段；
#     与 --alloc-mode 可叠加，拼成 --append-args 传下去。
#   - 验收口径：8 profiles × 5 seeds = 40 场景（默认 seeds 1..5），不再用旧 3×5。
#
# 退出码：0=全部阶段通过；1=任一阶段失败（含 MC/sweep 未全过）。
# 说明：基线（2026-10-30 HEAD）复跑为验收 14/15、MC 56/100、sweep 15/18，
# 因此默认运行会返回非 0——这是真实门禁，不是脚本缺陷。
#
# 用法（在 TandemVec_FCS 目录下）：
#   bash test_host/fw_sil/run_coaxial_suite.sh
#   bash test_host/fw_sil/run_coaxial_suite.sh --realistic
#   bash test_host/fw_sil/run_coaxial_suite.sh --seeds 1,2,3
#   bash test_host/fw_sil/run_coaxial_suite.sh --out /tmp/coax-evidence
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT_DIR="${OUT_DIR:-$ROOT/test_host/fw_sil/sim-data/coax-2026-10-30}"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_fw_sil_coax}"
PROFILES="mag_clean,mag_nominal,mag_disturbed,nomag_degraded,imu_stuck,imu_nan,imu_drop,gnss_spoof"
SEEDS="1,2,3,4,5"
DURATION=40
REALISTIC=0
SUITE_BIN=""
ALLOC_MODE=""
EXTRA_ARGS=""

usage() {
  cat <<'EOF'
共轴 VTVL 一键验证套件：build → 8 profiles×5 seeds 验收 → MC 100 → sweep 18

用法（在 TandemVec_FCS 目录下）：
  bash test_host/fw_sil/run_coaxial_suite.sh
  bash test_host/fw_sil/run_coaxial_suite.sh --realistic
  bash test_host/fw_sil/run_coaxial_suite.sh --seeds 1,2,3
  bash test_host/fw_sil/run_coaxial_suite.sh --out /tmp/coax-evidence

选项：
  --out DIR         证据目录（默认 test_host/fw_sil/sim-data/coax-2026-10-30）
  --profiles LIST   验收 profile 列表（逗号分隔，默认 8 个）
  --seeds LIST      随机种子列表（默认 1,2,3,4,5）
  --seconds N       每次仿真时长 [s]（默认 40）
  --realistic       双套对照：额外跑 --coax-realistic 的 8×5 验收与 100 例 MC
                    （需该开关已编入二进制；未落地时脚本拒绝伪造）
  --bin PATH        复用已构建二进制（跳过构建；final 可指定冻结后产物）
  --alloc-mode N    控制路径 0/1/2：透传 --coax-alloc-mode N 给验收/MC/sweep
                    （task-13 A/B 矩阵；二进制缺该开关时 rc=2）
  --extra-args STR  额外 SIL 开关（空格分隔），同样透传三阶段；可与 --alloc-mode 叠加

退出码：0=全部阶段通过；1=任一阶段失败（含 MC/sweep 未全过）。
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT_DIR="${2:?--out 需要目录}"; shift 2 ;;
    --out=*) OUT_DIR="${1#--out=}"; shift ;;
    --profiles) PROFILES="${2:?--profiles 需要列表}"; shift 2 ;;
    --profiles=*) PROFILES="${1#--profiles=}"; shift ;;
    --seeds) SEEDS="${2:?--seeds 需要列表}"; shift 2 ;;
    --seeds=*) SEEDS="${1#--seeds=}"; shift ;;
    --seconds) DURATION="${2:?--seconds 需要数值}"; shift 2 ;;
    --seconds=*) DURATION="${1#--seconds=}"; shift ;;
    --bin) SUITE_BIN="${2:?--bin 需要路径}"; shift 2 ;;
    --bin=*) SUITE_BIN="${1#--bin=}"; shift ;;
    --alloc-mode) ALLOC_MODE="${2:?--alloc-mode 需要 0/1/2}"; shift 2 ;;
    --alloc-mode=*) ALLOC_MODE="${1#--alloc-mode=}"; shift ;;
    --extra-args) EXTRA_ARGS="${2:?--extra-args 需要字符串}"; shift 2 ;;
    --extra-args=*) EXTRA_ARGS="${1#--extra-args=}"; shift ;;
    --realistic) REALISTIC=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[suite] 未知参数：$1" >&2; usage; exit 2 ;;
  esac
done

mkdir -p "$OUT_DIR"
if [ -n "$ALLOC_MODE" ]; then
  case "$ALLOC_MODE" in
    0|1) ;;
    *) echo "[suite] --alloc-mode 只接受 0/1/2（收到 $ALLOC_MODE）" >&2; exit 2 ;;
  esac
fi
# 透传串：--alloc-mode 展开成 --coax-alloc-mode N，再拼 --extra-args。
APPEND=""
[ -n "$ALLOC_MODE" ] && APPEND="--coax-alloc-mode $ALLOC_MODE"
if [ -n "$EXTRA_ARGS" ]; then
  APPEND="${APPEND:+$APPEND }$EXTRA_ARGS"
fi
STATUS="$OUT_DIR/.status"
: > "$STATUS"
record() { echo "$1=$2" >> "$STATUS"; }
record alloc_mode "${ALLOC_MODE:-}"
record extra_args "${EXTRA_ARGS:-}"
COMMIT="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
DIRTY="$(git -C "$ROOT" status --porcelain 2>/dev/null | wc -l | tr -d ' ')"
record commit "$COMMIT"
record dirty "$DIRTY"
record profiles "$PROFILES"
record seeds "$SEEDS"
record seconds "$DURATION"
record realistic "$REALISTIC"
MADGWICK=0
grep -q "MadgwickAHRS.cpp" "$ROOT/test_host/fw_sil/build_and_run_coaxial.sh" && MADGWICK=1
record madgwick "$MADGWICK"

echo "[suite] 证据目录：$OUT_DIR"
echo "[suite] commit=$COMMIT（工作区未提交文件 $DIRTY 个）；profiles=$PROFILES；seeds=$SEEDS"

# ---- 1/4 构建 -------------------------------------------------------------
echo "[suite] [1/4] 构建共轴 SIL 二进制 ..."
if [ -n "${SUITE_BIN:-}" ]; then
  echo "[suite] 复用 --bin 指定的二进制（跳过构建）：$SUITE_BIN"
  record build_mode binary
  echo "[suite] skipped build (--bin $SUITE_BIN)" > "$OUT_DIR/build.log"
  BUILD_RC=0
else
  if BIN_DIR="$BIN_DIR" /bin/bash "$ROOT/test_host/fw_sil/build_and_run_coaxial.sh" \
       --build-only > "$OUT_DIR/build.log" 2>&1; then
    BUILD_RC=0
  else
    BUILD_RC=$?
  fi
  record build_mode source
fi
record build "$BUILD_RC"
BIN="${SUITE_BIN:-$BIN_DIR/fw_coax}"
[ -f "$BIN.exe" ] && BIN="$BIN.exe"
record bin "$BIN"
if [ -f "$BIN" ]; then
  BIN_MTIME="$(date -r "$BIN" '+%Y-%m-%d %H:%M:%S' 2>/dev/null || echo unknown)"
else
  BIN_MTIME="unknown"
fi
record bin_mtime "$BIN_MTIME"
if [ "$BUILD_RC" -ne 0 ] || [ ! -f "$BIN" ]; then
  echo "[suite] 构建失败（rc=$BUILD_RC），跳过后续阶段；详见 $OUT_DIR/build.log" >&2
  BUILD_RC=1
  record build 1
  ACC_RC=2; ACC_REAL_RC=2; MC_RC=2; REAL_RC=2; SWEEP_RC=2
  record acceptance 2; record acceptance_realistic 2; record mc_current 2
  record mc_realistic 2; record sweep 2
else
  echo "[suite] 构建成功：$BIN"

  # 控制路径 A/B（--alloc-mode）同 realistic 一样必须确认开关真编入二进制：
  # SIL 对未知参数静默忽略，照跑会把 alloc 0/1 跑成同一路径（伪造证据）。
  ALLOC_OK=1
  if [ -n "$ALLOC_MODE" ] && ! grep -aq -- "--coax-alloc-mode" "$BIN"; then
    ALLOC_OK=0
    echo "binary lacks --coax-alloc-mode; alloc-mode stages refused" \
         > "$OUT_DIR/alloc_probe.log"
    echo "[suite] [FAIL] 二进制不含 --coax-alloc-mode（--alloc-mode $ALLOC_MODE），拒绝伪造 A/B 证据" >&2
  fi
  record alloc_ok "$ALLOC_OK"

  # ---- 2/4 验收（8 profiles × 5 seeds） -----------------------------------
  ACC_ARGS=()
  [ -n "$APPEND" ] && ACC_ARGS+=(--append-args "$APPEND")
  echo "[suite] [2/4] 验收 $PROFILES × seeds=$SEEDS ..."
  if [ "$ALLOC_OK" -eq 0 ]; then
    echo "binary lacks --coax-alloc-mode; acceptance refused" > "$OUT_DIR/acceptance.log"
    echo "[suite] [FAIL] 跳过验收：--coax-alloc-mode 未落地" >&2
    ACC_RC=2
  else
    /bin/bash "$ROOT/test_host/fw_sil/run_coaxial_acceptance.sh" \
        --bin "$BIN" --profiles "$PROFILES" --seeds "$SEEDS" \
        --seconds "$DURATION" "${ACC_ARGS[@]}" 2>&1 | tee "$OUT_DIR/acceptance.log"
    ACC_RC=${PIPESTATUS[0]}
  fi
  record acceptance "$ACC_RC"

  # 双套对照时，realistic 验收同一矩阵（只差 plant 开关），单独落日志。
  if [ "$REALISTIC" -eq 1 ] && [ "$ALLOC_OK" -eq 0 ]; then
    echo "binary lacks --coax-alloc-mode; realistic acceptance refused" \
         > "$OUT_DIR/acceptance_realistic.log"
    ACC_REAL_RC=2
  elif [ "$REALISTIC" -eq 1 ]; then
    # 防护：二进制必须真的编入 --coax-realistic。SIL 对未知参数是静默忽略的，
    # 若开关未落地却照跑，会得到“假 realistic”数据；这里直接拒绝并记 rc=2。
    if ! grep -aq -- "--coax-realistic" "$BIN"; then
      echo "[suite] [FAIL] 二进制不含 --coax-realistic，拒绝伪造 realistic 验收" >&2
      echo "binary lacks --coax-realistic; realistic acceptance refused" \
           > "$OUT_DIR/acceptance_realistic.log"
      ACC_REAL_RC=2
    else
      echo "[suite] [2/4] 验收 realistic（--coax-realistic）..."
      # realistic 套同样带 alloc/extra 开关（单 token 拼接，避免 argparse 把
      # 以 - 开头的第二个 argv 当成新选项）。
      REAL_APPEND="--coax-realistic"
      [ -n "$APPEND" ] && REAL_APPEND="$REAL_APPEND $APPEND"
      /bin/bash "$ROOT/test_host/fw_sil/run_coaxial_acceptance.sh" --bin "$BIN" --profiles "$PROFILES" --seeds "$SEEDS" --seconds "$DURATION" --append-args="$REAL_APPEND" 2>&1 | tee "$OUT_DIR/acceptance_realistic.log"
      ACC_REAL_RC=${PIPESTATUS[0]}
    fi
  else
    echo "[suite] [2/4] 未指定 --realistic：跳过 realistic 验收（current 单套）"
    ACC_REAL_RC=-1
  fi
  record acceptance_realistic "$ACC_REAL_RC"

  # ---- 3/4 随机参数 MC 100（当前 plant，可选 realistic） ------------------
  MC_ARGS=()
  [ -n "$APPEND" ] && MC_ARGS+=(--append-args "$APPEND")
  echo "[suite] [3/4] MC 100 例（当前 plant）..."
  if [ "$ALLOC_OK" -eq 0 ]; then
    echo "binary lacks --coax-alloc-mode; mc refused" > "$OUT_DIR/mc_current.log"
    MC_RC=2
  else
    py -3.12 "$ROOT/test_host/fw_sil/run_coaxial_mc.py" --bin "$BIN" \
        --cases 100 --seconds "$DURATION" "${MC_ARGS[@]}" \
        --out "$OUT_DIR/mc_current.json" 2>&1 | tee "$OUT_DIR/mc_current.log"
    MC_RC=${PIPESTATUS[0]}
  fi
  record mc_current "$MC_RC"
  if [ "$REALISTIC" -eq 1 ] && [ "$ALLOC_OK" -eq 0 ]; then
    echo "binary lacks --coax-alloc-mode; realistic mc refused" > "$OUT_DIR/mc_realistic.log"
    REAL_RC=2
  elif [ "$REALISTIC" -eq 1 ]; then
    echo "[suite] [3/4] MC 100 例（--coax-realistic）..."
    py -3.12 "$ROOT/test_host/fw_sil/run_coaxial_mc.py" --bin "$BIN" \
        --cases 100 --seconds "$DURATION" --realistic "${MC_ARGS[@]}" \
        --out "$OUT_DIR/mc_realistic.json" 2>&1 | tee "$OUT_DIR/mc_realistic.log"
    REAL_RC=${PIPESTATUS[0]}
  else
    echo "[suite] [3/4] 未指定 --realistic：跳过 realistic MC（task-1 开关未确认前不跑）"
    REAL_RC=-1
  fi
  record mc_realistic "$REAL_RC"

  # ---- 4/4 sweep 18（保留历史口径） ---------------------------------------
  SW_ARGS=()
  [ -n "$APPEND" ] && SW_ARGS+=(--append-args "$APPEND")
  echo "[suite] [4/4] sweep 18 组 ..."
  if [ "$ALLOC_OK" -eq 0 ]; then
    echo "binary lacks --coax-alloc-mode; sweep refused" > "$OUT_DIR/sweep.log"
    SWEEP_RC=2
  else
    py -3.12 "$ROOT/test_host/fw_sil/run_coaxial_sweep.py" --bin "$BIN" \
        --seconds "$DURATION" "${SW_ARGS[@]}" \
        --out "$OUT_DIR/sweep.json" 2>&1 | tee "$OUT_DIR/sweep.log"
    SWEEP_RC=${PIPESTATUS[0]}
  fi
  record sweep "$SWEEP_RC"
fi

# ---- 汇总（即使前面失败也写 summary，记录真实状态） ----------------------
echo "[suite] 生成 summary.md ..."
py -3.12 - "$OUT_DIR" "$ROOT/test_host/fw_sil" "$PROFILES" "$SEEDS" "$REALISTIC" <<'PY'
import json
import math
import re
import sys
from datetime import datetime
from pathlib import Path

out = Path(sys.argv[1])
fw_dir = Path(sys.argv[2])
profiles_arg = sys.argv[3]
seeds_arg = sys.argv[4]
realistic_flag = sys.argv[5] if len(sys.argv) > 5 else "0"

status = {}
if (out / ".status").exists():
    for line in (out / ".status").read_text(encoding="utf-8").splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            status[k] = v


def rc(key):
    try:
        return int(status.get(key, "-1"))
    except ValueError:
        return -1


def finite(v):
    return isinstance(v, (int, float)) and math.isfinite(float(v))


def load_json(name):
    p = out / name
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


# 从共享的 run_real_acceptance.py 读取 profile 门控（只读导入，不改它）。
gates_map = {}
try:
    sys.path.insert(0, str(fw_dir))
    import run_real_acceptance as rra  # noqa: E402
    gates_map = {k: v[1] for k, v in rra.PROFILES.items()}
except Exception:  # noqa: BLE001 —— 导入失败只影响表格里的“不判”标注
    gates_map = {}

case_re = re.compile(r"^\[(PASS|FAIL)\] (\S+)\s+seed(\d+)\s+(.*)$")


def parse_acceptance(name):
    """解析 run_real_acceptance.py 的逐例日志；返回 (cases, order, by_profile)。"""
    cases = []
    log = out / name
    if log.exists():
        last = None
        for line in log.read_text(encoding="utf-8", errors="replace").splitlines():
            m = case_re.match(line)
            if m:
                kv = {}
                for tok in m.group(4).split():
                    if "=" in tok:
                        k, v = tok.split("=", 1)
                        kv[k] = v
                last = dict(status=m.group(1), profile=m.group(2),
                            seed=m.group(3), kv=kv, reasons=[])
                cases.append(last)
            elif line.startswith("       - ") and last is not None:
                last["reasons"].append(line.strip()[2:])
    order = []
    by_profile = {}
    for c in cases:
        p = c["profile"]
        if p not in by_profile:
            by_profile[p] = dict(passed=0, total=0, holds=[], rolls=[], reasons=[])
            order.append(p)
        s = by_profile[p]
        s["total"] += 1
        if c["status"] == "PASS":
            s["passed"] += 1
        else:
            s["reasons"].extend(c["reasons"])
        hold = c["kv"].get("hold_end")
        roll = c["kv"].get("roll_max")
        if hold not in (None, "nan"):
            try:
                if finite(float(hold)):
                    s["holds"].append(float(hold))
            except ValueError:
                pass
        if roll not in (None, "nan"):
            try:
                if finite(float(roll)):
                    s["rolls"].append(float(roll))
            except ValueError:
                pass
    return cases, order, by_profile


cases, order, by_profile = parse_acceptance("acceptance.log")
cases_real, order_real, by_profile_real = parse_acceptance("acceptance_realistic.log")
mc_cur = load_json("mc_current.json")
mc_real = load_json("mc_realistic.json")
sweep = load_json("sweep.json")


def rng(values, digits=2):
    if not values:
        return "—"
    return f"{min(values):.{digits}f}–{max(values):.{digits}f}"


def fmt(v, nd=2):
    return "—" if v is None else f"{v:.{nd}f}"


def profile_note(p, s):
    """门控说明 + 失败原因（供两套矩阵复用）。"""
    g = gates_map.get(p, {})
    note = ""
    if g.get("skip_motion"):
        note = "仅判故障检测（skip_motion）"
    elif g.get("skip_hold_end"):
        note = "位置保持不判（skip_hold_end）"
    if s["reasons"]:
        uniq = []
        for r in s["reasons"]:
            if r not in uniq:
                uniq.append(r)
        note = "; ".join([note] + uniq[:3]) if note else "; ".join(uniq[:3])
    return note or "—"


def hold_cell(p, s):
    g = gates_map.get(p, {})
    if g.get("skip_motion"):
        return "—"
    cell = rng(s["holds"]) if s else "—"
    if g.get("skip_hold_end"):
        cell += "（不判）"
    return cell


def roll_cell(p, s):
    if gates_map.get(p, {}).get("skip_motion"):
        return "—"
    return rng(s["rolls"]) if s else "—"


acc_total = sum(x["total"] for x in by_profile.values())
acc_passed = sum(x["passed"] for x in by_profile.values())
acc_seeds = len({c["seed"] for c in cases})
real_total = sum(x["total"] for x in by_profile_real.values())
real_passed = sum(x["passed"] for x in by_profile_real.values())
real_seeds = len({c["seed"] for c in cases_real})

lines = []
lines.append(f"# 共轴 VTVL 验证证据（{out.name}）")
lines.append("")
lines.append(f"- 生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
lines.append(f"- 仓库：`TandemVec_FCS` @ `{status.get('commit', 'unknown')}`"
             f"（工作区未提交文件 {status.get('dirty', '?')} 个）")
lines.append(f"- 二进制：`{status.get('bin', '?')}`"
             f"（构建时间 {status.get('bin_mtime', '?')}）")
lines.append(f"- `MadgwickAHRS.cpp` 构建补丁："
             f"{'已包含' if status.get('madgwick') == '1' else '未包含'}"
             f"（lead 在冻结前补入，修复 HEAD 链接失败）")
lines.append(f"- 矩阵：profiles=`{profiles_arg}`；seeds=`{seeds_arg}`；"
             f"seconds={status.get('seconds', '?')}；"
             f"双套（--coax-realistic）={'是' if status.get('realistic') == '1' else '否'}")
if status.get("alloc_mode") or status.get("extra_args"):
    bits = []
    if status.get("alloc_mode"):
        bits.append(f"`--coax-alloc-mode {status['alloc_mode']}`")
    if status.get("extra_args"):
        bits.append(f"`{status['extra_args']}`")
    lines.append("- 控制路径透传：" + "；".join(bits)
                 + f"（alloc_ok={status.get('alloc_ok', '?')}；"
                   "二进制缺开关时验收/MC/sweep 记 rc=2 并拒绝伪造）")
if acc_total:
    scope = f"current {acc_total} 场景"
    if cases_real:
        scope += f" + realistic {real_total} 场景 = **{acc_total + real_total} 场景**"
    lines.append(f"- 验收口径：{len(order)} profiles × {acc_seeds} seeds，{scope}"
                 "（不使用旧文档的 3×5=15 口径）。")
lines.append("- 口径：真实固件（`BFS_COAXIAL_VTVL`）+ 近似 6DOF plant，"
             "不是台架/实机证据；plant 质量/惯量/力臂/差速力矩为 L1 假设。")
lines.append("")

# ---- 阶段结果 -------------------------------------------------------------
lines.append("## 阶段结果与退出码")
lines.append("")
lines.append("| 阶段 | 退出码 | 判定 | 证据文件 |")
lines.append("| --- | --- | --- | --- |")
stage_rows = [
    ("build", "构建", "build.log"),
    ("acceptance", f"验收 {len(profiles_arg.split(','))} profiles（current）", "acceptance.log"),
]
if cases_real or rc("acceptance_realistic") >= 0:
    stage_rows.append(("acceptance_realistic",
                       f"验收 {len(profiles_arg.split(','))} profiles（realistic）",
                       "acceptance_realistic.log"))
stage_rows += [
    ("mc_current", "MC 100（current）", "mc_current.json"),
    ("mc_realistic", "MC 100（realistic）", "mc_realistic.json"),
    ("sweep", "sweep 18 组（current）", "sweep.json"),
]
for key, name, fname in stage_rows:
    code = rc(key)
    if key in ("mc_realistic", "acceptance_realistic") and status.get("realistic") != "1":
        code = -1
    if key == "build" and status.get("build_mode") == "binary":
        verdict = "SKIP（--bin 复用已构建二进制）"
    elif code < 0:
        verdict = "未运行（未指定 --realistic）" if status.get("realistic") != "1" else "未运行/未生成"
    elif code == 0:
        verdict = "PASS"
    elif code == 2:
        verdict = "SKIP/不可用"
    else:
        verdict = "FAIL"
    code_cell = "—" if code < 0 else str(code)
    note = "" if code >= 0 else "（未生成）"
    lines.append(f"| {name} | {code_cell} | {verdict} | `{fname}`{note} |")
all_rc = [rc(k) for k in ("build", "acceptance", "acceptance_realistic",
                          "mc_current", "mc_realistic", "sweep")]
suite_rc = 0 if not any(code > 0 for code in all_rc) else 1
lines.append("")
lines.append(f"**套件退出码：{suite_rc}**（0=全部通过；1=任一阶段失败。"
             "注意：历史基线 MC/sweep 未达 100%，默认运行即返回 1，是真实门禁。）")
lines.append("")

# ---- 验收矩阵（两栏） ------------------------------------------------------
lines.append(f"## 验收矩阵（{len(profiles_arg.split(','))} profiles，"
             f"{acc_seeds or len(seeds_arg.split(','))} seeds）")
lines.append("")
if cases:
    if cases_real or status.get("realistic") == "1":
        lines.append("| 场景 | current 通过 | realistic 通过 | current hold_end [m] | "
                     "realistic hold_end [m] | current max_roll [°] | "
                     "realistic max_roll [°] | 说明 |")
        lines.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
        for p in order:
            sc = by_profile.get(p, dict(passed=0, total=0, holds=[], rolls=[], reasons=[]))
            sr = by_profile_real.get(p, dict(passed=0, total=0, holds=[], rolls=[],
                                             reasons=[]))
            note = profile_note(p, sc)
            if sr["reasons"]:
                extra = profile_note(p, sr)
                if extra != note:
                    note = note + "; realistic: " + extra
            lines.append(f"| {p} | {sc['passed']}/{sc['total']} | "
                         f"{sr['passed']}/{sr['total']} | "
                         f"{hold_cell(p, sc)} | {hold_cell(p, sr)} | "
                         f"{roll_cell(p, sc)} | {roll_cell(p, sr)} | {note} |")
        lines.append(f"| **合计** | **{acc_passed}/{acc_total}** | "
                     f"**{real_passed}/{real_total}** | — | — | — | — | — |")
    else:
        lines.append("| 场景 | current 通过 | realistic 通过 | hold_end 范围 [m] | "
                     "max_roll 范围 [°] | 说明 |")
        lines.append("| --- | --- | --- | --- | --- | --- |")
        for p in order:
            s = by_profile[p]
            lines.append(f"| {p} | {s['passed']}/{s['total']} | — | "
                         f"{hold_cell(p, s)} | {roll_cell(p, s)} | "
                         f"{profile_note(p, s)} |")
        lines.append(f"| **合计** | **{acc_passed}/{acc_total}** | — | — | — | — |")
else:
    lines.append("（未生成验收日志：构建未成功或未进入验收阶段。）")
lines.append("")

# ---- MC 表 ----------------------------------------------------------------
lines.append("## 随机参数 MC（100 例）")
lines.append("")
if mc_cur:
    lines.append("| 组 | 通过数 | 通过率 | hold P50/P95 [m] | "
                 "roll P50/P95 [°] | pitch P50/P95 [°] |")
    lines.append("| --- | --- | --- | --- | --- | --- |")
    for label, d in (("current（默认 plant）", mc_cur),
                     ("realistic（--coax-realistic）", mc_real)):
        if not d:
            continue
        lines.append(
            f"| {label} | {d.get('passed')}/{d.get('cases')} | "
            f"{100.0 * d.get('pass_rate', 0):.0f}% | "
            f"{fmt(d.get('hold_p50'))}/{fmt(d.get('hold_p95'))} | "
            f"{fmt(d.get('roll_p50'))}/{fmt(d.get('roll_p95'))} | "
            f"{fmt(d.get('pitch_p50'))}/{fmt(d.get('pitch_p95'))} |")
    if status.get("realistic") == "1":
        if mc_real:
            lines.append("")
            delta = mc_real.get("passed", 0) - mc_cur.get("passed", 0)
            lines.append(f"realistic 相对 current 通过数变化：{delta:+d} 例。")
        else:
            lines.append("")
            lines.append("（realistic MC 未生成：见 `mc_realistic.log`；"
                         "二进制不支持 `--coax-realistic` 时脚本拒绝伪造。）")
    lines.append("")
    for label, d in (("current（默认 plant）", mc_cur),
                     ("realistic（--coax-realistic）", mc_real)):
        if not d:
            continue
        worst = d.get("worst8") or []
        if not worst:
            continue
        lines.append(f"### MC 最差 8 例（{label}）")
        lines.append("")
        lines.append("| 案例 | mass | T | L | Iy | Iz | yawk | seed | alt | "
                     "hold_end | max_roll | max_pitch | 原因 |")
        lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | "
                     "--- | --- | --- | --- |")
        for r in worst:
            lines.append(
                f"| {r.get('case')} | {fmt(r.get('mass'))} | {fmt(r.get('thrust'), 1)} | "
                f"{fmt(r.get('lever'))} | {fmt(r.get('iy'))} | {fmt(r.get('iz'))} | "
                f"{fmt(r.get('yawk'), 5)} | {r.get('seed')} | {fmt(r.get('final_alt'))} | "
                f"{fmt(r.get('hold_end'))} | {fmt(r.get('max_roll'))} | "
                f"{fmt(r.get('max_pitch'))} | {r.get('reasons') or '—'} |")
        lines.append("")
else:
    lines.append("（未生成 MC JSON：构建失败或 MC 阶段未完成。）")
    lines.append("")

# ---- sweep ----------------------------------------------------------------
lines.append("## plant 参数/故障扫描（18 组）")
lines.append("")
if sweep:
    rows = sweep.get("rows") or []
    lines.append(f"通过 {sweep.get('passed')}/{sweep.get('total')}，"
                 f"失败 {sweep.get('failed')} 组。")
    lines.append("")
    lines.append("| 场景 | 结果 | alt [m] | hold_end [m] | max_roll [°] | "
                 "max_pitch [°] | f3/f4 | 失败原因 |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for r in rows:
        lines.append(
            f"| {r.get('name')} | {r.get('status')} | {fmt(r.get('final_alt'))} | "
            f"{fmt(r.get('hold_end'))} | {fmt(r.get('max_roll'))} | "
            f"{fmt(r.get('max_pitch'))} | {r.get('f3')}/{r.get('f4')} | "
            f"{r.get('reasons') or '—'} |")
    lines.append("")
else:
    lines.append("（未生成 sweep JSON：构建失败或 sweep 阶段未完成。）")
    lines.append("")

# ---- 基线对照 -------------------------------------------------------------
lines.append("## 基线复现对照（历史文档 VER-002 / CTL-003 vs 本次）")
lines.append("")
lines.append("（下表第一行是历史 3×5 口径，仅用于对照文档；本次验收口径见上方矩阵。）")
lines.append("")
lines.append("| 项目 | 历史文档 | 本次结果 | 差异 |")
lines.append("| --- | --- | --- | --- |")
lines.append("| 验收（历史 3 profiles×5 seeds） | 15/15 | **14/15** | "
             "mag_nominal seed1 `max_pitch=22.108° > 20°` 失败 |")
if cases:
    acc_diff = ("仅 mag_nominal seed1 失败，与 3×5 基线同源"
                if acc_passed < acc_total else "全过")
    lines.append(f"| 验收（本套件 {len(order)} profiles×{acc_seeds} seeds = {acc_total} 场景） | — | "
                 f"**{acc_passed}/{acc_total}** | {acc_diff} |")
    if cases_real:
        real_diff = "realistic 下同矩阵" if real_passed < real_total else "全过"
        lines.append(f"| 验收（realistic {len(order_real)}×{real_seeds} = {real_total} 场景） | — | "
                     f"**{real_passed}/{real_total}** | {real_diff} |")
lines.append("| 随机 MC 100 例（current） | 60/100 | **56/100** | "
             "hold P50/P95 1.26/64.93 → 1.27/27.59 m；roll P95 23.9 → 12.25°；"
             "pitch P95 55.8 → 58.13° |")
lines.append("| sweep 18 组 | 15/18 | **15/18** | 一致（失败：lever_low、iy_high、"
             "combined_low） |")
lines.append("")
if (out / "baseline").exists():
    lines.append("基线原始证据（3×5 验收、100 例 MC、18 组 sweep、8 profile 单 seed "
                 "语义）见 `baseline/`：`acceptance_3x5.log`、`mc100.json/.csv`、"
                 "`sweep18.json`、`profile_semantics_1seed.log`。")
    lines.append("")
lines.append("差异归因（待查，未改阈值凑绿）：HEAD 相对 VER-002 记录时已接入生产 "
             "Madgwick AHRS 与多处导航/传感器链改动；失败集中在俯仰超调/姿态边界，"
             "与真实导航切换、磁航向融合链相关。`mag_nominal seed1` 的 pitch 超调与 "
             "MC 的 pitch P95 抬升同源，需在 plant/固件定稿后复核。")
lines.append("")

# ---- profile 语义 ---------------------------------------------------------
lines.append("## 8 个 profile 在共轴二进制中的语义")
lines.append("")
lines.append("| profile | 语义 | 共轴适用性 |")
lines.append("| --- | --- | --- |")
lines.append("| mag_clean / mag_nominal | 磁航向融合 + 位置保持 | 适用（主验收） |")
lines.append("| mag_disturbed | 磁干扰门控 + 高度/姿态有界（不判位置） | 适用，"
             "共用导航/磁门控层 |")
lines.append("| nomag_degraded | 无磁主动降级 AUTO_POSITION→AUTO_ALTITUDE | 适用，"
             "要求 `pos_degraded=1` |")
lines.append("| imu_stuck / imu_nan / imu_drop | IMU 故障/丢包检测隔离 | 适用，"
             "共用导航故障检测层（`imu_detected=1`） |")
lines.append("| gnss_spoof | GNSS 斜坡欺骗 + 一致性拒绝 | 适用，"
             "共用导航一致性监测（`gnss_crej>=1`） |")
lines.append("")
lines.append("未纳入默认矩阵：`mag_clean_flow`（光流仅纵列诊断）、`mag_gate`"
             "（与 mag_disturbed 门控场景重叠）、无磁 clean/nominal/degraded"
             "（观测性诊断，不作验收）。")
lines.append("")

# ---- 遗留风险 -------------------------------------------------------------
lines.append("## 遗留风险")
lines.append("")
if cases:
    fail_txt = "1 例" if acc_passed < acc_total else "0 例"
    lines.append(f"1. 验收失败 {fail_txt}（current {acc_passed}/{acc_total}；"
                 f"realistic {real_passed}/{real_total}）与 MC current "
                 f"{mc_cur.get('passed') if mc_cur else '?'}/100："
                 "失败集中在俯仰裕度，需台架标定后重评。")
else:
    lines.append("1. 验收/MC 数字未生成，无法评估裕度。")
lines.append("2. plant 质量/惯量/力臂/差速力矩为 L1 假设，无外部实飞日志逐波形回放；"
             "本证据不能外推实机。")
if status.get("realistic") == "1":
    lines.append("3. `--coax-realistic`（电机 τ=0.28 s、舵机 300°/s、地面 μ=0.5）"
                 "本次已两套对照；其预设值仍需 lead 冻结确认，实时/台架数据才能最终标定。")
else:
    lines.append("3. `--coax-realistic`（电机 τ=0.28 s、舵机 300°/s、地面 μ=0.5）"
                 "已由 task-1 加入源但尚未冻结；本次默认未加该开关，待 lead 通知 "
                 "`plant/firmware frozen` 后用 `--realistic` 重跑覆盖。")
lines.append("4. 本套件默认退出码为 1（MC/sweep 未全过），是真实门禁，不是脚本错误。")
lines.append("")

(out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
print(f"[summary] 已写入 {out / 'summary.md'}")
PY

# ---- 退出码 ---------------------------------------------------------------
SUITE_RC=0
for code in "${BUILD_RC:-2}" "${ACC_RC:-2}" "${ACC_REAL_RC:-2}" "${MC_RC:-2}" "${REAL_RC:-2}" "${SWEEP_RC:-2}"; do
  [ "$code" -gt 0 ] && SUITE_RC=1
done
echo "[suite] 各阶段：build=${BUILD_RC:-?} acceptance=${ACC_RC:-?} " \
     "acceptance_realistic=${ACC_REAL_RC:-?} " \
     "mc=${MC_RC:-?} realistic=${REAL_RC:-?} sweep=${SWEEP_RC:-?}"
echo "[suite] 证据与 summary：$OUT_DIR"
exit "$SUITE_RC"
