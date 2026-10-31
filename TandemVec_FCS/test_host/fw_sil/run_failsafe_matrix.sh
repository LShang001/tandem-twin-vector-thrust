#!/usr/bin/env bash
# 断链失控保护真实导航 SIL 矩阵（6DOF plant + 真实 EKF/GNSS/气压 + 风扰）
#
# 用途：验证 CRSF 断链后「保持解锁 + AUTO_ALTITUDE 定高」在真实导航闭环中成立，
#       并与修复前源码（git 历史 worktree）做同命令对照。
#
# 用法（TandemVec_FCS 目录）：
#   BIN_DIR=/c/.../tv_fcs_sil bash test_host/fw_sil/run_failsafe_matrix.sh
#   ALTS=10,20 SEEDS=1,2 BIN_DIR=... bash test_host/fw_sil/run_failsafe_matrix.sh
#
# 判据（每个场景）：
#   1) max_desc_vz_down < 0.6 m/s   —— 断链段不得以 1 m/s 持续下降（旧实现 -1 m/s 指令）
#   2) final_alt > 0.85 * 目标高度  —— 不得掉到地面附近
#   3) land_t < 0                   —— plant 不得触地
#   4) gnss_crej == 0               —— 断链不应连带触发 GNSS 一致性拒绝
# 退出码：0 = 全部通过；1 = 有失败项。
set -eu

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
BIN_DIR="${BIN_DIR:-${TMPDIR:-/tmp}/tv_fcs_sil_matrix}"
OUT="${OUT:-$BIN_DIR/matrix}"
ALTS="${ALTS:-10,20,30}"
SEEDS="${SEEDS:-1,2,3}"
LOSSI="${LOSSI:-45:52}"
SECONDS_SIM="${SECONDS_SIM:-62}"
mkdir -p "$OUT"

PYTHON_BIN="${PYTHON:-}"
if [ -z "$PYTHON_BIN" ]; then
  if command -v python >/dev/null 2>&1; then
    PYTHON_BIN="python"
  elif command -v py >/dev/null 2>&1; then
    PYTHON_BIN="py -3.12"
  else
    echo "需要 python（或设置 PYTHON=...）" >&2
    exit 2
  fi
fi

echo "---- 编译 real SIL ----"
BIN_DIR="$BIN_DIR" bash test_host/fw_sil/build_and_run_real.sh --build-only >/dev/null

EXE="$BIN_DIR/fw_real"
[ -f "$EXE.exe" ] && EXE="$EXE.exe"

IFS=',' read -r -a alt_arr <<< "$ALTS"
IFS=',' read -r -a seed_arr <<< "$SEEDS"
CASES="${CASES:-loss,relink,combo}"
IFS=',' read -r -a case_arr <<< "$CASES"

for alt in "${alt_arr[@]}"; do
  for seed in "${seed_arr[@]}"; do
    for case_name in "${case_arr[@]}"; do
      if [ "$case_name" = "loss" ]; then
        rc="--rc-loss $LOSSI"
        extra="--gnss-outage 1000:1001 --no-wind"
        secs="$SECONDS_SIM"
      elif [ "$case_name" = "relink" ]; then
        # 复链：断链 3 s 后恢复（真实 linkUpCallback 路径）
        rc="--rc-loss 45:48"
        extra="--gnss-outage 1000:1001 --no-wind"
        secs="$SECONDS_SIM"
      else
        # combo：断链 20–27 s 与侧风 20–25 s、GNSS 失联 20–25 s 三重叠加
        rc="--rc-loss 20:27"
        extra="--gnss-outage 20:25"
        secs=40
      fi
      out="$OUT/${case_name}_alt${alt}_seed${seed}.log"
      # shellcheck disable=SC2086
      "$EXE" --real-nav --alt "$alt" --seconds "$secs" $rc $extra --seed "$seed" > "$out" 2>&1 || true
      json="$(grep '^SIL_REAL_JSON ' "$out" | tail -1 || true)"
      if [ -n "$json" ]; then
        printf '%s\n' "${json#SIL_REAL_JSON }" > "$OUT/${case_name}_alt${alt}_seed${seed}.json"
      fi
    done
  done
done

set +e
python_out="$(
  # shellcheck disable=SC2086
  $PYTHON_BIN - "$OUT" "$ALTS" "$SEEDS" "$CASES" <<'PY'
import csv, json, os, sys

out, alts_s, seeds_s, cases_s = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
alts = [float(x) for x in alts_s.split(',')]
seeds = [int(x) for x in seeds_s.split(',')]
cases = [c for c in cases_s.split(',') if c]
rows, fail = [], 0
for alt in alts:
    for seed in seeds:
        for case in cases:
            p = os.path.join(out, f'{case}_alt{alt:g}_seed{seed}.json')
            if not os.path.exists(p):
                rows.append([alt, seed, case, '', '', '', '', 'NO_JSON'])
                fail += 1
                continue
            d = json.load(open(p))
            desc = d.get('max_desc_vz_down', 0.0)
            final_alt = d.get('final_alt', 0.0)
            land_t = d.get('land_t', -1.0)
            crej = d.get('gnss_crej', 0)
            if case == 'combo':
                # 断链 + 侧风 + GNSS 失联叠加：断链发生在爬升途中（20 s），
                # 不能用目标高度当基准；判据 = 不得持续下降/触地，且相对断链时刻
                # 高度不得显著掉高（>3 m）。
                ref = d.get('alt_at_outage', -1.0)
                ok = (desc < 0.7) and (land_t < 0)
                if ref > 0.5:
                    ok = ok and (final_alt >= ref - 3.0)
            else:
                ok = (desc < 0.6) and (final_alt > 0.85 * alt) and (land_t < 0) and (crej == 0)
            if not ok:
                fail += 1
            rows.append([alt, seed, case, f'{desc:.3f}', f'{final_alt:.2f}',
                         f'{land_t:.2f}', crej, 'PASS' if ok else 'FAIL'])
with open(os.path.join(out, 'summary.csv'), 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['alt', 'seed', 'case', 'max_desc_vz', 'final_alt', 'land_t',
                'gnss_crej', 'verdict'])
    w.writerows(rows)
print(f'场景数={len(rows)} 失败={fail}')
for r in rows:
    print(','.join(str(x) for x in r))
sys.exit(0 if fail == 0 else 1)
PY
)"
python_rc=$?
set -e

printf '%s\n' "$python_out"
if [ "$python_rc" -eq 0 ] && printf '%s' "$python_out" | grep -q '失败=0'; then
  echo "=== 断链矩阵全部通过 ==="
  exit 0
fi
echo "=== 断链矩阵存在失败项 ==="
exit 1
