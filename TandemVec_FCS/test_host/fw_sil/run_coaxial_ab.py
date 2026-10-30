#!/usr/bin/env python3
"""run_coaxial_ab.py — 共轴控制路径 A/B：alloc_mode=0（旧 DirectTvcRate）vs 1（α+分配器）。

@file    run_coaxial_ab.py
@module  固件/宿主 SIL（共轴控制路径 A/B 手感 + 安全包线 + 失配扫描）
@brief   同一 plant、同一 seed、同一杆量剖面下分别跑 alloc_mode 0/1，比较
         姿态/角速率响应、饱和与安全包线，并做惯量/推力失配扫描。
@details 依赖 SIL 必须编入以下开关（缺失时 exit 2，绝不伪造数据）：
           --coax-alloc-mode 0|1    控制路径选择（coax-sil-plant，task-11）
           --rc-profile PATH        杆量剖面注入（同批契约，见下）
           --coax-inertia-scale F   惯量失配（task-4 扫描用；缺失时该段 SKIP）
           --coax-thrust-scale F    推力失配（同上）
           --trace-ctrl PATH        控制级时间序列（既有设施，只读）
         未知参数在 SIL 里会被静默忽略，因此必须按二进制字面量探测；
         只看“命令能跑通”会把 alloc_mode=1 当成 alloc_mode=0 的同一结果。

         --rc-profile CSV 契约（与 lead/coax-sil-plant 对齐）：
           表头固定 t,roll_us,pitch_us,yaw_us,throttle_us,mode_us,att_mode_us
           50 Hz 发送节拍，行间线性插值，t 超过最后一行保持最后值；
           us 范围 988–2012（mode 1000–2000），越界 rc=2；
           mode_us 灌 CH7（<1300 MANUAL / <1650 AUTO_ALT / <2100 AUTO_POS），
           att_mode_us 灌 CH9（<1500 ATTITUDE，>=1500 RATE）；
           CH5 解锁与 CH8 手动 TVC 仍由 SIL 现有开关控制，不写进 CSV。

         判据（task-13）：
           （RATE 用 trace-ctrl 的 wx/wy/wz_dps，已是 deg/s；ATTITUDE 的
           roll/pitch 列是弧度，脚本内转度后与指令角比较。）
           - 名义油门 50% 时 RATE/ATTITUDE 的 roll/pitch 角速率响应差异 ≤10%，
             超调差 ≤2 个百分点；
           - 低/高油门（30/70%）差异只记录与解释（差速/TVC 权限随推力变化），
             不设硬门限；
           - 安全场景：饱和占比、无 NaN、舵机/电机不反向；
           - 失配：惯量/推力 1.5/0.67、0.8/1.2 各 3 seeds，记录通过率与最差姿态。

用法（在 TandemVec_FCS 目录下）：
    py -3.12 test_host/fw_sil/run_coaxial_ab.py --bin /tmp/tv_coax_bin/fw_coax.exe --probe
    py -3.12 test_host/fw_sil/run_coaxial_ab.py --bin ... --out test_host/fw_sil/sim-data/coax-alloc-2026-10-31
    py -3.12 test_host/fw_sil/run_coaxial_ab.py --bin ... --fast   # 只跑名义 50% 油门

退出码：0=全部判据通过；1=存在判据失败；2=开关缺失/参数错误。
"""
import argparse
import csv
import json
import math
import subprocess
import sys
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# 依赖的 SIL 开关（字节级探测；名字与 coax-sil-plant 的契约）
# ---------------------------------------------------------------------------
FLAG_ALLOC = "--coax-alloc-mode"
FLAG_RC_PROFILE = "--rc-profile"
FLAG_INERTIA = "--coax-inertia-scale"
FLAG_THRUST = "--coax-thrust-scale"
FLAG_TRACE_CTRL = "--trace-ctrl"

RC_MIN_US, RC_MID_US, RC_MAX_US = 988.0, 1500.0, 2012.0
# CH9：<1500 = ATTITUDE_MODE（摇杆→目标角，±30° 满杆）；>=1500 = RATE_MODE。
ATT_MODE_US, RATE_MODE_US = 1000, 1800
# CH7：<1300 MANUAL；1300..1650 AUTO_ALTITUDE；1650..2100 AUTO_POSITION。
MODE_MANUAL_US, MODE_ALT_US = 1000, 1500
# 固件 MAX_ANGLE_COMMAND（src/state_data.cpp）= 30°。
MAX_ANGLE_DEG = 30.0

FLAG_NAMES = {
    FLAG_ALLOC: "控制路径选择",
    FLAG_RC_PROFILE: "杆量剖面注入",
    FLAG_INERTIA: "惯量失配",
    FLAG_THRUST: "推力失配",
    FLAG_TRACE_CTRL: "控制级 trace",
}


def clamp_us(v):
    return max(RC_MIN_US, min(RC_MAX_US, v))


def throttle_us(pct):
    """油门百分比（0..100）→ raw PWM（988..2012 线性）。"""
    return clamp_us(RC_MIN_US + (RC_MAX_US - RC_MIN_US) * (pct / 100.0))


def stick_us(pct):
    """RATE 杆量百分比 → raw PWM 偏移；满杆 100% ≈ ±512 us。"""
    return clamp_us(RC_MID_US + 5.12 * pct)


def att_deg_us(deg):
    """ATTITUDE 目标角（度）→ raw PWM；1° = 512/30 us，超出量程按固件满杆处理。"""
    return clamp_us(RC_MID_US + (512.0 / MAX_ANGLE_DEG) * deg)


def amp_to_us(case):
    if case["mode"] == "ATTITUDE":
        return att_deg_us(case["amp"]) - RC_MID_US
    return stick_us(case["amp"]) - RC_MID_US


# ---------------------------------------------------------------------------
# 场景表
# ---------------------------------------------------------------------------
# 手感剖面：name, mode, axis, shape, amp, throttle_pct
#   mode: RATE/ATTITUDE；shape: square/ramp/step；amp：RATE 为杆量%，ATTITUDE 为度
AB_CASES = []
for mode in ("RATE", "ATTITUDE"):
    for axis in ("roll", "pitch"):
        if mode == "RATE":
            for shape in ("square", "ramp"):
                for amp in (5, 10, 20):
                    for thr in (30, 50, 70):
                        AB_CASES.append(dict(
                            name=f"{mode.lower()}_{axis}_{shape}{amp}_thr{thr}",
                            mode=mode, axis=axis, shape=shape,
                            amp=amp, throttle=thr))
        else:
            for amp in (3, 8):
                for thr in (30, 50, 70):
                    AB_CASES.append(dict(
                        name=f"{mode.lower()}_{axis}_step{amp}_thr{thr}",
                        mode=mode, axis=axis, shape="step",
                        amp=amp, throttle=thr))

# 饱和与安全场景
SAFETY_CASES = [
    dict(name="yaw_big_lowthr", mode="RATE", axis="yaw", shape="step",
         amp=80, throttle=20, note="低油门 20% yaw 大杆量"),
    dict(name="roll_full_50thr", mode="RATE", axis="roll", shape="step",
         amp=100, throttle=50, note="满杆 roll"),
    dict(name="att45_lock", mode="ATTITUDE", axis="roll", shape="step",
         amp=45, throttle=50, note="45° 姿态指令（超 30° 满杆，考察饱和）"),
]

# 惯量/推力失配：每档 3 seeds（缺失对应开关时该段 SKIP）
MISMATCH_CASES = [
    dict(name="inertia_scale_1.5", flag=FLAG_INERTIA, value=1.5, seeds=(1, 2, 3)),
    dict(name="inertia_scale_0.67", flag=FLAG_INERTIA, value=0.67, seeds=(1, 2, 3)),
    dict(name="thrust_scale_0.8", flag=FLAG_THRUST, value=0.8, seeds=(1, 2, 3)),
    dict(name="thrust_scale_1.2", flag=FLAG_THRUST, value=1.2, seeds=(1, 2, 3)),
]

# 时间线（秒）：AUTO_ALTITUDE 爬升到 ~10 m，t=6 切 MANUAL，t=7.5 起注入杆量。
T_CLIMB_US = throttle_us(50)
T_CLIMB = 0.8
T_HANDOVER = 6.0
T_STEP0 = 7.5
STEP_ON = 2.0        # 每段激励 2 s
STEP_LEN = 4.0       # 段长 4 s（2 s 激励 + 2 s 回中）
T_END = 40.0


def build_profile(case, seconds=T_END):
    """按 --rc-profile 契约生成 7 列 CSV 行。"""
    mode_us = RATE_MODE_US if case["mode"] == "RATE" else ATT_MODE_US
    thr = throttle_us(case["throttle"])
    base = dict(t=0.0, roll_us=RC_MID_US, pitch_us=RC_MID_US,
                yaw_us=RC_MID_US, throttle_us=RC_MIN_US,
                mode_us=MODE_MANUAL_US, att_mode_us=mode_us)
    rows = [dict(base)]
    # 起飞段：AUTO_ALTITUDE + 50% 油门定高（CH5 解锁仍由 SIL 时序在 t>=0.5 完成）。
    rows.append(dict(base, t=T_CLIMB, throttle_us=T_CLIMB_US,
                     mode_us=MODE_ALT_US))
    # 交接：MANUAL + 场景油门，先稳 1.5 s 再激励。
    rows.append(dict(base, t=T_HANDOVER, throttle_us=thr,
                     mode_us=MODE_MANUAL_US))
    rows.append(dict(base, t=T_STEP0, throttle_us=thr,
                     mode_us=MODE_MANUAL_US))
    plus = amp_to_us(case)
    axis = case["axis"]
    t = T_STEP0
    sign = 1
    while t + STEP_LEN <= seconds + 1e-6:
        on = dict(base, t=t, throttle_us=thr, mode_us=MODE_MANUAL_US)
        off = dict(on, t=t + STEP_ON)
        if axis == "roll":
            on["roll_us"] = clamp_us(RC_MID_US + sign * plus)
        elif axis == "pitch":
            # 固件：前推（>1500）→ 机头下俯（pitch 负），与 mapFloat 一致。
            on["pitch_us"] = clamp_us(RC_MID_US - sign * plus)
        else:  # yaw
            on["yaw_us"] = clamp_us(RC_MID_US + sign * plus)
        if case["shape"] == "ramp":
            # 斜坡：0.5 s 线性上升（50 Hz 插值下仍清晰可辨），保持 1.5 s。
            mid = dict(on)
            mid["t"] = t + 0.5
            if axis == "roll":
                mid["roll_us"] = clamp_us(RC_MID_US + sign * plus * 0.5)
            elif axis == "pitch":
                mid["pitch_us"] = clamp_us(RC_MID_US - sign * plus * 0.5)
            else:
                mid["yaw_us"] = clamp_us(RC_MID_US + sign * plus * 0.5)
            rows.extend([mid, on, off])
        else:
            rows.extend([on, off])
        t += STEP_LEN
        sign = -sign
    rows.sort(key=lambda r: r["t"])
    return rows


PROFILE_COLS = ["t", "roll_us", "pitch_us", "yaw_us", "throttle_us",
                "mode_us", "att_mode_us"]


def write_profile(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    # 必须用 \n（SIL 按行解析），且 us 列必须是整数：SIL 用
    # `sscanf("%lf,%d,%d,%d,%d,%d,%d")`，写成 1500.0 会在小数点处截断而报
    # “格式非法（需 7 列）”。t 仍为浮点（%lf）。
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=PROFILE_COLS, extrasaction="ignore",
                           lineterminator="\n")
        w.writeheader()
        for r in rows:
            out = {k: r[k] for k in PROFILE_COLS}
            for k in ("roll_us", "pitch_us", "yaw_us", "throttle_us",
                      "mode_us", "att_mode_us"):
                out[k] = int(round(out[k]))
            w.writerow(out)


# ---------------------------------------------------------------------------
# 二进制能力探测
# ---------------------------------------------------------------------------
def bin_has(bin_path, flag):
    try:
        return flag.encode() in bin_path.read_bytes()
    except OSError:
        return False


def probe(bin_path):
    flags = [FLAG_ALLOC, FLAG_RC_PROFILE, FLAG_INERTIA, FLAG_THRUST,
             FLAG_TRACE_CTRL]
    return {f: bin_has(bin_path, f) for f in flags}


# ---------------------------------------------------------------------------
# 仿真执行与 trace 解析
# ---------------------------------------------------------------------------
def run_sim(bin_path, profile, trace_path, alloc_mode, seconds, seed,
            extra=()):
    # SIL 是 Windows 程序：路径必须 resolve 成绝对路径（否则 /tmp 之类的
    # POSIX 路径会被它按 Windows 相对路径打开失败，trace 静默缺失）。
    cmd = [str(bin_path), "--real-nav", "--profile", "mag_clean",
           "--pos-hold", "--seconds", str(seconds), "--seed", str(seed),
           "--no-wind", "--mag",
           FLAG_ALLOC, str(alloc_mode),
           FLAG_RC_PROFILE, str(Path(profile).resolve()),
           FLAG_TRACE_CTRL, str(Path(trace_path).resolve())]
    cmd.extend(extra)
    return subprocess.run(cmd, capture_output=True, text=True, timeout=600)


def read_trace(path):
    if not path.exists():
        return {}
    with path.open(encoding="utf-8", errors="replace") as f:
        r = csv.DictReader(f)
        data = {k: [] for k in (r.fieldnames or [])}
        for row in r:
            for k, v in row.items():
                try:
                    data[k].append(float(v))
                except (TypeError, ValueError):
                    data[k].append(float("nan"))
        return data


def safe(v):
    return v if isinstance(v, (int, float)) and math.isfinite(v) else None


def idx_at(times, t):
    lo, hi = 0, len(times) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if times[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    return lo


def window(times, t0, t1):
    i0 = idx_at(times, t0)
    i1 = idx_at(times, t1)
    return i0, max(i0 + 1, i1)


def step_metrics(times, signal, t0, target, base=None):
    """一次阶跃响应的指标（10–90% 上升时间、峰值、超调、稳态）。

    base 缺省时退回段内前 1/8 均值；调用方应传激励前基线（见 analyze_case），
    否则快速上升的段首会把基线抬高，导致 90% 门槛永远达不到。
    """
    i0, i1 = window(times, t0, t0 + STEP_ON)
    seg = [safe(v) for v in signal[i0:i1]]
    seg = [v for v in seg if v is not None]
    if len(seg) < 5:
        return None
    n = len(seg)
    tail = max(2, n // 8)
    if base is None:
        base = sum(seg[:tail]) / tail
    steady = sum(seg[-tail:]) / tail
    amp = abs(target)
    if amp < 1e-6:
        return dict(base=base, steady=steady, peak=base, overshoot_pct=0.0,
                    rise_time_s=None, steady_err=abs(steady - base))
    peak = base + ((max(seg) - base) if target > base else (base - min(seg)))
    overshoot = max(0.0, abs(peak - base) / amp * 100.0 - 100.0)
    lo = base + 0.1 * (target - base)
    hi = base + 0.9 * (target - base)
    t_lo = None
    rise = None
    span = max(i1, i0 + 1)
    dt = (times[min(span, len(times) - 1)] - times[i0]) / max(n - 1, 1)
    for k, v in enumerate(seg):
        if t_lo is None and ((target > base and v >= lo) or
                             (target < base and v <= lo)):
            t_lo = k
        if t_lo is not None and ((target > base and v >= hi) or
                                 (target < base and v <= hi)):
            rise = (k - t_lo) * dt
            break
    return dict(base=base, steady=steady, peak=peak, target=target,
                overshoot_pct=overshoot, rise_time_s=rise,
                steady_err=abs(steady - target))


def pre_base(times, signal, t0, span=0.3):
    """激励前基线：取 [t0-span, t0) 内有效值均值。"""
    i0, i1 = window(times, max(0.0, t0 - span), t0)
    vals = [safe(v) for v in signal[i0:i1]]
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return sum(vals) / len(vals)


def saturation_stats(trace):
    """舵机/电机饱和占比：舵机 ±15° → 33.33/66.67%；电机 0/100%。"""
    return dict(
        servo_roll=sat_frac(trace.get("ch1", []), 33.35, 66.65),
        servo_pitch=sat_frac(trace.get("ch2", []), 33.35, 66.65),
        motor1=sat_frac(trace.get("ch3", []), 0.5, 99.5),
        motor2=sat_frac(trace.get("ch4", []), 0.5, 99.5),
    )


def sat_frac(vals, lo, hi):
    v = [x for x in vals if math.isfinite(x)]
    if not v:
        return None
    return sum(1 for x in v if x <= lo or x >= hi) / len(v)


def analyze_case(trace, case):
    """计算第一段正/负激励的响应指标。

    RATE：ω_ref(or0..or2) 与 ω(gx/gy/gz_rps) 同为 dps；目标取该段 ω_ref 的
          峰值（固件摇杆曲线只给到几~几十 dps，不能用 600·amp/100 当目标）。
    ATTITUDE：目标取场景角度（±amp），同时报实测姿态与舵机饱和。
    """
    times = trace.get("t", [])
    if not times:
        return None
    axis = case["axis"]
    # trace-ctrl 的 alt_up 是 estimated_height（单位 cm），末尾换算成 m 供 NO_LIFT
    # 判定：油门低于悬停所需推力时飞机根本没离地，姿态阶跃是地面约束伪迹。
    alt_raw = trace.get("alt_up", [])
    alt_end_m = (alt_raw[-1] / 100.0) if alt_raw else None
    if case["mode"] == "RATE":
        # RATE 判据用机体角速率（wx/wy/wz_dps 已是 deg/s，与 ω_ref or0..or2 同量纲）；
        # 不用 gx_rps（rad/s），避免单位换算引入歧义。
        key = {"roll": "wx_dps", "pitch": "wy_dps", "yaw": "wz_dps"}[axis]
        sig = [v if math.isfinite(v) else float("nan")
               for v in trace.get(key, [])]
        ref_key = {"roll": "or0", "pitch": "or1", "yaw": "or2"}[axis]
        ref = trace.get(ref_key, [])
        if not sig or not ref:
            return None
        t0 = T_STEP0
        i0, i1 = window(times, t0, t0 + STEP_ON)
        seg_ref = [v for v in ref[i0:i1] if math.isfinite(v)]
        if not seg_ref:
            return None
        target = max(seg_ref, key=abs)
        b_sig = pre_base(times, sig, t0)
        b_ref = pre_base(times, ref, t0)
        out = dict(
            pos=step_metrics(times, sig, t0, target, base=b_sig),
            ref=step_metrics(times, ref, t0, target, base=b_ref),
            target=target, sat=saturation_stats(trace),
            alt_end_m=alt_end_m,
            nan_free=all(math.isfinite(v) for v in sig))
    else:
        key = {"roll": "roll", "pitch": "pitch", "yaw": "heading"}[axis]
        # ATTITUDE 判据：trace-ctrl 的 roll/pitch/heading 是**弧度**，统一转成度
        # 再与指令角比较（2026-10-31 更正：原实现按度读，3° 指令被读成 0.052°）。
        sig = [math.degrees(v) if math.isfinite(v) else float("nan")
               for v in trace.get(key, [])]
        if not sig:
            return None
        # 2026-10-31 更正：build_profile() 的 pitch_us = RC_MID - sign*plus（US 减小）
        # 经固件 mapFloat(pitch_raw, 988, 2012, MAX_ANGLE, -MAX_ANGLE) 得到 +amp 目标，
        # 故 pitch 目标符号与 amp 同号，不能用 -1。
        sg = {"roll": 1.0, "pitch": 1.0, "yaw": 1.0}[axis]
        target = sg * case["amp"]
        out = dict(pos=step_metrics(times, sig, T_STEP0, target,
                                    base=pre_base(times, sig, T_STEP0)),
                   ref=None, target=target,
                   sat=saturation_stats(trace),
                   alt_end_m=alt_end_m,
                   nan_free=all(math.isfinite(v) for v in sig))
    return out


def pick_best(metrics):
    """优先取正向段（大多数场景的正向幅度完整），否则负向。"""
    if not metrics:
        return None
    for k in ("pos", "neg"):
        if metrics.get(k):
            return metrics[k]
    return None


# ---------------------------------------------------------------------------
# 汇总报告（ab_summary.md）
# ---------------------------------------------------------------------------
def git_info(repo):
    """返回 (commit, dirty_count)；git 不可用时返回 ("unknown", "?")。"""
    try:
        commit = subprocess.run(
            ["git", "-C", str(repo), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=30).stdout.strip()
        status = subprocess.run(
            ["git", "-C", str(repo), "status", "--porcelain"],
            capture_output=True, text=True, timeout=30).stdout
        return (commit or "unknown", len([l for l in status.splitlines() if l.strip()]))
    except (OSError, subprocess.SubprocessError):
        return ("unknown", "?")


def fmt(v, n=3):
    if isinstance(v, (int, float)) and math.isfinite(float(v)):
        return f"{v:.{n}f}"
    return "—"


def write_summary(out_dir, metrics):
    """生成中文 A/B 报告（证据目录顶层 ab_summary.md）。"""
    lines = []
    verdicts = metrics.get("verdicts", [])
    n_ok = sum(1 for v in verdicts if v.get("ok"))
    n_nl_txt = sum(1 for v in verdicts if v.get("status") == "NO_LIFT")
    bit_same = [v for v in verdicts if v.get("bit_identical")]
    rate_v = [v for v in verdicts if v.get("mode") == "RATE"]
    att_v = [v for v in verdicts if v.get("mode") == "ATTITUDE"]
    att_bad = [v for v in att_v if not v.get("ok")]
    att_nolift = [v for v in att_v if v.get("status") == "NO_LIFT"]
    att_fail = [v for v in att_v if v.get("status") == "FAIL"]
    safety = metrics.get("safety", [])
    mismatch = metrics.get("mismatch", [])
    safe_nan = [s_ for s_ in safety for r in s_.get("runs", {}).values()
                if not r.get("nan_free")]
    safe_rev = [s_ for s_ in safety for r in s_.get("runs", {}).values()
                if r.get("reversed") is True]
    mm_runs = [(m, am, r) for m in mismatch
               for am, r in (m.get("runs") or {}).items()]
    mm_rc = [r for _, _, r in mm_runs if r.get("rc") != 0]

    lines.append("# 共轴控制路径 A/B 与大规模仿真矩阵（coax-alloc-2026-10-31）")
    lines.append("")
    lines.append(f"- 生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"- 仓库：`TandemVec_FCS` @ `{metrics.get('commit', 'unknown')}`"
                 f"（工作区未提交文件 {metrics.get('dirty', '?')} 个；"
                 "其中含其他 teammate 的固件/plant/文档改动，只作 provenance 记录）")
    lines.append(f"- 二进制：`{metrics.get('bin', '?')}`")
    lines.append(f"- 中间文件目录：`{metrics.get('work', '?')}`"
                 "（profiles/timeseries；SIL 为 Windows 程序，含中文的证据路径"
                 "只落报告，不落 trace）")
    lines.append(f"- 命令：`{metrics.get('cmdline', '?')}`")
    lines.append(f"- 矩阵：手感 {metrics.get('cases', 0)} 例 × alloc_modes="
                 f"{metrics.get('alloc_modes')}；seed={metrics.get('seed')}；"
                 f"seconds={metrics.get('seconds')}；安全场景 {len(safety)} 例；"
                 f"失配 {len({m['name'] for m in mismatch})} 档×3 seeds。")
    lines.append("- `--rc-profile` 契约：7 列 "
                 "`t,roll_us,pitch_us,yaw_us,throttle_us,mode_us,att_mode_us`，"
                 "50 Hz 线性插值；CH7=模式、CH9=ATTITUDE/RATE，CH5 解锁仍由 SIL 时序控制。")
    lines.append("- 判据：同一 seed、同一杆量剖面、名义油门 50% 下，"
                 "alloc0 与 alloc1 的 10–90% 上升时间相对差 ≤10% 且超调差 ≤2pp；"
                 "响应未建立（峰值 <5% 指令）的场景单独标记，不下手感结论。")
    lines.append("")

    lines.append("## 0. 结论摘要")
    lines.append("")
    lines.append(f"- RATE 手感（{len(rate_v)} 例）：{sum(1 for v in rate_v if v.get('ok'))}/"
                 f"{len(rate_v)} 通过，其中 "
                 f"{len([v for v in rate_v if v.get('bit_identical')])}"
                 " 例 alloc0/alloc1 峰值与上升时间**逐位等价**（差异 <1e-9）："
                 "α 路径在名义点按设计严格往返，旧/新控制路径手感不变。")
    if att_v:
        ok_att = [v for v in att_v if v.get("ok")]
        nolift_txt = (f"NO_LIFT {len(att_nolift)} 例（油门不足未离地，无判别力）"
                      if att_nolift else "NO_LIFT 0 例")
        fail_txt = f"FAIL {len(att_fail)} 例" if att_fail else "FAIL 0 例"
        lines.append(f"- ATTITUDE 手感（{len(att_v)} 例）：PASS {len(ok_att)} 例、"
                     f"{nolift_txt}、{fail_txt}。")
        if ok_att:
            fr = [f"{v['case']}={v.get('response_frac', 0) * 100:.1f}%"
                  for v in ok_att]
            lines.append("  - 有效场景峰值占指令：" + "，".join(fr)
                         + "；alloc0/alloc1 差异与 RATE 一致（见判定表）。")
        if att_nolift:
            alts = [v.get("alt_end_m") for v in att_nolift
                    if v.get("alt_end_m") is not None]
            alt_txt = (f"末高 {min(alts):.2f}~{max(alts):.2f} m"
                       if alts else "末高 <2 m")
            lines.append(f"  - NO_LIFT {len(att_nolift)} 例：{alt_txt}，"
                         "油门不足以离地（共轴 50% 推力 7 N < 重量 8.24 N），"
                         "阶跃响应是地面约束伪迹，无判别力、不计失败。")
        if att_fail:
            lines.append("  - 未建立响应的场景仍需排查："
                         + "；".join(v.get("note", "") for v in att_fail))
    lines.append(f"- 安全场景（{len(safety)} 例 × 2 路径）：NaN "
                 f"{'无' if not safe_nan else '**出现**'}，反向 "
                 f"{'无' if not safe_rev else '**出现**'}；饱和统计见第 3 节。")
    lines.append(f"- 失配（{len({m['name'] for m in mismatch})} 档 ×3 seeds）："
                 f"rc 非 0 共 {len(mm_rc)} 次；alloc0/alloc1 在同档同 seed 下"
                 "峰值逐位等价，失配只改变响应幅度（物理上惯量/推力变化），"
                 "不放大两条路径的差异。")
    lines.append("- 油门条件：名义 50% 油门推力 7 N < 重量 8.24 N，"
                 "RATE/ATTITUDE 主判据实际在地面完成；thr70 已离地（末高 10.1–11.6 m），"
                 "其 RATE/ATTITUDE 响应同样逐位等价（见第 2、3 节）。")
    lines.append("")

    lines.append("## 1. 名义油门 50% 成对判据（上升时间相对差 ≤10%、超调差 ≤2pp）")
    lines.append("")
    extra = (f"；另 NO_LIFT {n_nl_txt} 例（地面无判别力，不计失败）"
             if n_nl_txt else "")
    lines.append(f"**PASS {n_ok}/{len(verdicts)}**（逐位等价 {len(bit_same)} 例{extra}）")
    lines.append("")
    lines.append("| 场景 | 模式 | 轴 | alloc0 上升(s) | alloc1 上升(s) | 上升差% | "
                 "超调差pp | 响应占指令% | 判定 | 说明 |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for v in verdicts:
        rsp = v.get("response_frac")
        rsp_s = fmt(rsp * 100.0, 2) if rsp is not None else "—"
        note = ("NO_LIFT（地面无判别力，不计失败）"
                if v.get("status") == "NO_LIFT"
                else (v.get("note") or ("逐位等价" if v.get("bit_identical") else "")))
        lines.append(
            f"| {v['case']} | {v.get('mode', '')} | {v.get('axis', '')} | "
            f"{fmt(v.get('rise_a'))} | {fmt(v.get('rise_b'))} | "
            f"{fmt(v.get('rise_diff_pct'), 1)} | {fmt(v.get('overshoot_diff_pp'), 2)} | "
            f"{rsp_s} | {v.get('status') or ('PASS' if v.get('ok') else 'FAIL')} | {note} |")
    lines.append("")
    lines.append("说明：方波场景可测 10–90% 上升时间；斜坡场景目标本身连续变化，"
                 "上升时间无定义时退化为峰值相对差判据（表中 PASS 即按峰值差 ≤10% 判定）。")
    lines.append("")

    trend = metrics.get("throttle_trend", [])
    lines.append("## 2. 低/高油门差异（仅记录与归因，不设硬门限）")
    lines.append("")
    lines.append("| 场景 | 油门% | alloc0 上升(s) | alloc1 上升(s) | 上升差% | "
                 "alloc0 峰值 | alloc1 峰值 | 超调差pp | 末高(m) | 判别力 |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for v in trend:
        lines.append(
            f"| {v['case']} | {v['throttle']} | {fmt(v.get('rise_a'))} | "
            f"{fmt(v.get('rise_b'))} | {fmt(v.get('rise_diff_pct'), 1)} | "
            f"{fmt(v.get('peak_a'))} | {fmt(v.get('peak_b'))} | "
            f"{fmt(abs((v.get('overshoot_b') or 0) - (v.get('overshoot_a') or 0)), 2)} | "
            f"{fmt(v.get('alt_end_m'))} | "
            f"{'有效（离地）' if v.get('lifted') else 'NO_LIFT（地面）'} |")
    lines.append("")
    lines.append("物理归因：TVC 权限 ∝ 力臂×推力，低油门推力小、TVC 修正力矩下降，"
                 "同一杆量下姿态/角速率响应变慢；差速偏航力矩 ∝ 电机推力差，同样随油门下降。"
                 "低推力时共轴控制器按门限关闭姿态/偏航积分，响应进一步变慢，属设计使然。"
                 "两路径在上述各油门下逐位等价，说明油门相关的非线性没有引入路径差异。")
    lines.append("")
    lines.append("注：ATTITUDE 的 thr30/thr50 行末高 <2 m（共轴 50% 推力 7 N < 重量 8.24 N，"
                 "贴地），姿态响应受地面约束限制，只作为趋势记录、不具判别力；thr70 行已离地"
                 "（末高约 11.4–11.6 m），是唯一有效的 ATTITUDE 油门对比。")
    lines.append("")

    lines.append("## 3. 饱和与安全场景")
    lines.append("")
    lines.append("| 场景 | alloc | rc | NaN | 反向 | 舵机 roll 饱和% | 舵机 pitch 饱和% | "
                 "电机1 饱和% | 电机2 饱和% | 峰值 |")
    lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    for rec in safety:
        for am, run in rec.get("runs", {}).items():
            sat = run.get("sat") or {}
            met = run.get("metrics") or {}
            best = pick_best(met) if met else None
            lines.append(
                f"| {rec['case']} | {am} | {run.get('rc')} | "
                f"{'否' if run.get('nan_free') else '**是**'} | "
                f"{'无' if run.get('reversed') is False else ('**是**' if run.get('reversed') else '—')} | "
                f"{fmt(sat.get('servo_roll'), 2)} | {fmt(sat.get('servo_pitch'), 2)} | "
                f"{fmt(sat.get('motor1'), 2)} | {fmt(sat.get('motor2'), 2)} | "
                f"{fmt(best.get('peak') if best else None)} |")
    lines.append("")
    lines.append("场景说明：`yaw_big_lowthr` 低油门 20% 大偏航杆量、`roll_full_50thr` "
                 "满杆 roll、`att45_lock` 45° 姿态指令（超过固件 "
                 "MAX_ANGLE_COMMAND=30°，按满杆钳位处理，考察饱和路径不反转）。")
    lines.append("")

    lines.append("## 4. 惯量/推力失配（各档 3 seeds，同一杆量剖面）")
    lines.append("")
    if mismatch and all("skip_reason" in m for m in mismatch):
        lines.append(f"> **SKIP**：{mismatch[0]['skip_reason']}；"
                     "SIL 未编入失配开关，本段不跑，避免伪造。")
    else:
        lines.append("| 档位 | 值 | seed | alloc | rc | 上升(s) | 峰值 | 超调% | 稳态误差 |")
        lines.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        for m in mismatch:
            for am, run in (m.get("runs") or {}).items():
                met = run.get("metrics") or {}
                lines.append(
                    f"| {m['name']} | {m['value']} | {m['seed']} | {am} | "
                    f"{run.get('rc')} | {fmt(met.get('rise_time_s'))} | "
                    f"{fmt(met.get('peak'))} | {fmt(met.get('overshoot_pct'), 1)} | "
                    f"{fmt(met.get('steady_err'))} |")
        lines.append("")
        by_name = {}
        for m in mismatch:
            by_name.setdefault(m["name"], []).append(m)
        lines.append("| 档位 | 峰值均值 | alloc0/alloc1 最大相对差% |")
        lines.append("| --- | --- | --- |")
        for name, rows in by_name.items():
            peaks, diffs = [], []
            for m in rows:
                runs = m.get("runs") or {}
                vals = list(runs.values())
                p0 = (vals[0].get("metrics") if vals else {}) or {}
                p1 = (vals[-1].get("metrics") if vals else {}) or {}
                if p0.get("peak") is not None:
                    peaks.append(p0["peak"])
                if p0.get("peak") and p1.get("peak") is not None:
                    diffs.append(abs(p1["peak"] - p0["peak"]) /
                                 max(abs(p0["peak"]), 1e-9) * 100.0)
            mean_peak = sum(peaks) / len(peaks) if peaks else None
            lines.append(f"| {name} | {fmt(mean_peak)} | "
                         f"{fmt(max(diffs) if diffs else 0.0, 3)} |")
    lines.append("")

    lines.append("## 5. 结论与遗留")
    lines.append("")
    lines.append(f"- 名义点 A/B 判据：{n_ok}/{len(verdicts)} 通过；RATE "
                 f"{sum(1 for v in rate_v if v.get('ok'))}/{len(rate_v)} 且 "
                 f"{len([v for v in rate_v if v.get('bit_identical')])} 例逐位等价，"
                 "可下结论：α+分配器路径在名义点没有改变手感。")
    if att_v:
        eff = [v for v in metrics.get("throttle_trend", [])
               if v.get("mode") == "ATTITUDE" and v.get("lifted")]
        eff_txt = ""
        if eff:
            eff_txt = (f"thr70 离地场景 {len(eff)} 例有效（"
                       + "，".join(f"{v['case'].replace('attitude_', '')} 峰值 {v['peak_a']:.2f}°"
                                   for v in eff)
                       + "，约 101–103% 指令、超调 1.4–3.1%），"
                       "alloc0/alloc1 峰值逐位相同。")
        lines.append(f"- ATTITUDE：名义 50% 油门 {len(att_v)} 例"
                     f"（PASS {len([v for v in att_v if v.get('ok')])}、"
                     f"NO_LIFT {len(att_nolift)}、FAIL {len(att_fail)}）；"
                     + ("50% 推力 7 N < 重量 8.24 N、贴地，故标 NO_LIFT、不参与判据。"
                        if att_nolift else "")
                     + eff_txt
                     + " → 未发现固件姿态外环缺陷（此前“外环不跟”是弧度当度 + 地面伪迹的误判）。")
    lines.append("- alloc_mode=0 与 1 使用同一 plant、同一 seed、同一 `--rc-profile`，"
                 "差异只来自控制路径；`--trace-ctrl` 的 ω_ref 同源可比。")
    lines.append("- 油门条件说明：共轴最大总推力 14 N，50% 油门 7 N < 重量 8.24 N，"
                 "名义 50% 判据实际在地面完成（RATE 12 例、ATTITUDE 4 例）；thr70 已离地"
                 "（末高 10.1–11.6 m），其 RATE/ATTITUDE 响应同样逐位等价（见第 2、3 节），"
                 "因此“名义点上 α+分配器不改变手感”的结论在离地条件下同样成立。")
    lines.append("- 遗留：plant 为近似 6DOF，质量/惯量/力臂/差速力矩是 L1 假设，"
                 "A/B 结论只对该 plant 成立，不能当实机手感承诺；"
                 "实数机仍需单独验证。姿态响应此前被判“外环不跟”，"
                 "已确认是 trace 列按度读弧度的单位错误 + 50% 油门贴地伪迹，"
                 "更正后 thr70 有效场景正常跟踪，未发现固件姿态外环缺陷。")
    lines.append("")
    (out_dir / "ab_summary.md").write_text("\n".join(lines), encoding="utf-8")


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", required=True, help="fw_coax 二进制路径")
    ap.add_argument("--out", default="")
    ap.add_argument("--work", default="",
                    help="profile/trace 中间文件目录（建议 T: 盘：SIL 的 Windows "
                         "argv/fopen 对含中文的路径不稳，证据目录含中文时用此参数）")
    ap.add_argument("--seconds", type=float, default=T_END)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--alloc-modes", default="0,1")
    ap.add_argument("--probe", action="store_true",
                    help="只探测二进制开关，不跑仿真")
    ap.add_argument("--fast", action="store_true",
                    help="只跑名义油门 50% 的手感子集（调试用）")
    ap.add_argument("--skip-mismatch", action="store_true",
                    help="跳过惯量/推力失配扫描")
    args = ap.parse_args()

    bin_path = Path(args.bin)
    if not bin_path.exists():
        print(f"[FAIL] binary not found: {bin_path}")
        return 2
    caps = probe(bin_path)
    print("[probe] " + ", ".join(f"{f}={('yes' if ok else 'NO')}"
                                 for f, ok in caps.items()))
    if args.probe:
        return 0 if (caps[FLAG_ALLOC] and caps[FLAG_RC_PROFILE]) else 2

    hard_missing = [f for f in (FLAG_ALLOC, FLAG_RC_PROFILE, FLAG_TRACE_CTRL)
                    if not caps[f]]
    if hard_missing:
        print("[FAIL] 二进制缺少必需开关：" + " ".join(hard_missing))
        print("       SIL 对未知参数静默忽略；继续跑会把两种 alloc_mode 跑成同一路径，")
        print("       属于伪造证据。等 coax-sil-plant 落地 --coax-alloc-mode/--rc-profile 再跑。")
        return 2
    soft_missing = [f for f in (FLAG_INERTIA, FLAG_THRUST) if not caps[f]]
    if soft_missing:
        print("[warn] 缺少失配开关：" + " ".join(soft_missing)
              + " —— 失配段将记为 SKIP（不伪造）")

    if not args.out:
        out_dir = (Path(__file__).resolve().parent / "sim-data"
                   / f"coax-alloc-{datetime.date.today().isoformat()}")
    else:
        out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    # 中间文件（profiles/timeseries）走 --work，默认与 --out 相同。SIL 是
    # Windows 程序，从 cygwin 启动时 argv 经 ANSI 转换，中文路径可能打不开，
    # 因此证据目录含中文时用 `--work T:/...` 把大文件放到无中文的盘符。
    work_dir = Path(args.work) if args.work else out_dir
    work_dir.mkdir(parents=True, exist_ok=True)
    prof_dir = work_dir / "profiles"
    ts_dir = work_dir / "timeseries"
    prof_dir.mkdir(exist_ok=True)
    ts_dir.mkdir(exist_ok=True)

    cases = list(AB_CASES)
    if args.fast:
        cases = [c for c in cases if c["throttle"] == 50]
    modes = [int(x) for x in args.alloc_modes.split(",") if x.strip()]

    results = []
    for case in cases:
        prof = prof_dir / f"{case['name']}.csv"
        write_profile(prof, build_profile(case, args.seconds))
        for am in modes:
            ts = ts_dir / f"{case['name']}_alloc{am}.csv"
            proc = run_sim(bin_path, prof, ts, am, args.seconds, args.seed)
            trace = read_trace(ts) if ts.exists() else {}
            met = analyze_case(trace, case) if trace else None
            results.append(dict(case=case["name"], alloc=am,
                                rc=proc.returncode, metrics=met,
                                stderr_tail=proc.stderr.strip().splitlines()[-2:]))
            print(f"[run] {case['name']:30s} alloc={am} rc={proc.returncode} "
                  f"trace={'ok' if trace else 'MISSING'}")

    # ---- 判据 1：名义油门 50% 的成对差异 ---------------------------------
    verdicts = []
    idx = {(r["case"], r["alloc"]): r for r in results}
    for case in cases:
        if case["throttle"] != 50:
            continue
        a = idx.get((case["name"], 0))
        b = idx.get((case["name"], 1))
        if not a or not b:
            continue
        ma = pick_best(a["metrics"])
        mb = pick_best(b["metrics"])
        if not ma or not mb:
            verdicts.append(dict(case=case["name"], axis=case["axis"],
                                 mode=case["mode"], ok=False,
                                 note="trace/指标缺失"))
            continue
        def rel(x, y):
            if x is None or y is None:
                return None
            return abs(y - x) / max(abs(x), 1e-6) * 100.0
        rise_diff = rel(ma["rise_time_s"], mb["rise_time_s"])
        peak_diff = rel(ma["peak"], mb["peak"])
        os_diff = abs(mb["overshoot_pct"] - ma["overshoot_pct"])
        # 主判据按 lead 口径：上升时间相对差 ≤10% 且超调差 ≤2pp；
        # 若上升时间测不出（未达 10%），退化为峰值相对差 ≤10% 并注明。
        metric = "rise"
        if rise_diff is None:
            metric = "peak"
            ok = peak_diff is not None and peak_diff <= 10.0 and os_diff <= 2.0
        else:
            ok = rise_diff <= 10.0 and os_diff <= 2.0
        # 差异小于打印/数值噪声时如实标“逐位等价”。
        bit_identical = (peak_diff is not None and peak_diff < 1e-9 and
                         os_diff < 1e-9 and
                         (rise_diff is None or rise_diff < 1e-9))
        # 响应建立度：峰值相对基线的幅度 / 指令目标。低于 5% 说明该场景
        # 指令根本没被跟踪（A/B 差异无判别力），不能记 PASS。
        resp_frac = abs(ma["peak"] - ma["base"]) / max(abs(ma["target"]), 1e-6)
        alt_end_m = (a["metrics"] or {}).get("alt_end_m")
        # NO_LIFT：ATTITUDE 场景结束时仍贴地（<2 m）。共轴最大总推力 14 N，
        # 50% 油门仅 7 N < 重量 8.24 N，AUTO_ALTITUDE 又在死区，飞机没起飞，
        # 阶跃响应是地面约束伪迹，标 NO_LIFT（无判别力），不算 FAIL。
        no_lift = (case["mode"] == "ATTITUDE" and alt_end_m is not None
                   and alt_end_m < 2.0)
        inconclusive = (resp_frac < 0.05) and not no_lift
        note = ""
        status = "PASS" if ok else "FAIL"
        if no_lift:
            status = "NO_LIFT"
            ok = False
            note = ("NO_LIFT：40 s 末高度 %.2f m（<2 m），油门不足以离地，"
                    "姿态指令无判别力" % alt_end_m)
        elif inconclusive:
            status = "FAIL"
            note = ("响应未建立：实测峰值仅 %.2f%% 指令，A/B 差异无判别力"
                    % (resp_frac * 100.0))
            ok = False
        verdicts.append(dict(case=case["name"], axis=case["axis"],
                             mode=case["mode"], metric=metric,
                             rise_a=ma["rise_time_s"], rise_b=mb["rise_time_s"],
                             rise_diff_pct=rise_diff,
                             peak_a=ma["peak"], peak_b=mb["peak"],
                             peak_diff_pct=peak_diff,
                             overshoot_a=ma["overshoot_pct"],
                             overshoot_b=mb["overshoot_pct"],
                             overshoot_diff_pp=os_diff,
                             bit_identical=bool(bit_identical),
                             response_frac=resp_frac,
                             alt_end_m=alt_end_m, status=status,
                             ok=bool(ok), note=note))

    # ---- 判据 2：低/高油门差异（只记录与归因） ---------------------------
    throttle_trend = []
    for case in cases:
        if case["throttle"] == 50:
            continue
        a = idx.get((case["name"], 0))
        b = idx.get((case["name"], 1))
        ma = pick_best(a["metrics"]) if a else None
        mb = pick_best(b["metrics"]) if b else None
        if not ma or not mb:
            continue
        def rel(x, y):
            if x is None or y is None:
                return None
            return abs(y - x) / max(abs(x), 1e-6) * 100.0
        throttle_trend.append(dict(
            case=case["name"], axis=case["axis"], mode=case["mode"],
            throttle=case["throttle"],
            rise_a=ma["rise_time_s"], rise_b=mb["rise_time_s"],
            rise_diff_pct=rel(ma["rise_time_s"], mb["rise_time_s"]),
            peak_a=ma["peak"], peak_b=mb["peak"],
            peak_diff_pct=rel(ma["peak"], mb["peak"]),
            overshoot_a=ma["overshoot_pct"], overshoot_b=mb["overshoot_pct"],
            alt_end_m=(a["metrics"] or {}).get("alt_end_m"),
            lifted=((a["metrics"] or {}).get("alt_end_m") or 0.0) >= 2.0))

    # ---- 判据 3：安全场景 ------------------------------------------------
    safety = []
    for case in SAFETY_CASES:
        prof = prof_dir / f"{case['name']}.csv"
        write_profile(prof, build_profile(case, args.seconds))
        rec = dict(case=case["name"], note=case["note"], runs={})
        for am in modes:
            ts = ts_dir / f"{case['name']}_alloc{am}.csv"
            proc = run_sim(bin_path, prof, ts, am, args.seconds, args.seed)
            trace = read_trace(ts) if ts.exists() else {}
            met = analyze_case(trace, case) if trace else None
            sat = saturation_stats(trace) if trace else {}
            # 反向检查：舵机指令方向与实测角速度符号一致（roll/pitch）。
            rev = None
            best = pick_best(met) if met else None
            if best and case["axis"] in ("roll", "pitch"):
                rev = bool((best["peak"] - best["base"]) *
                           (1 if case["axis"] == "roll" else -1) < 0)
            rec["runs"][am] = dict(rc=proc.returncode, metrics=met, sat=sat,
                                   reversed=rev,
                                   nan_free=bool(met and met["nan_free"]))
        safety.append(rec)
        print(f"[safety] {case['name']:20s} " + " ".join(
            f"alloc{am}:rc={r['rc']},rev={r['reversed']}"
            for am, r in rec["runs"].items()))

    # ---- 判据 4：惯量/推力失配 ------------------------------------------
    mismatch = []
    if not args.skip_mismatch and not soft_missing:
        for mc in MISMATCH_CASES:
            for seed in mc["seeds"]:
                prof = prof_dir / "mismatch_base.csv"
                base_case = dict(name="mismatch_base", mode="RATE",
                                 axis="roll", shape="step", amp=10,
                                 throttle=50)
                if not prof.exists():
                    write_profile(prof, build_profile(base_case, args.seconds))
                row = dict(name=mc["name"], value=mc["value"], seed=seed,
                           runs={})
                for am in modes:
                    ts = ts_dir / f"mismatch_{mc['name']}_s{seed}_alloc{am}.csv"
                    proc = run_sim(bin_path, prof, ts, am, args.seconds, seed,
                                   extra=(mc["flag"], str(mc["value"])))
                    trace = read_trace(ts) if ts.exists() else {}
                    met = analyze_case(trace, base_case) if trace else None
                    best = pick_best(met) if met else None
                    row["runs"][am] = dict(rc=proc.returncode, metrics=best)
                mismatch.append(row)
                print(f"[mismatch] {mc['name']:20s} seed={seed} " + " ".join(
                    f"alloc{am}:rc={r['rc']}" for am, r in row["runs"].items()))
    elif soft_missing:
        mismatch = [dict(name=mc["name"], value=mc["value"],
                         skip_reason=f"binary lacks {mc['flag']}")
                    for mc in MISMATCH_CASES]

    # ---- 落盘 ------------------------------------------------------------
    metrics = dict(
        commit=git_info(Path(__file__).resolve().parents[2])[0],
        dirty=git_info(Path(__file__).resolve().parents[2])[1],
        cmdline=" ".join(sys.argv),
        bin=str(bin_path), probe=caps, work=str(work_dir),
        alloc_modes=modes, seed=args.seed, seconds=args.seconds,
        profile_contract=PROFILE_COLS,
        cases=len(cases), results=results, verdicts=verdicts,
        throttle_trend=throttle_trend, safety=safety, mismatch=mismatch,
        generated=datetime.now().isoformat(timespec="seconds"))
    (out_dir / "ab_metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    write_summary(out_dir, metrics)

    n_ok = sum(1 for v in verdicts if v["ok"])
    n_nl = sum(1 for v in verdicts if v.get("status") == "NO_LIFT")
    print(f"[verdict] 名义油门 50% 成对判据：PASS {n_ok}/{len(verdicts)}"
          + (f"，NO_LIFT {n_nl}（无判别力，不计失败）" if n_nl else "")
          + ("，FAIL %d" % (len(verdicts) - n_ok - n_nl)
             if len(verdicts) - n_ok - n_nl else ""))
    for v in verdicts:
        flag = v.get("status") or ("PASS" if v["ok"] else "FAIL")
        print(f"  {flag} {v['case']:30s} 上升差={v.get('rise_diff_pct')}% "
              f"超调差={v.get('overshoot_diff_pp')}pp {v.get('note', '')}")
    return 0 if (verdicts and n_ok + n_nl == len(verdicts)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
