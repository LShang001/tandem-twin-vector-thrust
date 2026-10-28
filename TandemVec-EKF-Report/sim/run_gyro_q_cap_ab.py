# -*- coding: utf-8 -*-
"""陀螺 Q 保守夹持的同一二进制 runtime A/B（导弹 + VTOL 生产口径）。

@file    run_gyro_q_cap_ab.py
@brief   只改 `--gyro-q-cap`，比较 gyro 量测噪声上界对大过载/失联精度的影响
@module  TandemVec-EKF-Report/分析脚本（L2 研究设施）
@details 背景：振动估计器的高通只有 0.5 Hz，且吃在 IMU 软件低通之后；实测洁净弹道
         `accel_rms_g`=0.138–0.279 g（随过载档位上升、零注入振动），`vib_high`
         占空比 63–72%，`gyro_std_suggest` 长期停在标称 0.0015 的约 2.3 倍处。
         加计侧有 [0.15,0.25] 保守夹持（弹道上 100% 卡在 0.25，等于不再生效），
         陀螺侧过去每拍无条件写入 suggest——本实验检验收紧它有无可测收益。

预注册判据（跑前写死，跑后不改口径）：
  P1 主判据：6.4 g 洁净 + 8.4 g 的 `nav_att_rmse` 中位相对 cap=0.010 改善 ≥10%，
     且 `pos_rms` 不劣化超过 5%；
  P2 失联：失联 5 s / 20 s 的 `pos_rms` 中位劣化 ≤5%；
  P3 行为保持：cap=0.010 档与加夹持前的旧二进制逐 seed 完全相同；
  P4 VTOL：mag_clean / mag_nominal ×8 seeds 的 `hold_end_rms` 与 `max_roll`
     中位不劣化超过 5%。
  四条里 P1 是唯一"收益"判据；P1 不成立即按"无可测收益完整撤回"处理，
  不得改用 pos_max/单 seed 等次要量补判据。
"""
import concurrent.futures as cf
import hashlib
import json
import os
import re
import statistics as st
import subprocess
import sys

import psutil

JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")
EXE = r"C:/Users/12631/AppData/Local/Temp/tv_vib/fw_real.exe"
MBASE = ["--real-nav", "--missile", "--seconds", "45", "--gnss-rate-hz", "10",
         "--profile", "mag_clean", "--imu-grade", "icm"]
VBASE = ["--real-nav", "--pos-hold", "--flow", "--seconds", "40"]
CAPS = [("0.010(历史)", "0.010"), ("0.0035", "0.0035"), ("0.0015(关)", "0.0015")]
NSEED = 8
# (名称, kind, 追加参数)
CONDS = [
    ("6.4g洁净", "m", []),
    ("8.4g", "m", ["--ms-thrust", "8000"]),
    ("失联5s", "m", ["--gnss-outage", "20:25"]),
    ("失联20s", "m", ["--gnss-outage", "20:40"]),
    ("振1g80Hz", "m", ["--vib", "1.0", "--vib-hz", "80", "--vib-start", "1.5"]),
    ("VTOL clean", "v", ["--profile", "mag_clean"]),
    ("VTOL nominal", "v", ["--profile", "mag_nominal"]),
]
KEY = ["pos_rms", "pos_max", "nav_att_rmse", "nav_roll_rmse", "nav_pitch_rmse",
       "nav_yaw_rmse", "nav_vel_rms", "max_roll", "hold_end_rms", "ad_app"]


def run(kind, extra, cap, seed):
    base = MBASE if kind == "m" else VBASE
    cmd = [EXE] + base + extra + ["--gyro-q-cap", cap, "--seed", str(seed)]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         text=True, env=os.environ.copy())
    try:
        psutil.Process(p.pid).cpu_affinity([2])
    except Exception:
        pass
    try:
        out, err = p.communicate(timeout=600)
    except subprocess.TimeoutExpired:
        p.kill()
        return None
    m = JRE.findall((out or "") + (err or ""))
    return json.loads(m[-1]) if m else None


def main():
    print("exe md5", hashlib.md5(open(EXE, "rb").read()).hexdigest()[:8])
    print(f"{'工况':<13}{'指标':<14}" + "".join(f"{c[0]:>16}" for c in CAPS))
    print("-" * 76)
    rows = {}
    for cname, kind, extra in CONDS:
        per = {}
        for label, cap in CAPS:
            with cf.ThreadPoolExecutor(max_workers=8) as pool:
                rs = [r for r in pool.map(
                    lambda s: run(kind, extra, cap, s),
                    range(1, NSEED + 1)) if r]
            per[label] = rs
        rows[cname] = per
        show = (["pos_rms", "nav_att_rmse", "nav_pitch_rmse", "nav_yaw_rmse"]
                if kind == "m" else
                ["pos_rms", "nav_att_rmse", "hold_end_rms", "max_roll"])
        for key in show:
            line = f"{cname:<13}{key:<14}"
            for label, _ in CAPS:
                vals = [r.get(key) or 0.0 for r in per[label]]
                line += f"{st.median(vals):>16.4f}"
            print(line)
        # 相对历史档的中位变化
        ref = per[CAPS[0][0]]
        for label, _ in CAPS[1:]:
            cur = per[label]
            n = min(len(ref), len(cur))
            d_att = st.median([cur[i]["nav_att_rmse"] - ref[i]["nav_att_rmse"]
                               for i in range(n)])
            d_pos = st.median([cur[i]["pos_rms"] - ref[i]["pos_rms"]
                               for i in range(n)])
            w = sum(1 for i in range(n)
                    if cur[i]["nav_att_rmse"] < ref[i]["nav_att_rmse"])
            print(f"{'':<13}  {label}: Δatt={d_att:+.4f}° Δpos={d_pos:+.3f} m"
                  f"  姿态变好 {w}/{n}")
    path = ("D:/纵列双发矢量推力飞行器/TandemVec-EKF-Report/sim-data/"
            "gyro_q_cap_ab_2026-10-27.json")
    with open(path, "w", encoding="utf-8") as fh:
        json.dump({"exe_md5": hashlib.md5(open(EXE, "rb").read()).hexdigest(),
                   "nseed": NSEED, "rows": rows}, fh, ensure_ascii=False)
    print("json:", path)


if __name__ == "__main__":
    sys.exit(main())
