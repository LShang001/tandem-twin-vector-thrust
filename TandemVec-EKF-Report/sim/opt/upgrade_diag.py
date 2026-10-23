# -*- coding: utf-8 -*-
"""upgrade_diag.py — 候选 A（磁航向）诊断：门控时序/观测一致性/变差归因。

@file    upgrade_diag.py
@module  analysis/EKF 优化研究（诊断）
@brief   供 notes/05 的"负面/边界结果"章节使用的磁航向消融与逐相位统计。
@details 回答三问：Q1 门控时序（gate_tilt 是否把过渡段磁全关、SC3 转弯期融合情况）→
         按场景相位统计融合/拒绝（按原因）数量与自适应 σ；
         Q2 观测一致性（磁方位误差符号 + H 索引，baseline H[0,7:10] 错位）→ 单点注入实验；
         Q3 SC5/SC7 变差归因 → 消融：无磁 / 候选形态 / σ×3 / 关倾斜门 / 鲁棒软更新 / 受扰磁。
         输出：TandemVec-EKF-Report/sim-data/opt/upgrade_mag_diag.json。
@theory  磁航向四元数 H 与倾斜补偿 @ref docs/08-参考资料/notes/04-航向与磁辅助.md；
         NIS 门控 @ref REF-121 §7.3.1 p.255–259。
@frame  NED + FRD；磁矢量在机体系（µT）。
@units   µT、deg、无量纲 NIS/σ。
@note    结论"磁不标定不上机"：受扰磁裸用 SC3 647 m、加门控仍 96 m；悬停 SC7 与过渡
         SC4 磁无增益。该结论的外推边界见 @ref docs/08-参考资料/notes/05-仿真验证.md §7.1。
@see     upgrade_candidates.py、@ref docs/08-参考资料/notes/05-仿真验证.md
"""
import json
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))
sys.path.insert(0, _HERE)
from ekf_core import HOME_LLA, G_MPS2, quat_to_euler
from opt_ekf import OptEkf, build_cfg, quat_from_yp
from upgrade_candidates import (CUR_CFG, CUR_SPEC, MAG_B_NED, MAG_DEC,
                                UpgradeEkf, UpgradeScenario, _mag_up,
                                deep_merge, mag_azimuth_error, scenario_spec)

OUT = os.path.abspath(os.path.join(_HERE, "..", "..", "sim-data", "opt",
                                   "upgrade_mag_diag.json"))

PHASES = {
    "SC3_cruise_turn": [("static", 0, 2), ("cruise", 2, 32),
                        ("turn", 32, 52), ("cruise2", 52, 90)],
    "SC4_transition": [("static", 0, 3), ("transition", 3, 15),
                       ("hover", 15, 60)],
    "SC5_outage_reacq": [("static", 0, 2), ("cruise", 2, 30),
                         ("outage", 30, 80), ("multipath", 80, 95),
                         ("cruise2", 95, 120)],
    "SC7_hover_vib": [("hover", 0, 60)],
}

# 场景 -> (up 配置, 说明)
def _up(**mag):
    d = _mag_up(**mag)
    d["yaw_h2_fix"] = True
    return d


ABLATIONS = {
    "nomag": dict(up_extra={"yaw_h2_fix": True}),
    "ideal_cand": dict(up_extra=_up(mode="ideal")),
    "ideal_sigma3": dict(up_extra=_up(mode="ideal"),
                         ekf_opts={"mag": {"sigma_base_deg": 3.0,
                                           "k_norm": 0.0, "k_dip": 0.0,
                                           "k_track": 0.0, "k_att": 0.0}}),
    "ideal_no_tiltgate": dict(up_extra=_up(mode="ideal"),
                              ekf_opts={"mag": {"gate_tilt_deg": 90.0}}),
    "ideal_robust": dict(up_extra=_up(mode="ideal", robust=True)),
    "dist_gate": dict(up_extra=_up(mode="disturbed")),
    "dist_raw": dict(up_extra=_up(mode="disturbed", gate=False, adapt=False)),
}


def _task(task):
    name, scn, seeds, win = task
    ab = ABLATIONS[name]
    spec = deep_merge(scenario_spec(scn), CUR_SPEC)
    cfgd = build_cfg(CUR_CFG)
    up = deep_merge({"mag": None}, ab["up_extra"])
    up.setdefault("mag", None)
    if up.get("mag"):
        up["mag"] = deep_merge({"enable": True}, up["mag"])
    rows = []
    for seed in seeds:
        ekf_opts = ab.get("ekf_opts", {})
        sc = UpgradeScenario(spec, lambda: UpgradeEkf(cfgd,
                                                      dict(ekf_opts)), up)
        sc.run(seed=seed)
        t0, t1 = win if win else (None, None)
        m = dict(sc.metrics(t0=t0, t1=t1))
        r = sc.rec
        mask = np.ones(len(r["t"]), bool)
        if t0 is not None:
            mask &= r["t"] >= t0
        if t1 is not None:
            mask &= r["t"] <= t1
        m["att_rmse_deg"] = float(np.sqrt(np.mean(
            r["att_err"][mask] ** 2)) * 180 / math.pi)
        m["yaw_max_err_deg"] = float(np.max(np.abs(r["yaw_err"][mask]))
                                     * 180 / math.pi)
        kicks = [k * 180 / math.pi for (tt, s, k) in sc.ekf.kicks
                 if (t0 is None or tt >= t0) and (t1 is None or tt <= t1)]
        m["yaw_kick_max_deg"] = float(np.max(kicks)) if kicks else float("nan")
        # 分相位磁事件统计
        if sc.ekf.mag_log:
            for pname, a, b in PHASES[scn]:
                ev = [e for e in sc.ekf.mag_log if a <= e["t"] < b]
                d = dict(n=len(ev))
                if ev:
                    d["fused"] = sum(1 for e in ev if e.get("fused"))
                    for rs in ("norm", "dip", "track", "res", "tilt"):
                        d["rej_" + rs] = sum(
                            1 for e in ev if rs in (e.get("reasons") or []))
                    d["rej_nis"] = sum(
                        1 for e in ev if e.get("mag_fused") is False)
                    sig = [e["sigma"] for e in ev if "sigma" in e]
                    d["sigma_deg_mean"] = float(np.mean(
                        np.array(sig) * 180 / math.pi)) if sig else float("nan")
                    kk = [e["mag_kick"] for e in ev if "mag_kick" in e]
                    d["kick_max_deg"] = float(np.max(kk) * 180 / math.pi) \
                        if kk else float("nan")
                m["phase_" + pname] = d
        else:
            for pname, a, b in PHASES[scn]:
                m["phase_" + pname] = dict(n=0)
        rows.append(m)
    out = {}
    for k in sorted({k for r in rows for k in r}):
        vals = [r[k] for r in rows if k in r]
        if isinstance(vals[0], dict):
            keys = sorted({kk for v in vals for kk in v})
            out[k] = {}
            for kk in keys:
                vv = [v[kk] for v in vals if kk in v]
                out[k][kk] = dict(mean=float(np.mean(vv)),
                                  std=float(np.std(vv, ddof=1))
                                  if len(vv) > 1 else 0.0)
        else:
            vv = [float(v) for v in vals if not (isinstance(v, float)
                                                 and math.isnan(v))]
            if vv:
                out[k] = dict(mean=float(np.mean(vv)),
                              std=float(np.std(vv, ddof=1))
                              if len(vv) > 1 else 0.0)
    return dict(ablation=name, scenario=scn, metrics=out)


def _h2_single_point():
    """Q2：单点注入（航向 +10°）对比 baseline H2 与修正 H2"""
    def fresh():
        e = UpgradeEkf(build_cfg(CUR_CFG), {})
        e.home_lla = HOME_LLA.copy()
        e.initialize(np.array([0.0, 0.0, -G_MPS2]), np.zeros(3), HOME_LLA)
        e.P[8, 8] = math.radians(20.0) ** 2
        return e
    e1 = fresh()
    ba0 = e1.ba.copy()
    yaw_obs = math.radians(10.0)
    e1.measurement_update_yaw_h2(yaw_obs, math.radians(1.0))
    e2 = fresh()
    ba2 = e2.ba.copy()
    e2.yaw_update_h2_fixed(yaw_obs, math.radians(1.0))
    # 磁单点：真值航向 10°，估计 0°
    e3 = fresh()
    m_b = np.array([0.0, 0.0, 0.0])
    from ekf_core import quat_to_dcm
    m_b = quat_to_dcm(quat_from_yp(math.radians(10.0), 0, 0)).T @ MAG_B_NED
    y, az, _ = mag_azimuth_error(m_b, e3.quat, MAG_DEC)
    ba3 = e3.ba.copy()
    e3.mag_update(m_b, math.radians(1.0))
    return dict(
        baseline_h2=dict(yaw_after_deg=float(np.degrees(
            quat_to_euler(e1.quat)[0])),
            dba_x=float(e1.ba[0] - ba0[0])),
        fixed_h2=dict(yaw_after_deg=float(np.degrees(
            quat_to_euler(e2.quat)[0])),
            dba_x=float(e2.ba[0] - ba2[0])),
        mag_single=dict(y_deg=float(np.degrees(y)),
                        yaw_after_deg=float(np.degrees(
                            quat_to_euler(e3.quat)[0])),
                        dba_x=float(e3.ba[0] - ba3[0])),
        note="真值航向 10°、估计 0° 时的单次注入效果；正确应把航向推向 10°、加计零偏不动")


def main():
    """程序入口：跑磁门控相位统计与消融，写 upgrade_mag_diag.json。"""
    t0 = time.time()
    out = dict(meta=dict(generated=time.strftime("%Y-%m-%d %H:%M:%S"),
                         notes="候选 A 诊断；baseline 双矢量航向 H 索引错位证据"))
    out["h2_single_point"] = _h2_single_point()
    print("== Q2 单点注入 ==")
    print(json.dumps(out["h2_single_point"], ensure_ascii=False, indent=2))

    # Q1/Q3：门控时序 + 消融。SC3/SC4 用相位统计；SC5/SC7 重点看消融
    tasks = []
    for scn in ("SC3_cruise_turn", "SC4_transition", "SC5_outage_reacq",
                "SC7_hover_vib"):
        for name in ABLATIONS:
            tasks.append((name, scn, [1, 2, 3], None))
    print(f"\n[{len(tasks)} 组消融运行] ...")
    res = []
    with ProcessPoolExecutor(max_workers=10) as ex:
        for i, r in enumerate(ex.map(_task, tasks, chunksize=1)):
            res.append(r)
            if (i + 1) % 10 == 0:
                print(f"   {i + 1}/{len(tasks)}  {time.time() - t0:.0f}s")
    out["ablations"] = res
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"[完成] 写入 {OUT}  ({time.time() - t0:.0f}s)")

    # 打印紧凑表
    print("\n== Q1/Q3 消融（3 种子均值）==")
    for scn in ("SC3_cruise_turn", "SC4_transition", "SC5_outage_reacq",
                "SC7_hover_vib"):
        print(f"\n--- {scn}")
        print(f"{'variant':18s} {'pos_ne':>9s} {'vel_ne':>7s} {'yaw':>7s} "
              f"{'att':>7s} {'kick':>7s} {'magfuse':>8s} {'skip%':>6s}")
        for r in res:
            if r["scenario"] != scn:
                continue
            m = r["metrics"]
            g = lambda k, d=0.0: m[k]["mean"] if k in m else d
            # 融合率：从 phase 统计汇总
            tot = fuse = 0
            for pname, _, _ in PHASES.get(scn, []):
                ph = m.get("phase_" + pname, {})
                tot += ph.get("n", {}).get("mean", 0)
                fuse += ph.get("fused", {}).get("mean", 0)
            print(f"{r['ablation']:18s} {g('pos_ne_rmse'):9.2f} "
                  f"{g('vel_ne_rmse'):7.2f} {g('yaw_rmse_deg'):7.2f} "
                  f"{g('att_rmse_deg'):7.2f} {g('yaw_kick_max_deg'):7.2f} "
                  f"{fuse:8.0f} {(1 - fuse / max(tot, 1)) * 100:5.0f}%")
    print("\n== Q1 门控原因分解（候选形态 ideal_cand / dist_gate）==")
    for scn in PHASES:
        print(f"\n--- {scn}")
        for r in res:
            if r["scenario"] != scn or r["ablation"] not in (
                    "ideal_cand", "dist_gate"):
                continue
            print(f"  [{r['ablation']}]")
            for pname, _, _ in PHASES[scn]:
                ph = r["metrics"].get("phase_" + pname)
                if not ph or ph.get("n", {}).get("mean", 0) == 0:
                    continue
                gg = lambda k: ph.get(k, {}).get("mean", 0)
                print(f"    {pname:11s} n={gg('n'):4.0f} fused={gg('fused'):4.0f} "
                      f"norm={gg('rej_norm'):4.0f} dip={gg('rej_dip'):4.0f} "
                      f"track={gg('rej_track'):4.0f} res={gg('rej_res'):4.0f} "
                      f"tilt={gg('rej_tilt'):4.0f} nis={gg('rej_nis'):4.0f} "
                      f"σ={gg('sigma_deg_mean'):4.1f}° kick={gg('kick_max_deg'):5.2f}°")


if __name__ == "__main__":
    main()
