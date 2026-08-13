# -*- coding: utf-8 -*-
"""
高保真固件闭环仿真 —— EKF 变体验证
====================================
在 6DOF 动力学 + 固件控制律闭环中，用全固件语义导航（OptEkf：
三档静止辅助+ZUPT残差自适应+气压辅助+双矢量航向+延迟回放+重捕获）
对基准与候选变体做 A/B。

场景：
  H1 悬停+阵风 | H2 定高10m | H3 GPS丢失30s | H4 加计零偏跳变 | H5 倾斜20°
输出 sim-data/opt/hifi_variants.json
"""
import json
import math
import os
import sys

import numpy as np

_BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _BASE)
sys.path.insert(0, os.path.join(_BASE, "opt"))
sys.path.insert(0, os.path.join(_BASE, "hifi"))

from ekf_core import quat_to_euler, quat_to_dcm, quat_mult, rotvec_to_quat, \
    D2R, R2D
from hifi_core import P, CTRL, Dynamics, FirmwareControl, w_hover, \
    quat_from_euler
from opt_ekf import OptEkf, build_cfg, StaticDetector, StaticAid, FW, wrap_pi

OUT = os.path.join(_BASE, "..", "sim-data", "opt")
os.makedirs(OUT, exist_ok=True)

DT = 0.005
HZ = 200
Q_HOVER = quat_from_euler(0.0, 90.0 * D2R, 0.0)


class NavOpt:
    """全固件语义导航适配器（OptEkf）"""

    def __init__(self, cfgd, opts, seed):
        self.ekf = OptEkf(cfgd, opts)
        self.ekf.home_lla = np.array([28.2 * D2R, 112.9 * D2R, 50.0])
        self.det = StaticDetector(FW)
        self.aid = StaticAid(FW)
        self.aid_rng = np.random.default_rng(1000 + seed)
        self.gyro_avg = np.zeros(3)
        self.gyro_cnt = 0
        self.initialized = False
        self.vel_up = 0.0
        self.alt_up = 0.0
        self._last_gyro = np.zeros(3)
        self.gyro_bias_est = np.zeros(3)

    def init(self, accel, gyro):
        self.ekf.initialize(accel, gyro, self.ekf.home_lla)
        # 悬停构型姿态覆盖（最小旋转法无法识别悬停）
        self.ekf.quat = Q_HOVER.copy()
        self.ekf.Cnb = quat_to_dcm(Q_HOVER)
        nav_rate = np.array([0.0, 0.0, 0.0])  # 简化：本地静止地球项小
        self.ekf.bg = np.array(gyro, float) - self.ekf.Cnb.T @ nav_rate
        self.initialized = True

    def time_update(self, accel, gyro, dt):
        if not self.initialized:
            return
        self.ekf.time_update(gyro * dt, np.zeros(3), accel * dt, np.zeros(3), dt)
        static_now = self.det.update(accel, gyro)
        if static_now:
            self.gyro_avg += gyro
            self.gyro_cnt += 1
            tier = self.aid.tier(self.det)
            g_n, v_n, v_d = self.aid.noise(tier)
            dz, dg, ds = self.aid.divs(tier)
            self.aid.frame_cnt += 1
            c = self.aid.frame_cnt
            if c % dz == 0:
                self.ekf.measurement_update_velocity_adaptive(
                    np.zeros(3), v_n, v_d)
            if c % dg == 1:
                self.ekf.measurement_update_gravity(accel, g_n)
            if c % ds == 3:
                self.ekf.measurement_update_static_gyro(
                    self.gyro_avg / max(self.gyro_cnt, 1),
                    np.full(3, FW["sg_noise"]))
                self.gyro_avg = np.zeros(3)
                self.gyro_cnt = 0
        self.gyro_bias_est = self.ekf.bg.copy()
        self.vel_up = -self.ekf.vel[2]
        self.alt_up = self.ekf.lla[2] - self.ekf.home_lla[2]
        self._last_gyro = gyro.copy()

    def gps_update(self, pos_ned, vel_ned):
        if not self.initialized:
            return
        R_std = [2.0, 3.0, 0.15, 0.25]
        self.ekf.measurement_update_gnss(pos_ned, vel_ned, R_std,
                                         age_s=0.015, hist=self.ekf.hist)
        self.gyro_bias_est = self.ekf.bg.copy()
        self.vel_up = -self.ekf.vel[2]
        self.alt_up = self.ekf.lla[2] - self.ekf.home_lla[2]

    def baro_update(self, alt_up_m):
        if not self.initialized:
            return
        self.ekf.measurement_update_baro(alt_up_m, FW["baro_noise"])

    def dual_vector_yaw(self, gps_vel_ned, flow_body, gyro):
        if not self.initialized:
            return
        speed_g = float(np.hypot(gps_vel_ned[0], gps_vel_ned[1]))
        speed_f = float(np.hypot(flow_body[0], flow_body[1]))
        turn = float(np.linalg.norm(gyro))
        if speed_g < FW["dv_min_speed"] or speed_f < FW["dv_min_speed"]:
            return
        if turn > FW["dv_max_turn"]:
            return
        if abs(speed_f / max(speed_g, 1e-9) - 1.0) > FW["dv_ratio_tol"]:
            return
        track = math.atan2(gps_vel_ned[1], gps_vel_ned[0])
        slip = math.atan2(flow_body[1], max(flow_body[0], 1e-9))
        est_yaw = wrap_pi(track - slip)
        noise = max(FW["dv_min_noise"], FW["dv_base_noise"] / speed_g)
        self.ekf.measurement_update_yaw_h2(est_yaw, noise)

    def get_nav(self):
        return {
            "quat": self.ekf.quat,
            "omega_body": self._last_gyro - self.gyro_bias_est,
            "gyro_bias": self.gyro_bias_est.copy(),
            "alt_up": self.alt_up,
            "vel_up": self.vel_up,
            "pos": self.ekf.ned_from_lla(self.ekf.lla, self.ekf.home_lla),
        }


SCENES = {
    "H1_hover_gust": dict(duration=30.0,
                          fault={"wind_gust": (10.0, 15.0)}, ref={}),
    "H2_althold": dict(duration=30.0, fault={}, ref={"alt_hold": 10.0}),
    "H3_gps_loss": dict(duration=60.0,
                        fault={"gps_lost": (15.0, 45.0)}, ref={}),
    "H4_accel_jump": dict(duration=40.0,
                          fault={"accel_bias_jump": np.array([1.0, 0, 0]),
                                 "accel_bias_jump_t": 10.0}, ref={}),
    "H5_tilt20": dict(duration=30.0, fault={}, ref={"tilt": (10.0, 20.0)}),
}


def run_scene(scene, duration, nav_factory, seed=1, seed_dyn=1):
    dyn = Dynamics(seed=seed_dyn)
    nav = nav_factory()
    ctrl = FirmwareControl()
    fault = scene.get("fault", {})
    ref = scene.get("ref", {})
    q_target = Q_HOVER.copy()
    dyn.quat = Q_HOVER.copy()

    t_arr, att_t, att_e, pos_t, pos_e, vel_t, vel_e = [], [], [], [], [], [], []
    alt_arr, ealt_arr, gps_ok = [], [], []

    accel0, gyro0 = dyn.imu_measure(0.0)
    nav.init(accel0, gyro0)
    baro_cnt = 0

    for k in range(int(duration / DT)):
        t = k * DT
        dyn.disturb = np.zeros(3)
        if fault.get("wind_gust") and fault["wind_gust"][0] < t < fault["wind_gust"][1]:
            dyn.disturb[1] = 3.0
        if fault.get("thrust_drop") and t > fault["thrust_drop"]:
            dyn.disturb[0] = -2.0

        accel, gyro = dyn.imu_measure(t, fault)
        nav.time_update(accel, gyro, DT)

        gps = None
        if k % (HZ // 10) == 0:
            gps = dyn.gps_measure(t, fault)
            if gps is not None:
                nav.gps_update(gps[0], gps[1])
        # 双矢量航向（GPS 帧）
        if gps is not None:
            flow = np.array([dyn.vel[0], dyn.vel[1]]) + \
                np.random.default_rng(500 + k).normal(0, 0.05 * max(
                    1.0, float(np.linalg.norm(dyn.vel[:2]))), 2)
            nav.dual_vector_yaw(gps[1], flow, gyro)
        # 气压 25Hz
        if k % (HZ // 25) == 0:
            nav.baro_update(dyn.baro_measure(t))

        q_ref = Q_HOVER.copy()
        if ref.get("tilt") and t > ref["tilt"][0]:
            q_ref = quat_mult(Q_HOVER, rotvec_to_quat(
                np.array([0, ref["tilt"][1] * D2R, 0])))
        ref_dict = {
            "q_target": q_ref,
            "thr": ref.get("thr", 50.0),
            "alt_hold": ref.get("alt_hold"),
            "yaw_lock": ref.get("yaw_lock"),
        }
        act = ctrl.step(nav.get_nav(), ref_dict, DT)
        dyn.step(act, DT)

        # 记录（四元数距离评估姿态）
        t_arr.append(t)
        att_t.append(dyn.quat.copy())
        att_e.append(nav.ekf.quat.copy())
        pos_t.append(dyn.pos.copy())
        pos_e.append(nav.get_nav()["pos"])
        vel_t.append(quat_to_dcm(dyn.quat) @ dyn.vel)
        vel_e.append(nav.ekf.vel.copy())
        alt_arr.append(-dyn.pos[2])
        ealt_arr.append(nav.alt_up)
        gps_ok.append(1 if gps is not None else 0)

    def qdist(qa, qb):
        """估计/真值四元数旋转角误差（用共轭，非取反）"""
        from ekf_core import quat_conj
        qe = quat_mult(qa, quat_conj(qb))
        return 2.0 * math.acos(max(-1.0, min(1.0, abs(qe[0]))))

    t_arr = np.array(t_arr)
    pos_t = np.array(pos_t)
    pos_e = np.array(pos_e)
    vel_t = np.array(vel_t)
    vel_e = np.array(vel_e)
    att_qd = np.array([qdist(qt, qe) for qt, qe in zip(att_t, att_e)])
    return dict(t=t_arr, pos_err=(pos_e - pos_t), vel_err=(vel_e - vel_t),
                att_qerr=att_qd, alt=np.array(alt_arr),
                ealt=np.array(ealt_arr), gps_ok=np.array(gps_ok))


def summarize(r, t0, t1):
    m = (r["t"] >= t0) & (r["t"] <= t1)
    pe = r["pos_err"][m]
    return dict(
        pos_ne_rmse=float(np.sqrt(np.mean(pe[:, 0]**2 + pe[:, 1]**2))),
        pos_d_rmse=float(np.sqrt(np.mean(pe[:, 2]**2))),
        vel_ne_rmse=float(np.sqrt(np.mean(r["vel_err"][m, 0]**2 +
                                          r["vel_err"][m, 1]**2))),
        att_q_rmse_deg=float(np.sqrt(np.mean(r["att_qerr"][m]**2)) * R2D),
        alt_rmse=float(np.sqrt(np.mean(r["alt"][m]**2))),
    )


VARIANTS = {
    "base": (dict(), dict()),
    "baro_off": (dict(), dict(no_baro=True)),
    "q_calib": (dict(accel_std=0.05), dict()),
    "r_tight": (dict(), dict(r_floor=[1.0, 1.5, 0.075, 0.125])),
    "adr": (dict(), dict(adr=True)),
    "combo": (dict(accel_std=0.05), dict(r_floor=[1.0, 1.5, 0.075, 0.125],
                                        adr=True)),
    "zupt_nis16": (dict(nis_zupt=16.3), dict()),
}

WINDOWS = {
    "H1_hover_gust": (5.0, 30.0),
    "H2_althold": (8.0, 30.0),
    "H3_gps_loss": (15.0, 45.0),
    "H4_accel_jump": (12.0, 40.0),
    "H5_tilt20": (12.0, 30.0),
}


class NavV(NavOpt):
    """支持变体覆盖的导航适配器：R 地板 / 气压开关 / 初始航向 P0 / 静态门控"""

    def __init__(self, cfgd, opts, seed=1, r_floor=None, no_baro=False,
                 p0_hdg=None, static_gate=False):
        super().__init__(cfgd, opts, seed)
        self.r_floor = r_floor
        self.no_baro = no_baro
        self.p0_hdg = p0_hdg
        self.static_gate = static_gate

    def init(self, accel, gyro):
        super().init(accel, gyro)
        if self.p0_hdg is not None:
            self.ekf.P[8, 8] = self.p0_hdg**2

    def gps_update(self, pos_ned, vel_ned):
        if not self.initialized:
            return
        # 静态门控：GNSS 速度超阈 → 强制退出静止（防误静止巡航）
        if self.static_gate and \
                float(np.hypot(vel_ned[0], vel_ned[1])) > 1.5:
            self.det.force_exit()
        R = self.r_floor if self.r_floor else [2.0, 3.0, 0.15, 0.25]
        self.ekf.measurement_update_gnss(pos_ned, vel_ned, R,
                                         age_s=0.015, hist=self.ekf.hist)

    def baro_update(self, alt_up_m):
        if self.no_baro:
            return
        super().baro_update(alt_up_m)


VARIANTS = {
    "firmware": dict(cfg={}, opts={}, nav=dict()),
    "p0fix": dict(cfg={}, opts={}, nav=dict(p0_hdg=0.2)),
    "gate": dict(cfg={}, opts={}, nav=dict(static_gate=True)),
    "fixbundle": dict(cfg={},
                      opts=dict(yaw_p_floor=0.05, yaw_reacq=True),
                      nav=dict(p0_hdg=0.2, static_gate=True)),
    "baro_off": dict(cfg={}, opts={}, nav=dict(no_baro=True, p0_hdg=0.2)),
    "q_calib": dict(cfg={"accel_std": 0.05}, opts={}, nav=dict(p0_hdg=0.2)),
    "r_tight": dict(cfg={}, opts={},
                    nav=dict(p0_hdg=0.2, r_floor=[1.0, 1.5, 0.075, 0.125])),
    "adr": dict(cfg={}, opts=dict(adaptive_r=True, adr_window=40,
                                  adr_max_scale=8.0), nav=dict(p0_hdg=0.2)),
    "combo": dict(cfg={"accel_std": 0.05},
                  opts=dict(adaptive_r=True, adr_window=40,
                            adr_max_scale=8.0),
                  nav=dict(p0_hdg=0.2, static_gate=True,
                           r_floor=[1.0, 1.5, 0.075, 0.125])),
}


def main():
    out = {"variants": {}}
    for vname, v in VARIANTS.items():
        cfgd = build_cfg(v["cfg"])
        o = v["opts"]
        navp = v["nav"]

        def nav_factory():
            return NavV(cfgd, o, r_floor=navp.get("r_floor"),
                        no_baro=navp.get("no_baro", False),
                        p0_hdg=navp.get("p0_hdg"),
                        static_gate=navp.get("static_gate", False))

        res = {}
        for scn, sc in SCENES.items():
            r = run_scene(sc, sc["duration"], nav_factory)
            t0, t1 = WINDOWS[scn]
            res[scn] = summarize(r, t0, t1)
            print(f"[{vname}/{scn}] pos_ne={res[scn]['pos_ne_rmse']:.3f} "
                  f"pos_d={res[scn]['pos_d_rmse']:.3f} "
                  f"att={res[scn]['att_q_rmse_deg']:.2f}°")
        out["variants"][vname] = res
    with open(os.path.join(OUT, "hifi_variants.json"), "w",
              encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"\n[完成] 写入 {os.path.join(OUT, 'hifi_variants.json')}")


if __name__ == "__main__":
    main()
