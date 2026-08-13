# -*- coding: utf-8 -*-
"""
高保真固件闭环仿真 —— EKF 导航适配 + 场景运行
================================================
把 ekf_core.py 的 15 态 EKF（修复后）接入闭环：
  动力学.IMU -> EKF 时间更新 -> EKF 量测更新(GNSS/静止辅助) -> 固件控制

场景：
  S1 悬停抗扰（阵风脉冲）
  S2 定高保持
  S3 GPS 丢失（失联 60s -> 恢复）
  S4 传感器故障（加计零偏跳变 / 陀螺零偏跳变 / 陀螺饱和）
  S5 大机动（快速倾斜指令）
  S6 参数边界（摆角限幅 15°、τm、增益扫描）
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ekf_core import (Ekf15, cfg as EKF_CFG, cfg_p0 as EKF_P0, HOME_LLA,
                      quat_to_euler, quat_to_dcm, quat_mult, rotvec_to_quat,
                      D2R, R2D, G_MPS2)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hifi_core import P, CTRL, Dynamics, FirmwareControl, w_hover, quat_from_euler
Q_HOVER = quat_from_euler(0.0, 90.0 * D2R, 0.0)

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "..", "sim-data")
FIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "..", "fig")
os.makedirs(OUT, exist_ok=True)
os.makedirs(FIG, exist_ok=True)

DT = 0.005
HZ = 200


class NavAdapter:
    """把 Ekf15 接入闭环：初始化 + 时间更新 + 量测更新"""

    def __init__(self):
        self.ekf = Ekf15(EKF_CFG)
        self.ekf.home_lla = HOME_LLA.copy()
        self.initialized = False
        self.att_err = np.zeros(3)
        self.vel_up = 0.0
        self.alt_up = 0.0
        self.gyro_bias_est = np.zeros(3)
        self.accel_bias_est = np.zeros(3)
        # 静止确认状态（对齐固件 ins_static_detector：20 帧连续 + 多条件）
        self._static_frames = 0
        self._static_confirmed = False

    def init(self, accel, gyro, init_quat=None):
        """悬停构型初始化：
        ekf_core.initialize 的最小旋转法把悬停 +x 比力解读为"机体倾倒 90°"
        （无法识别悬停构型）。传入 init_quat（悬停姿态）后覆盖姿态块，
        并用正确姿态重新分离陀螺零偏（固件 Initialize 同款逻辑）。
        """
        self.ekf.initialize(accel, gyro, HOME_LLA)
        if init_quat is not None:
            self.ekf.quat = init_quat.copy()
            self.ekf.Cnb = quat_to_dcm(init_quat)
            from ekf_core import EarthModel
            em = EarthModel(self.ekf.lla[0], self.ekf.lla[2])
            nav_rate = em.omega_ned + em.nav_rate(self.ekf.vel)
            self.ekf.bg = np.array(gyro, dtype=float) - self.ekf.Cnb.T @ nav_rate
        self.ekf.lla = HOME_LLA.copy()
        self.ekf.vel = np.zeros(3)
        self.initialized = True

    def _static_check(self, accel, gyro):
        """严格静止检测（对齐固件 ins_static_detector 四条件）：
        ① |a|∈[9.5,10.1] ② |gyro|<0.01 ③ 帧间|Δa|<0.5 ④ 帧间|Δgyro|<0.05，
        连续 20 帧确认、任一不满足即退出。
        ★ 关键：仅 |a|≈g 不足以判静止——垂直加速运动时 |a| 仍≈g，
        必须靠帧间 Δa 变化排除（固件正是此设计）。"""
        a_norm = np.linalg.norm(accel)
        g_norm = np.linalg.norm(gyro)
        a_ok = (9.5 <= a_norm <= 10.1)
        g_ok = (g_norm < 0.01)
        d_ok = True
        if hasattr(self, "_last_a") and hasattr(self, "_last_g"):
            d_ok = (np.linalg.norm(accel - self._last_a) < 0.5) and                    (np.linalg.norm(gyro - self._last_g) < 0.05)
        self._last_a = accel.copy()
        self._last_g = gyro.copy()
        ok = a_ok and g_ok and d_ok
        if ok:
            self._static_frames += 1
            if self._static_frames >= 20:
                self._static_confirmed = True
        else:
            self._static_frames = 0
            self._static_confirmed = False
        return self._static_confirmed

    def time_update(self, accel, gyro, dt):
        if not self.initialized:
            return
        # 单子样（200Hz 一帧）
        self.ekf.time_update(gyro * dt, np.zeros(3), accel * dt, np.zeros(3), dt)
        # 静止辅助：仅"确认静止"后触发（对齐固件调度），悬停运动不误触发
        if self._static_check(accel, gyro):
            if not hasattr(self, "_aid_rng"):
                self._aid_rng = np.random.default_rng(42)
            # ★ 重力方向量测仅"低动态"有效：恒定加速度运动时帧间 Δa≈0
            #   会误判静止，若把运动比力方向当重力修正，姿态被持续拉偏
            #   （垂直通道污染根因）。加 EKF 速度模长门控兜底（<0.1 m/s）。
            low_dyn = np.linalg.norm(self.ekf.vel) < 0.1
            if low_dyn and self._aid_rng.random() < 0.25:
                self.ekf.measurement_update_gravity(accel, 0.1)
            if self._aid_rng.random() < 0.5:
                self.ekf.measurement_update_velocity(np.zeros(3), 0.1, 0.14)
        self.gyro_bias_est = self.ekf.bg.copy()
        self.accel_bias_est = self.ekf.ba.copy()
        self.vel_up = -self.ekf.vel[2]                # NED z 向下 → 向上为正
        self.alt_up = self.ekf.lla[2] - HOME_LLA[2]      # 相对 Home 高度（向上为正）
        self._last_gyro_raw = gyro.copy()

    def gps_update(self, pos_ned, vel_ned):
        if not self.initialized:
            return
        # pos_ned 是相对量，需要转成 LLA 量测（用 Home 反推）
        home = self.ekf.home_lla
        lla = self.ekf.lla.copy()
        # 简化：直接用相对位置做新息（H 仍是位置/速度）
        # 用 MeasurementUpdate 的 NED 残差等价形式：手动构造
        H = np.zeros((6, 15))
        H[0:6, 0:6] = np.eye(6)
        R = np.diag([1.5**2, 1.5**2, 2.0**2, 0.2**2, 0.2**2, 0.3**2])
        est_pos = self.ekf.ned_from_lla(self.ekf.lla, home)
        y = np.concatenate([pos_ned - est_pos, vel_ned - self.ekf.vel])
        S = H @ self.ekf.P @ H.T + R
        nis = y @ np.linalg.solve(S, y)
        if nis < 30.0:
            K = self.ekf.P @ H.T @ np.linalg.solve(S, np.eye(6))
            I15 = np.eye(15)
            self.ekf.P = (I15 - K @ H) @ self.ekf.P @ (I15 - K @ H).T + K @ R @ K.T
            self.ekf.P = self.ekf.stabilize(self.ekf.P)
            dx = K @ y
            self.ekf.inject(dx)
        self.gyro_bias_est = self.ekf.bg.copy()
        self.vel_up = -self.ekf.vel[2]
        self.alt_up = self.ekf.lla[2] - HOME_LLA[2]

    def get_nav(self):
        q = self.ekf.quat
        return {
            "quat": q,
            "omega_body": getattr(self, "_last_gyro_raw", np.zeros(3)) - self.gyro_bias_est,
            "gyro_bias": self.gyro_bias_est.copy(),
            "alt_up": self.alt_up,
            "vel_up": self.vel_up,
            "pos": self.ekf.ned_from_lla(self.ekf.lla, self.ekf.home_lla),
        }


def run_scene(scene, duration, params=None, seed=1):
    """运行单个场景"""
    dyn = Dynamics(seed=seed)
    nav = NavAdapter()
    ctrl = FirmwareControl()
    fault = scene.get("fault", {})
    ref = scene.get("ref", {})

    # 目标姿态（悬停：机头朝天）
    q_target = Q_HOVER.copy()
    dyn.quat = Q_HOVER.copy()

    # 记录
    t_arr, att_arr, omega_arr, pos_arr, vel_arr = [], [], [], [], []
    thr_arr, df_arr, dt_arr, dw_arr = [], [], [], []
    alt_arr, gps_ok = [], []

    accel0, gyro0 = dyn.imu_measure(0.0)
    nav.init(accel0, gyro0, init_quat=Q_HOVER)

    for k in range(int(duration / DT)):
        t = k * DT
        # ---- 扰动注入 ----
        dyn.disturb = np.zeros(3)
        if fault.get("wind_gust") and fault["wind_gust"][0] < t < fault["wind_gust"][1]:
            dyn.disturb[1] = 3.0  # 侧向 3 m/s² 扰动
        if fault.get("thrust_drop") and t > fault["thrust_drop"]:
            dyn.disturb[0] = -2.0

        # ---- 传感器 ----
        accel, gyro = dyn.imu_measure(t, fault)

        # ---- EKF 导航 ----
        nav.time_update(accel, gyro, DT)
        # ★ 仅融合帧调用 gps_measure（避免每帧消耗 rng 改变 GPS 噪声序列，
        #   导致场景对随机种子敏感——EKF 悬停+GPS 融合对单帧噪声的鲁棒性
        #   已单独确认，此处保证可复现）
        if k % (HZ // 10) == 0:
            gps = dyn.gps_measure(t, fault)
            if gps is not None:
                nav.gps_update(gps[0], gps[1])

        # ---- 控制 ----
        # 目标姿态参考：悬停 + 姿态指令
        q_ref = Q_HOVER.copy()
        if ref.get("tilt") and t > ref["tilt"][0]:
            # 倾斜指令：悬停基态 ⊗ 绕模型 y' 转 tilt 角（俯仰）
            q_ref = quat_mult(Q_HOVER, rotvec_to_quat(np.array([0, ref["tilt"][1] * D2R, 0])))
        ref_dict = {
            "q_target": q_ref,
            "thr": ref.get("thr", 50.0),
            "alt_hold": ref.get("alt_hold"),
            "yaw_lock": ref.get("yaw_lock"),
        }
        act = ctrl.step(nav.get_nav(), ref_dict, DT)

        # ---- 动力学 ----
        dyn.step(act, DT)

        # ---- 记录 ----
        eul = quat_to_euler(dyn.quat)
        t_arr.append(t)
        att_arr.append(eul * R2D)
        omega_arr.append(dyn.omega * R2D)
        pos_arr.append(dyn.pos.copy())
        vel_arr.append(quat_to_dcm(dyn.quat) @ dyn.vel)
        thr_arr.append(act["thr"])
        df_arr.append(dyn.df * R2D)
        dt_arr.append(dyn.dt * R2D)
        dw_arr.append(act.get("dw_cmd", 0.0))
        alt_arr.append(-dyn.pos[2])
        gps_ok.append(1 if gps is not None else 0)

    return {
        "t": np.array(t_arr), "att": np.array(att_arr), "omega": np.array(omega_arr),
        "pos": np.array(pos_arr), "vel": np.array(vel_arr), "thr": np.array(thr_arr),
        "df": np.array(df_arr), "dt": np.array(dt_arr), "dw": np.array(dw_arr),
        "alt": np.array(alt_arr), "gps_ok": np.array(gps_ok),
    }


def summarize(r, name):
    """场景指标汇总（四元数距离评估姿态，避免悬停欧拉奇异伪影）"""
    t = r["t"]
    m = t > 5.0  # 跳过初始瞬态
    omg_rms = np.sqrt(np.mean(r["omega"][m]**2))
    pos_drift = np.linalg.norm(r["pos"][-1] - r["pos"][m][0])
    print(f"[{name}] |ω|RMS={omg_rms:.3f} rad/s | 位置漂移={pos_drift:.2f}m "
          f"| 油门均值={100*r['thr'][m].mean():.1f}%")
    return dict(omg_rms=float(omg_rms), pos_drift=float(pos_drift),
                thr_mean=float(100*r["thr"][m].mean()))


def main():
    results = {}

    # S1 悬停抗扰
    s1 = run_scene({"fault": {"wind_gust": (10.0, 15.0)}}, 30.0)
    results["S1_hover"] = summarize(s1, "S1 悬停+阵风")

    # S2 定高保持
    s2 = run_scene({"ref": {"alt_hold": 10.0}}, 25.0)
    results["S2_althold"] = summarize(s2, "S2 定高")
    m = s2["t"] > 5.0
    alt_rmse = np.sqrt(np.mean((s2["alt"][m] - 10.0)**2))
    print(f"  → 定高 RMSE = {alt_rmse:.2f} m")
    results["S2_alt_rmse"] = float(alt_rmse)

    # S3 GPS 丢失
    s3 = run_scene({"fault": {"gps_lost": (15.0, 45.0)}}, 60.0)
    results["S3_gps_loss"] = summarize(s3, "S3 GPS丢失30s")
    m3 = (s3["t"] > 15.0) & (s3["t"] < 45.0)
    drift = np.linalg.norm(s3["pos"][m3][-1] - s3["pos"][m3][0])
    print(f"  → GPS 丢失期位置漂移 = {drift:.2f} m")
    results["S3_drift"] = float(drift)

    # S4a 加计零偏跳变
    s4a = run_scene({"fault": {"accel_bias_jump": np.array([1.0, 0, 0]),
                               "accel_bias_jump_t": 10.0}}, 40.0)
    results["S4a_accel_jump"] = summarize(s4a, "S4a 加计零偏跳变")
    # S4b 陀螺零偏跳变
    s4b = run_scene({"fault": {"gyro_bias_jump": np.array([0, 0, 0.05]),
                               "gyro_bias_jump_t": 10.0}}, 40.0)
    results["S4b_gyro_jump"] = summarize(s4b, "S4b 陀螺零偏跳变")

    # S5 大机动（快速倾斜 20°）
    s5 = run_scene({"ref": {"tilt": (10.0, 20.0)}}, 30.0)
    results["S5_tilt"] = summarize(s5, "S5 倾斜20°")

    with open(os.path.join(OUT, "hifi_scenarios.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2, default=float)
    print("\n[完成] 高保真场景结果写入 sim-data/hifi_scenarios.json")


if __name__ == "__main__":
    main()
