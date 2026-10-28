# -*- coding: utf-8 -*-
"""fwapi.ekf — 固件 Ekf15State 的 Python 直调封装。

@file    ekf.py
@module  analysis/固件算法直调（EKF）
@brief   用 numpy 数组驱动固件 15 态 ESKF；不做任何算法复刻，参数与门限全部透传。
@details 典型用法：
             from fwapi import Ekf
             ekf = Ekf()
             ekf.initialize(accel, gyro, mag, vel, lla)
             ekf.time_update_two_sample(dth1, dth2, dv1, dv2, 0.005)
             r = ekf.gnss_update(vel_ned, lla, 0.005, age_s=0.0)
             s = ekf.state()
         所有 update 返回 dict：input_valid/fused/nis_*/innovation_norm/... 。
@units  SI：m、m/s、m/s²、rad、rad/s、s；LLA=(纬度 rad, 经度 rad, 椭球高 m)。
@see    TandemVec_FCS/lib/navigation-main/src/ekf_15_state.h
"""
from __future__ import annotations

import ctypes
import json

import numpy as np

from . import library


def _f3(v) -> "ctypes.c_float * 3":
    a = np.asarray(v, dtype=np.float32).reshape(3)
    return (ctypes.c_float * 3)(*a)  # type: ignore[attr-defined]


def _d3(v) -> "ctypes.c_double * 3":
    a = np.asarray(v, dtype=np.float64).reshape(3)
    return (ctypes.c_double * 3)(*a)  # type: ignore[attr-defined]


def _result(r: library.FwUpdateResult) -> dict:
    return {
        "input_valid": bool(r.input_valid),
        "attempted": bool(r.attempted),
        "fused": bool(r.fused),
        "reacquired": bool(r.reacquired),
        "outlier_rejected": bool(r.outlier_rejected),
        "subset_position": bool(r.subset_position),
        "subset_velocity": bool(r.subset_velocity),
        "nis_pos3": float(r.nis_pos3),
        "nis_vel3": float(r.nis_vel3),
        "innovation_norm": float(r.innovation_norm),
        "test_ratio": float(r.test_ratio),
        "yaw_correction_rad": float(r.yaw_correction_rad),
        "yaw_correction_clamped": bool(r.yaw_correction_clamped),
    }


class Ekf:
    """固件 15 态 ESKF 的句柄封装（一个实例对应一段独立的滤波状态）。"""

    def __init__(self, auto_build: bool = True):
        self._lib = library.load(force_build=not auto_build)
        self._h = self._lib.fw_ekf_create()
        if not self._h:
            raise MemoryError("fw_ekf_create 返回空句柄")
        self._closed = False

    # ---- 生命周期 ----
    def close(self) -> None:
        if not self._closed and self._h:
            self._lib.fw_ekf_destroy(self._h)
            self._h = None
            self._closed = True

    def __del__(self):  # pragma: no cover - 析构兜底
        try:
            self.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _check(self) -> None:
        if self._closed or not self._h:
            raise RuntimeError("EKF 句柄已关闭")

    # ---- 初始化 ----
    def initialize(self, accel, gyro, mag=(0, 0, 0), vel=(0, 0, 0), lla=None,
                   yaw_offset_rad: float = 0.0) -> None:
        self._check()
        if lla is None:
            raise ValueError("必须给出初始 lla=(纬度 rad, 经度 rad, 高度 m)")
        init = library.FwEkfInit()
        init.accel_mps2 = _f3(accel)
        init.gyro_radps = _f3(gyro)
        init.mag_ut = _f3(mag)
        init.vel_ned_mps = _f3(vel)
        init.lla_rad_m = _d3(lla)
        init.yaw_offset_rad = float(yaw_offset_rad)
        rc = self._lib.fw_ekf_init(self._h, ctypes.byref(init))  # type: ignore[attr-defined]
        if rc != 0:
            raise RuntimeError(f"fw_ekf_init 失败：{rc}")

    # ---- 传播 ----
    def time_update(self, accel, gyro, dt_s: float) -> None:
        """单子样传播：accel/gyro 为已去零偏的机体系量。"""
        self._check()
        self._lib.fw_ekf_time_update(self._h, _f3(accel), _f3(gyro), float(dt_s))

    def time_update_two_sample(self, dth1, dth2, dv1, dv2, dt_s: float) -> None:
        """双子样传播（固件生产口径）：增量含零偏，函数内部按 half_dt 扣除。"""
        self._check()
        self._lib.fw_ekf_time_update_two_sample(
            self._h, _f3(dth1), _f3(dth2), _f3(dv1), _f3(dv2), float(dt_s))

    # ---- 量测更新 ----
    def gnss_update(self, vel_ned, lla, dt_avg_s: float,
                    measurement_age_s: float = 0.0,
                    require_delayed_snapshot: bool = False) -> dict:
        self._check()
        r = self._lib.fw_ekf_gnss_update(
            self._h, _f3(vel_ned), _d3(lla), float(dt_avg_s),
            float(measurement_age_s), 1 if require_delayed_snapshot else 0)
        return _result(r)

    def mag_update(self, mag_body_ut, mag_ned_ut, noise_ut: float) -> dict:
        self._check()
        r = self._lib.fw_ekf_mag_update(
            self._h, _f3(mag_body_ut), _f3(mag_ned_ut), float(noise_ut))
        return _result(r)

    def baro_update(self, altitude_m: float, noise_std_m: float = 0.4) -> dict:
        self._check()
        r = self._lib.fw_ekf_baro_update(self._h, float(altitude_m), float(noise_std_m))
        return _result(r)

    def aiding_velocity_update(self, vel_ned, vel_ne_std_mps: float = 1.0,
                               vel_d_std_mps: float = 1.0) -> dict:
        self._check()
        r = self._lib.fw_ekf_aiding_velocity_update(
            self._h, _f3(vel_ned), float(vel_ne_std_mps), float(vel_d_std_mps))
        return _result(r)

    def gravity_update(self, accel_mps2, noise_rad: float = 0.05) -> dict:
        self._check()
        r = self._lib.fw_ekf_gravity_update(self._h, _f3(accel_mps2), float(noise_rad))
        return _result(r)

    def static_gyro_update(self, gyro_radps, noise_radps: float = 0.002) -> dict:
        self._check()
        r = self._lib.fw_ekf_static_gyro_update(
            self._h, _f3(gyro_radps), float(noise_radps))
        return _result(r)

    def yaw_update(self, yaw_rad: float, noise_rad: float = 0.05) -> dict:
        self._check()
        r = self._lib.fw_ekf_yaw_update(self._h, float(yaw_rad), float(noise_rad))
        return _result(r)

    # ---- 复位 ----
    def reset_position_velocity(self, vel_ned, lla) -> None:
        self._check()
        self._lib.fw_ekf_reset_position_velocity(self._h, _f3(vel_ned), _d3(lla))

    def reset_velocity(self, vel_ned, vel_std_mps: float = 0.5) -> None:
        self._check()
        self._lib.fw_ekf_reset_velocity(self._h, _f3(vel_ned), float(vel_std_mps))

    # ---- 配置 ----
    def set_gnss_gates(self, consider_heading: bool = False,
                       reanchor_allowed: bool = True,
                       vertical_reanchor_allowed: bool = True) -> None:
        self._check()
        self._lib.fw_ekf_set_gnss_gates(
            self._h, int(consider_heading), int(reanchor_allowed),
            int(vertical_reanchor_allowed))

    def set_velocity_subset(self, enabled: bool = True,
                            max_pos_resid_m: float = 25.0,
                            max_fires_per_epoch: int = 1) -> None:
        self._check()
        self._lib.fw_ekf_set_velocity_subset(
            self._h, int(enabled), float(max_pos_resid_m), int(max_fires_per_epoch))

    def set_imu_limits(self, accel_mps2: float = 120.0,
                       gyro_radps: float = 40.0) -> None:
        self._check()
        self._lib.fw_ekf_set_imu_limits(self._h, float(accel_mps2), float(gyro_radps))

    def set_q_accel_scale(self, scale: float) -> None:
        self._check()
        self._lib.fw_ekf_set_q_accel_scale(self._h, float(scale))

    def set_lever_arm(self, lever_body_m) -> None:
        self._check()
        self._lib.fw_ekf_set_lever_arm(self._h, _f3(lever_body_m))

    # ---- 读取 ----
    def state(self) -> dict:
        """返回扁平状态字典（numpy 数组 + 标量），便于直接组 DataFrame/JSONL。"""
        self._check()
        s = library.FwNavState()
        self._lib.fw_ekf_get_state(self._h, ctypes.byref(s))  # type: ignore[attr-defined]
        return {
            "lla": np.array(s.lla_rad_m, dtype=np.float64),
            "vel": np.array(s.vel_ned_mps, dtype=np.float32),
            "quat": np.array(s.quat_wxyz, dtype=np.float32),
            "ypr": np.array(s.euler_ypr_rad, dtype=np.float32),
            "ba": np.array(s.accel_bias_mps2, dtype=np.float32),
            "bg": np.array(s.gyro_bias_radps, dtype=np.float32),
            "yaw_sigma": float(s.yaw_sigma_rad),
            "tilt_sigma": float(s.tilt_sigma_rad),
            "gravity": float(s.gravity_mps2),
            "cov_diag": np.array(s.cov_diag, dtype=np.float32),
            "reacq": int(s.gnss_reacquire_reset_count),
            "vreanch": int(s.gnss_velocity_reanchor_count),
            "outlier": int(s.gnss_outlier_reject_count),
            "mag_fused": int(s.mag_fused_count),
            "mag_reject": int(s.mag_reject_count),
            "vel_subset": int(s.vel_subset_count),
            "resid_pos": float(s.last_gnss_resid_pos_ne_m),
            "resid_vel": float(s.last_gnss_resid_vel_ne_mps),
            "gate_pos": bool(s.last_gnss_gate_pos),
            "gate_vel": bool(s.last_gnss_gate_vel),
            "tsg": float(s.time_since_gnss_s),
        }

    def diag(self) -> dict:
        """返回固件侧诊断 JSON（含协方差对角与计数器）。"""
        self._check()
        buf = ctypes.create_string_buffer(4096)  # type: ignore[attr-defined]
        n = self._lib.fw_ekf_diag_json(self._h, buf, 4096)
        if n <= 0:
            raise RuntimeError("fw_ekf_diag_json 输出失败（缓冲不足）")
        return json.loads(buf.value.decode("utf-8"))
