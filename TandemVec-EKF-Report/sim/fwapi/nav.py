"""
@file    fwapi/nav.py
@brief   固件导航步进（fw_nav_step）的 Python 封装
@module  analysis/固件算法直调
@details 把导航任务每拍消费的外部量灌进固件 `handleNavigationSystem()`，读回 EKF 状态；不做算法复刻。
@frame   NED / FRD（透传固件约定）
@units   SI；LLA 输入按 UBX 线格式 1e-7 deg / mm


直接驱动固件 `handleNavigationSystem()`：输入 = 导航任务每拍消费的外部量
（聚合 IMU / AHRS / GNSS 历元 / 气压 / 磁 / 光流 / 解锁油门），输出 = 15 态 EKF
状态与门控。**不做任何导航算法复刻**；传感器层（2 kHz 低通 + Madgwick）由调用方
负责，见 `fw_nav_api.h` 的边界说明。

@file    fwapi/nav.py
@see     test_host/fw_algo/fw_nav_api.h、sim/run_fw_nav_demo.py
"""
from __future__ import annotations

import ctypes

import numpy as np

from . import library


class Nav:
    """固件导航单例（进程内一个实例，与固件全局 EKF 对应）。"""

    def __init__(self):
        self._lib = library.load()
        handle = ctypes.c_void_p()
        rc = self._lib.fw_nav_create(ctypes.byref(handle))
        if rc != 0:
            raise RuntimeError(f"fw_nav_create 失败 rc={rc}")
        self._handle = handle

    def close(self) -> None:
        if getattr(self, "_handle", None):
            self._lib.fw_nav_destroy(self._handle)
            self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def reset(self) -> None:
        rc = self._lib.fw_nav_reset(self._handle)
        if rc != 0:
            raise RuntimeError(f"fw_nav_reset 失败 rc={rc}")

    def step(self, dt, accel, gyro, ahrs_quat=None, sample_count=None,
             delta_theta=None, delta_vel=None, delta_dt=None,
             gnss=None, baro_alt_up=None, mag=None, flow=None,
             unlocked=True, throttle_pct=0.0, sums=False):
        """推进一拍。

        @param dt        拍长 [s]（0.005）
        @param accel     比力输入 [m/s²]（FRD，已滤波）。`sums=False`（默认）时
                         按均值处理并乘 `sample_count`（缺省 1）；`sums=True` 时
                         原样作为本拍的和传入（与 SIL 逐位对拍用）。
        @param gyro      角速率输入 [rad/s]（FRD，含零偏），语义同 accel
        @param ahrs_quat (w,x,y,z) 或 None（None ⇒ 本拍不更新 AHRS 量测）
        @param gnss      dict(lat_deg, lon_deg, alt_m, vel_ned, h_acc, v_acc,
                              s_acc, itow_ms) 或 None
        @param baro_alt_up / mag (3,) / flow dict(vx, vy, range_m, quality,
                              range_quality) 或 None
        @return dict（见 FwNavOutput 字段）
        """
        inp = library.FwNavInput()
        inp.struct_size = ctypes.sizeof(library.FwNavInput)
        inp.abi_version = library.NAV_ABI_VERSION
        inp.dt_s = float(dt)
        count = int(sample_count) if sample_count is not None else 1
        if count < 0:
            count = 0
        inp.sample_count = count
        scale = 1.0 if sums else float(count)
        for i in range(3):
            inp.accel_sum_body_mps2[i] = float(accel[i]) * scale
            inp.gyro_sum_body_radps[i] = float(gyro[i]) * scale
        n = 0
        if delta_theta is not None:
            n = min(len(delta_theta), library.NAV_MAX_DELTA_SAMPLES)
            for i in range(n):
                for j in range(3):
                    inp.delta_theta_rad[i][j] = float(delta_theta[i][j])
                    inp.delta_vel_mps[i][j] = float(delta_vel[i][j])
                inp.delta_dt_s[i] = float(delta_dt[i])
        inp.delta_count = n

        if ahrs_quat is not None:
            for i in range(4):
                inp.ahrs_quat_wxyz[i] = float(ahrs_quat[i])
            inp.ahrs_valid = 1

        if gnss is not None:
            # 与 UBX 线格式一致：1e-7 deg / mm / mm·s⁻¹（float 在纬度 28° 只有
            # ~0.2 m、经度 112° ~0.75 m 分辨率，会在量测入口丢精度）。
            inp.gnss_valid = 1
            inp.gnss_lat_1e7 = int(round(float(gnss["lat_deg"]) * 1e7))
            inp.gnss_lon_1e7 = int(round(float(gnss["lon_deg"]) * 1e7))
            inp.gnss_alt_mm = int(round(float(gnss["alt_m"]) * 1000.0))
            for i in range(3):
                inp.gnss_vel_ned_mmps[i] = int(round(float(gnss["vel_ned"][i]) * 1000.0))
            inp.gnss_h_acc_mm = int(round(float(gnss.get("h_acc", 0.5)) * 1000.0))
            inp.gnss_v_acc_mm = int(round(float(gnss.get("v_acc", 0.8)) * 1000.0))
            inp.gnss_s_acc_mmps = int(round(float(gnss.get("s_acc", 0.1)) * 1000.0))
            inp.gnss_itow_ms = int(gnss.get("itow_ms", 0))

        if baro_alt_up is not None:
            inp.baro_alt_up_m = float(baro_alt_up)
            inp.baro_valid = 1

        if mag is not None:
            for i in range(3):
                inp.mag_body_ut[i] = float(mag[i])
            inp.mag_valid = 1

        if flow is not None:
            inp.flow_valid = 1
            inp.flow_vx_mps = float(flow.get("vx", 0.0))
            inp.flow_vy_mps = float(flow.get("vy", 0.0))
            inp.flow_range_m = float(flow.get("range_m", 0.0))
            inp.flow_quality = int(flow.get("quality", 100))
            inp.range_quality = int(flow.get("range_quality", 100))

        inp.unlocked = 1 if unlocked else 0
        inp.throttle_pct = float(throttle_pct)

        out = library.FwNavOutput()
        out.struct_size = ctypes.sizeof(library.FwNavOutput)
        out.abi_version = library.NAV_ABI_VERSION
        rc = self._lib.fw_nav_step(self._handle, ctypes.byref(inp),
                                   ctypes.byref(out))
        if rc != 0:
            raise RuntimeError(f"fw_nav_step 失败 rc={rc}")
        return {
            "quat": np.array(out.quat_wxyz, dtype=float),
            "euler_rpy_deg": np.array(out.euler_rpy_deg, dtype=float),
            "lla_deg": np.array(out.lla_deg, dtype=float),
            "pos_ned": np.array(out.pos_ned_m, dtype=float),
            "vel_ned": np.array(out.vel_ned_mps, dtype=float),
            "alt_up_m": float(out.alt_up_m),
            "accel_bias": np.array(out.accel_bias_mps2, dtype=float),
            "gyro_bias": np.array(out.gyro_bias_radps, dtype=float),
            "yaw_sigma_deg": float(out.yaw_sigma_deg),
            "tilt_sigma_deg": float(out.tilt_sigma_deg),
            "initialized": int(out.initialized),
            "gnss_fix3d": int(out.gnss_fix3d),
            "gnss_epoch_count": int(out.gnss_epoch_count),
            "gnss_fix3d_count": int(out.gnss_fix3d_count),
            "gnss_quality_fail": int(out.gnss_quality_fail),
            "gnss_duplicate": int(out.gnss_duplicate),
            "gnss_age_s": float(out.gnss_age_s),
            "baro_fused": int(out.baro_fused),
            "baro_innov_m": float(out.baro_innov_m),
            "heading_aiding": int(out.heading_aiding),
            "pos_degraded": int(out.pos_degraded),
            "imu_health": int(out.imu_health),
            "imu_health_flags": int(out.imu_health_flags),
            "gnss_queued": int(out.gnss_queued),
            "gnss_pvt": int(out.gnss_pvt),
            "gnss_eoe": int(out.gnss_eoe),
            "gnss_overflow": int(out.gnss_overflow),
            "last_epoch_tow_ms": int(out.last_epoch_tow_ms),
            "steps": int(out.steps),
        }
