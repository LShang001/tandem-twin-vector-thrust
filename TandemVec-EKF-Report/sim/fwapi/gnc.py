# -*- coding: utf-8 -*-
"""固件控制链直调（fwapi.Gnc）。

@file    gnc.py
@module  analysis/固件算法直调（控制链）
@brief   用 Python 驱动固件 `runGNCExecutive()`：模式机 + 制导 + 姿态/速率环 +
          分配 + 混控，全部走固件原码（共享库），不在仿真侧重写。
@details 每个 step 注入导航状态 + 遥控 + 目标，读回舵机 PWM/分配诊断。
          输入角速度必须是**已去偏**的实测角速度（库内 nav_ekf 不运行）。
@frame   NED / FRD；四元数 FRD←NED，欧拉角 theta=−asin(R13)
@units   SI；遥控 PWM 输入 µs；执行器输出 %；摆角 deg
@note    固件控制层是进程内单例（PID/滤波器为全局状态）：同一进程只创建一个 Gnc；
         需要多条独立轨迹时，用子进程隔离。
@see     TandemVec_FCS/test_host/fw_algo/fw_gnc_api.h
"""
from __future__ import annotations

import ctypes

from . import library as _lib_mod
from .library import (
    FEATURE_NAMES,
    GNC_ABI_VERSION,
    GNC_TRACK_ABI_VERSION,
    VEHICLE_NAMES,
    FwCoreCaps,
    FwGncInput,
    FwGncOutput,
    FwGncTrackInput,
    FwGncTrackOutput,
)


def _lib():
    lib = _lib_mod.load()
    if not hasattr(lib, "fw_gnc_create"):
        raise RuntimeError(
            "当前 fw_algo 库不含控制链 ABI；请重新运行 "
            "TandemVec_FCS/test_host/fw_algo/build_fw_algo.sh")
    return lib


def caps() -> dict:
    """返回当前库的能力描述（ABI 版本/机型/特性位/构建号）。"""
    lib = _lib()
    c = FwCoreCaps()
    c.struct_size = ctypes.sizeof(FwCoreCaps)
    lib.fw_core_caps(ctypes.byref(c))
    feats = [name for bit, name in FEATURE_NAMES.items() if c.feature_bits & bit]
    return {
        "core_abi": c.core_abi_version,
        "ekf_abi": c.ekf_abi_version,
        "ctrl_abi": c.ctrl_abi_version,
        "gnc_abi": c.gnc_abi_version,
        "nav_abi": c.nav_abi_version,
        "param_count": c.param_count,
        "vehicle": VEHICLE_NAMES.get(c.vehicle_profile, str(c.vehicle_profile)),
        "features": feats,
        "feature_bits": c.feature_bits,
        "build_id": c.build_id.decode("utf-8", "replace"),
    }


def param_names() -> list[str]:
    """全部参数名（顺序即参数 ID，与 GCS 0xE2 同源）。"""
    lib = _lib()
    return [lib.fw_param_name(i).decode() for i in range(lib.fw_param_count())]


def param_get(name: str) -> float:
    lib = _lib()
    pid = lib.fw_param_find(name.encode())
    if pid < 0:
        raise KeyError(f"未知参数: {name}")
    v = ctypes.c_float()
    if lib.fw_param_get(pid, ctypes.byref(v)) != 0:
        raise RuntimeError(f"读参数失败: {name}")
    return float(v.value)


def param_dump() -> dict[str, float]:
    """导出全部参数（名字→值）：实验开始时落盘，保证参数可复现。"""
    lib = _lib()
    out: dict[str, float] = {}
    for i in range(lib.fw_param_count()):
        name = lib.fw_param_name(i)
        v = ctypes.c_float()
        if name and lib.fw_param_get(i, ctypes.byref(v)) == 0:
            out[name.decode()] = float(v.value)
    return out


def param_set(name: str, value: float) -> None:
    lib = _lib()
    pid = lib.fw_param_find(name.encode())
    if pid < 0:
        raise KeyError(f"未知参数: {name}")
    if lib.fw_param_set(pid, float(value)) != 0:
        raise RuntimeError(f"写参数失败（NaN/Inf 或越界）: {name}={value}")


class Gnc:
    """固件控制链句柄（进程内单例；不要在同一进程建多个）。"""

    def __init__(self) -> None:
        self._lib = _lib()
        self._h = self._lib.fw_gnc_create()
        if not self._h:
            raise RuntimeError("fw_gnc_create 失败")
        self._in = FwGncInput()
        self._in.struct_size = ctypes.sizeof(FwGncInput)
        self._in.abi_version = GNC_ABI_VERSION
        self._out = FwGncOutput()
        self._out.struct_size = ctypes.sizeof(FwGncOutput)

    def close(self) -> None:
        if self._h:
            self._lib.fw_gnc_destroy(self._h)
            self._h = None

    def __del__(self):  # pragma: no cover - 解释器退出路径
        try:
            self.close()
        except Exception:
            pass

    def step(self, *, quat, omega, vel, pos, alt_up, rc, dt,
             link_up=True, failsafe_in_flight=False, target_alt=0.0,
             heading_hold_allowed=False, pos_hold_degraded=False,
             gnss_outage_age_s=0.0, apply_target_alt=True) -> dict:
        """推进一拍固件控制链。

        @param quat      (w,x,y,z) FRD←NED
        @param omega     已去偏机体系角速度 [rad/s]
        @param vel       NED 速度 [m/s]
        @param pos       相对原点 NED 位置 [m]
        @param alt_up    相对起飞点高度（向上为正）[m]
        @param rc        长度 16 的 PWM 序列（µs；未用通道填 988/1500）
        @param dt        步长 [s]
        @param target_alt 定高目标 [m]
        @return dict：执行器百分比、模式、分配诊断
        """
        o = self._in
        for i in range(4):
            o.quat_wxyz[i] = float(quat[i])
        for i in range(3):
            o.omega_body_radps[i] = float(omega[i])
            o.vel_ned_mps[i] = float(vel[i])
            o.pos_ned_m[i] = float(pos[i])
        o.alt_up_m = float(alt_up)
        rc_list = list(rc)
        if len(rc_list) < 16:
            rc_list += [988] * (16 - len(rc_list))
        for i in range(16):
            o.rc_pwm[i] = int(rc_list[i])
        o.link_up = 1 if link_up else 0
        o.failsafe_in_flight = 1 if failsafe_in_flight else 0
        o.target_alt_m = float(target_alt)
        o.dt_s = float(dt)
        o.heading_hold_allowed = 1 if heading_hold_allowed else 0
        o.pos_hold_degraded = 1 if pos_hold_degraded else 0
        o.gnss_outage_age_s = float(gnss_outage_age_s)
        o.apply_target_alt = 1 if apply_target_alt else 0

        rc_code = self._lib.fw_gnc_step(self._h, ctypes.byref(self._in),
                                        ctypes.byref(self._out))
        if rc_code != 0:
            raise RuntimeError(f"fw_gnc_step 失败：code={rc_code}")
        r = self._out
        return {
            "ch1_pct": r.ch1_pct, "ch2_pct": r.ch2_pct,
            "ch3_pct": r.ch3_pct, "ch4_pct": r.ch4_pct,
            "flight_mode": int(r.flight_mode),
            "unlocked": bool(r.unlocked),
            "tvc_upper_deg": r.tvc_upper_deg,
            "tvc_lower_deg": r.tvc_lower_deg,
            "omega_ref_dps": list(r.omega_ref_dps),
            "alpha_ref_dps2": list(r.alpha_ref_dps2),
            "M_cmd": list(r.M_cmd),
            "delta_f_deg": r.delta_f_deg,
            "delta_t_deg": r.delta_t_deg,
            "dw": r.dw,
            "mode_degraded_heading": bool(r.mode_degraded_heading),
            "mode_degraded_gnss": bool(r.mode_degraded_gnss),
            "failsafe": bool(r.failsafe_in_flight),
            "actuator_fault_bits": int(r.actuator_fault_bits),
            "target_alt_m": r.target_alt_m,
            "specific_force_up_mps2": r.specific_force_up_mps2,
            "thrust_cos_tilt": r.thrust_cos_tilt,
            "throttle_cmd_pct": r.throttle_cmd_pct,
            "alt_error_m": r.alt_error_m,
            "target_vz_mps": r.target_vz_mps,
            "z_up_m": r.z_up_m,
        }


class GncTrack:
    """固件姿态目标直驱（跳过遥控模式机，直接给 q_target + 油门）。

    与 `Gnc` 共用同一个固件单例；同一进程内不要同时用两条通道驱动同一条轨迹。
    """

    def __init__(self) -> None:
        self._lib = _lib()
        self._h = self._lib.fw_gnc_create()
        if not self._h:
            raise RuntimeError("fw_gnc_create 失败")
        self._in = FwGncTrackInput()
        self._in.struct_size = ctypes.sizeof(FwGncTrackInput)
        self._in.abi_version = GNC_TRACK_ABI_VERSION
        self._out = FwGncTrackOutput()
        self._out.struct_size = ctypes.sizeof(FwGncTrackOutput)

    def close(self) -> None:
        if self._h:
            self._lib.fw_gnc_destroy(self._h)
            self._h = None

    def __del__(self):  # pragma: no cover
        try:
            self.close()
        except Exception:
            pass

    def step(self, *, quat, omega, q_target, throttle_pct, dt,
             heading_hold_allowed=True) -> dict:
        """推进一拍固件姿态/速率/分配/混控。

        @param quat     当前姿态 (w,x,y,z) FRD←NED
        @param omega    原始已去偏机体系角速度 [rad/s]
        @param q_target 期望姿态 (w,x,y,z) FRD←NED
        @param throttle_pct 直通油门 [%]
        @param dt       步长 [s]
        """
        o = self._in
        for i in range(4):
            o.quat_wxyz[i] = float(quat[i])
            o.q_target_wxyz[i] = float(q_target[i])
        for i in range(3):
            o.omega_body_radps[i] = float(omega[i])
        o.throttle_pct = float(throttle_pct)
        o.dt_s = float(dt)
        o.heading_hold_allowed = 1 if heading_hold_allowed else 0
        rc = self._lib.fw_gnc_track_step(self._h, ctypes.byref(self._in),
                                         ctypes.byref(self._out))
        if rc != 0:
            raise RuntimeError(f"fw_gnc_track_step 失败：code={rc}")
        r = self._out
        return {
            "ch1_pct": r.ch1_pct, "ch2_pct": r.ch2_pct,
            "ch3_pct": r.ch3_pct, "ch4_pct": r.ch4_pct,
            "tvc_upper_deg": r.tvc_upper_deg,
            "tvc_lower_deg": r.tvc_lower_deg,
            "delta_f_deg": r.delta_f_deg,
            "delta_t_deg": r.delta_t_deg,
            "dw": r.dw,
            "omega_ref_dps": list(r.omega_ref_dps),
            "alpha_ref_dps2": list(r.alpha_ref_dps2),
            "M_cmd": list(r.M_cmd),
        }
