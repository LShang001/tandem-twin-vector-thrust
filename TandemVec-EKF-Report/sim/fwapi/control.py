# -*- coding: utf-8 -*-
"""fwapi.control — 固件控制/分配层的 Python 直调封装。

@file    control.py
@module  analysis/固件算法直调（控制分配）
@brief   直接调用固件 `allocateTandemVec` 与 `allocateLinearThrustThrottle`，
         不在 Python 里复刻 B 矩阵/差速开方/油门映射。
@details 典型用法：
             from fwapi import TandemVecAllocator, thrust_alloc
             alloc = TandemVecAllocator()          # 默认机型参数来自固件
             out = alloc.step(alpha_roll, alpha_pitch, alpha_yaw,
                              throttle_pct=50.0, gyro_body=omega, dt=0.005)
             thr = thrust_alloc(specific_force_up=9.79, cos_tilt=1.0,
                                mass=0.7, max_thrust=13.73)
@note    α/gyro 为**存档系**（x=滚转、y=俯仰、z=推力轴），与固件一致；
         摆角/电机输出与 SIL 的 ActuatorOutputMap 同源。
@frame   α/ω 存档系；推力分配为 NED 天向。
@units   α rad/s²、ω rad/s、油门 %、摆角 rad、推力 N。
"""
from __future__ import annotations

import ctypes

import numpy as np

from . import library


def default_params() -> "library.FwTvParams":
    """固件默认机型参数（TandemVec_Config.h::kDefaultTandemVecParams）。"""
    p = library.FwTvParams()
    library.load().fw_ctrl_default_params(ctypes.byref(p))
    return p


def params_to_dict(p: "library.FwTvParams") -> dict:
    """机型参数字段 → dict（便于打印/落盘）。"""
    return {name: float(getattr(p, name)) for name, _ in p._fields_}


def set_params(p: "library.FwTvParams", **kwargs) -> "library.FwTvParams":
    """按字段名覆盖参数（如 kT=1.1e-5、m=0.65）。"""
    names = {name for name, _ in p._fields_}
    for k, v in kwargs.items():
        if k not in names:
            raise KeyError(f"未知机型参数：{k}")
        setattr(p, k, float(v))
    return p


class TandemVecAllocator:
    """纵列双发机型分配器（α_cmd + 油门 → 摆角/电机/差速）。"""

    def __init__(self, params=None, hover: bool = True):
        self._lib = library.load()
        self.params = params if params is not None else default_params()
        self.state = library.FwTvAllocState()
        if hover:
            self._lib.fw_ctrl_state_hover(ctypes.byref(self.params),
                                          ctypes.byref(self.state))
        self._out = None

    def reset_hover(self) -> None:
        self._lib.fw_ctrl_state_hover(ctypes.byref(self.params),
                                      ctypes.byref(self.state))

    def step(self, alpha_roll: float, alpha_pitch: float, alpha_yaw: float,
             throttle_pct: float, gyro_body=(0.0, 0.0, 0.0),
             dt: float = 0.005, inertia_mask: int = 0x03) -> dict:
        inp = library.FwTvAllocInput()
        inp.alpha_roll_radps2 = float(alpha_roll)
        inp.alpha_pitch_radps2 = float(alpha_pitch)
        inp.alpha_yaw_radps2 = float(alpha_yaw)
        inp.throttle_pct = float(throttle_pct)
        g = np.asarray(gyro_body, dtype=np.float32).reshape(3)
        for i in range(3):
            inp.gyro_body_radps[i] = float(g[i])
        inp.dt_s = float(dt)
        inp.inertia_comp_mask = int(inertia_mask)
        r = self._lib.fw_tandemvec_alloc(ctypes.byref(self.params),
                                         ctypes.byref(self.state),
                                         ctypes.byref(inp))
        self.state = r.next
        self._out = r
        return {
            "valid": bool(r.valid),
            "delta_f_rad": float(r.delta_f_rad),
            "delta_t_rad": float(r.delta_t_rad),
            "motor1_pct": float(r.motor1_pct),
            "motor2_pct": float(r.motor2_pct),
            "dw": float(r.dw),
            "M_cmd": np.array(r.M_cmd, dtype=np.float32),
            "M_ff": np.array(r.M_ff, dtype=np.float32),
            "w0_eff": float(r.w0_eff),
            "yaw_gain_sched": float(r.yaw_gain_sched),
            "sat_delta_f": bool(r.sat_delta_f),
            "sat_delta_t": bool(r.sat_delta_t),
            "sat_dw": bool(r.sat_dw),
        }


def thrust_alloc(specific_force_up: float, cos_tilt: float = 1.0,
                 mass: float = 0.7, max_thrust: float = 13.73,
                 min_cos_tilt: float = 0.5) -> dict:
    """期望天向比力 → 总推力/线性油门（固件 allocateLinearThrustThrottle）。"""
    inp = library.FwThrustAllocInput()
    inp.specific_force_up_mps2 = float(specific_force_up)
    inp.thrust_axis_up_cos = float(cos_tilt)
    inp.mass_kg = float(mass)
    inp.max_total_thrust_n = float(max_thrust)
    inp.min_cos_tilt = float(min_cos_tilt)
    r = library.load().fw_thrust_alloc(ctypes.byref(inp))
    return {
        "valid": bool(r.valid),
        "total_thrust_n": float(r.total_thrust_n),
        "throttle_pct": float(r.throttle_pct),
        "saturated_high": bool(r.saturated_high),
        "saturated_low": bool(r.saturated_low),
    }
