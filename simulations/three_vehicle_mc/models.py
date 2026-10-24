# -*- coding: utf-8 -*-
"""models.py — 三机型“可飞行参数 → 等效系统模型”反演表。

@file    models.py
@module  仿真/三机型快速 MC（等效模型反演）
@brief   汇总 TandemVec/共轴/液体三型可飞行配置，给出快速 MC 使用的等效模型、
         参数来源、置信度与不可辨识项
@details 直接来源：models/aircraft-model.json、TandemVec_Config.h、
         FlightCtrlParams.h、VTVL_ElectricDualRotor_FCS 的 VtvLParams/state_data/
         flight_control、VTVL_LiquidRocket_FC 的 CCAEGT_Config/state_data。
         共轴/液体的惯量与力臂没有实飞辨识值，本文件用控制权限/时间尺度反演
         给出 L1 假设，不能当实机参数。
@theory  TandemVec: T=kT·ω²、M=I·α；共轴：M=L·T·δ、yaw=k·Δ；液体：T=m_dot·g0·Isp
@frame   机体系悬停存档系；快速 MC 只用角度/角速率的小角度等效
@units   kg、kg·m²、N、N·m、m、s、deg
@note    ★ 这张表是“反演假设 + 实飞配置”的混合体：
         - TandemVec 参数来自实飞配置/组件估算，置信度中；
         - 共轴 mass/Tmax/增益来自实飞配置，惯量/力臂/差速力矩是反演假设；
         - 液体 mass/Tmax/Tmin/Isp 来自 CCAEGT/实飞配置，惯量/力臂是假设；
         - 没有原始飞行日志，不能据此证明实机模型唯一。
@test    run_mega.py（只验证快速 MC 自身可运行/可复现，不验证实机）
@see     ../../TandemVec-EKF-Report/sim/hifi/hifi_core.py、VER-002
"""
from dataclasses import dataclass, asdict


@dataclass(frozen=True)
class VehicleModel:
    name: str
    mass_kg: float
    ix: float
    iy: float
    iz: float
    thrust_max_n: float
    thrust_min_n: float
    thrust_curve: str          # "quadratic_throttle" | "linear_throttle"
    tau_motor_s: float
    lever_m: float
    gimbal_max_deg: float
    att_kp: float              # 外环 1/s
    rate_kp: float             # 内环 1/s（角度域等效）
    rate_max_dps: float
    alpha_max_radps2: float
    yaw_alpha_max_radps2: float
    vertical_kp: float         # 高度 P
    vertical_kd: float         # 垂直速度 D
    source: str
    confidence: str
    unidentifiable: str


# TandemVec：实飞配置/组件估算（与 hifi_core.py、aircraft-model.json 同源）
TANDEMVEC = VehicleModel(
    name="TandemVec",
    mass_kg=0.70, ix=0.0021, iy=0.022, iz=0.022,
    thrust_max_n=2.0 * 1.04e-5 * 1150.0 ** 2, thrust_min_n=0.0,
    thrust_curve="quadratic_throttle", tau_motor_s=0.28,
    lever_m=0.315, gimbal_max_deg=15.0,
    att_kp=2.8, rate_kp=16.042818, rate_max_dps=600.0,
    alpha_max_radps2=100.0, yaw_alpha_max_radps2=100.0,
    vertical_kp=1.0, vertical_kd=5.0,
    source="models/aircraft-model.json; TandemVec_Config.h; FlightCtrlParams.h",
    confidence="mass/inertia/thrust ESTIMATED；TandemVec 已实飞，参数为实飞配置/组件估算",
    unidentifiable="Iy/Iz 组件估算、Jp/气动阻尼；未做实机频响辨识",
)

# 共轴：mass/Tmax/增益来自实飞工程；惯量/力臂由 rate-loop 时间尺度反演假设
# rate_kp_eff = Kp_rate(0.25) * L*T/I ≈ 0.25*0.15*14/0.05 = 10.5 1/s
COAXIAL = VehicleModel(
    name="CoaxialVTVL",
    mass_kg=0.84, ix=0.05, iy=0.05, iz=0.08,
    thrust_max_n=14.0, thrust_min_n=0.0,
    thrust_curve="linear_throttle", tau_motor_s=0.28,
    lever_m=0.15, gimbal_max_deg=15.0,
    att_kp=4.25, rate_kp=10.5, rate_max_dps=80.0,
    alpha_max_radps2=7.33, yaw_alpha_max_radps2=5.0,
    vertical_kp=1.2, vertical_kd=5.0,
    source="VTVL_ElectricDualRotor_FCS: VtvLParams.h/state_data.cpp/flight_control.cpp",
    confidence="mass 0.84/Tmax 14/增益/filter 来自实飞配置；I/lever 为低置信反演",
    unidentifiable="Ix/Iy/Iz、lever、差速力矩系数、TVC 机械方向无实飞辨识；"
                   "rate_kp_eff 由 assumed 0.12 s rate-loop 时间尺度反演",
)

# 液体：mass/Tmax/Tmin/Isp 来自 CCAEGT/实飞配置；I/lever 为细长体假设
LIQUID = VehicleModel(
    name="LiquidRocket",
    mass_kg=16.5, ix=0.35, iy=3.0, iz=3.0,
    thrust_max_n=200.0, thrust_min_n=80.0,
    thrust_curve="linear_throttle", tau_motor_s=0.10,
    lever_m=0.45, gimbal_max_deg=10.0,
    att_kp=3.0, rate_kp=8.0, rate_max_dps=60.0,
    alpha_max_radps2=10.0, yaw_alpha_max_radps2=0.0,
    vertical_kp=1.0, vertical_kd=5.0,
    source="VTVL_LiquidRocket_FC: CCAEGT_Config.h/state_data.cpp; TandemVec_FCS/LiquidRocketModel.h",
    confidence="M0/Tmax/Tmin/Isp 来自 CCAEGT 示例；I/lever/姿态增益为低置信假设",
    unidentifiable="I/lever/气动阻尼/RCS；CCAEGT M_DRY=12.5 与固件 DRY_MASS=13.5 不一致",
)

MODELS = {m.name: m for m in (TANDEMVEC, COAXIAL, LIQUID)}


def as_dict():
    return {name: asdict(m) for name, m in MODELS.items()}


if __name__ == "__main__":
    import json
    print(json.dumps(as_dict(), ensure_ascii=False, indent=2))
