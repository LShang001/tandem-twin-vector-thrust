# -*- coding: utf-8 -*-
"""fwapi — 固件算法直调通道（Python 侧）。

@file    library.py
@module  analysis/固件算法直调（加载与构建）
@brief   定位/构建 fw_algo 共享库，声明 C ABI 的 ctypes 结构体与函数原型。
@details 共享库由 TandemVec_FCS/test_host/fw_algo/fw_algo_api.cpp 编译而来，
         源码就是固件 Ekf15State，本包不做任何算法复刻。
         默认输出到系统临时目录（ASCII 路径），避免 MinGW 在中文路径下链接失败。
@note    源码比库新时自动重建（可用 FW_ALGO_NO_BUILD=1 禁用、FW_ALGO_LIB 指定现成库）。
@see     TandemVec_FCS/test_host/fw_algo/fw_algo_api.h
"""
from __future__ import annotations

import ctypes
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ABI_VERSION = 2
STATE_DIM = 15


def repo_root() -> Path:
    """仓库根目录（含 TandemVec_FCS 与 TandemVec-EKF-Report）。"""
    return Path(__file__).resolve().parents[3]


def fcs_root() -> Path:
    return repo_root() / "TandemVec_FCS"


def _bash() -> str:
    """选择构建用 bash：Windows 上必须显式指向 Git Bash。

    裸 `bash` 在部分 Windows 环境会解析到 WSL 的 bash（路径/挂载不同，
    且不会继承 MinGW 工具链），表现为构建静默失败。可用 FW_BASH 覆盖。
    """
    env = os.environ.get("FW_BASH")
    if env:
        return env
    if sys.platform.startswith("win"):
        for cand in (r"C:\Program Files\Git\bin\bash.exe",
                     r"C:\Program Files (x86)\Git\bin\bash.exe"):
            if Path(cand).exists():
                return cand
    return "bash"


def build_dir() -> Path:
    """共享库输出目录：优先 BIN_DIR/FW_ALGO_BIN_DIR，否则系统临时目录。"""
    env = os.environ.get("FW_ALGO_BIN_DIR") or os.environ.get("BIN_DIR")
    d = Path(env) if env else Path(tempfile.gettempdir()) / "tv_fw_algo"
    d.mkdir(parents=True, exist_ok=True)
    return d


def library_name() -> str:
    return "fw_algo.dll" if sys.platform.startswith("win") else "libfw_algo.so"


def _sources() -> list[Path]:
    """参与共享库的源码（与 test_host/fw_algo/build_fw_algo.sh 的清单一致）。

    只用于信息展示/人工核对；增量判断由构建脚本按目标文件 mtime 完成。
    """
    root = fcs_root()
    core = [
        "test_host/fw_algo/core_sources.sh",
        "src/flight_control.cpp",
        "src/navigation_task.cpp",
        "src/state_data.cpp",
        "src/ano_params.cpp",
        "lib/ublox-main/src/ubx.cpp",
        "test_host/fw_sil/fw_sil_stubs.cpp",
        "lib/DPS310-Pressure-Sensor/src/Dps310.cpp",
        "lib/DPS310-Pressure-Sensor/src/DpsClass.cpp",
        "lib/units/src/convang.cpp",
        "lib/units/src/convangacc.cpp",
        "lib/units/src/convangvel.cpp",
        "lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp",
        "lib/ublox-main/src/minmea.c",
    ]
    api = [
        "test_host/fw_algo/fw_algo_api.cpp",
        "test_host/fw_algo/fw_ctrl_api.cpp",
        "test_host/fw_algo/fw_gnc_api.cpp",
        "test_host/fw_algo/fw_algo_api.h",
        "test_host/fw_algo/fw_gnc_api.h",
    ]
    return [root / p for p in core + api]


def build(force: bool = False) -> Path:
    """调用固件侧唯一构建入口（build_fw_algo.sh）编译共享库并返回路径。

    刻意不在 Python 里维护第二份编译参数：脚本负责源清单、宏、C/C++ 分流与
    静态链接；Python 只传输出目录与（可选）强制重建。这样"仿真用的库"与
    "SIL 用的库"永远来自同一份清单。
    """
    out = build_dir() / library_name()
    script = fcs_root() / "test_host" / "fw_algo" / "build_fw_algo.sh"
    env = dict(os.environ)
    env["BIN_DIR"] = str(build_dir())
    args = [_bash(), str(script)]
    if force:
        args.append("--clean")
    args.append("--print-path")
    try:
        proc = subprocess.run(args, check=True, capture_output=True, text=True, env=env)
    except FileNotFoundError as exc:  # pragma: no cover - 环境缺失
        raise RuntimeError("找不到 bash；请先安装 Git Bash/MinGW。") from exc
    except subprocess.CalledProcessError as exc:  # pragma: no cover
        raise RuntimeError(f"fw_algo 构建失败：\n{exc.stdout}\n{exc.stderr}") from exc
    path = Path(proc.stdout.strip().splitlines()[-1])
    if not path.exists():
        raise RuntimeError(f"构建脚本未产出共享库：{proc.stdout}\n{proc.stderr}")
    return path


# ---- ctypes 结构体（与 fw_algo_api.h 一一对应，改动需同步 ABI_VERSION） ----


class FwEkfInit(ctypes.Structure):
    _fields_ = [
        ("accel_mps2", ctypes.c_float * 3),
        ("gyro_radps", ctypes.c_float * 3),
        ("mag_ut", ctypes.c_float * 3),
        ("vel_ned_mps", ctypes.c_float * 3),
        ("lla_rad_m", ctypes.c_double * 3),
        ("yaw_offset_rad", ctypes.c_float),
    ]


class FwUpdateResult(ctypes.Structure):
    _fields_ = [
        ("input_valid", ctypes.c_int32),
        ("attempted", ctypes.c_int32),
        ("fused", ctypes.c_int32),
        ("reacquired", ctypes.c_int32),
        ("outlier_rejected", ctypes.c_int32),
        ("subset_position", ctypes.c_int32),
        ("subset_velocity", ctypes.c_int32),
        ("nis_pos3", ctypes.c_float),
        ("nis_vel3", ctypes.c_float),
        ("innovation_norm", ctypes.c_float),
        ("test_ratio", ctypes.c_float),
        ("yaw_correction_rad", ctypes.c_float),
        ("yaw_correction_clamped", ctypes.c_int32),
    ]


class FwNavState(ctypes.Structure):
    _fields_ = [
        ("lla_rad_m", ctypes.c_double * 3),
        ("vel_ned_mps", ctypes.c_float * 3),
        ("quat_wxyz", ctypes.c_float * 4),
        ("euler_ypr_rad", ctypes.c_float * 3),
        ("accel_bias_mps2", ctypes.c_float * 3),
        ("gyro_bias_radps", ctypes.c_float * 3),
        ("yaw_sigma_rad", ctypes.c_float),
        ("tilt_sigma_rad", ctypes.c_float),
        ("gravity_mps2", ctypes.c_float),
        ("cov_diag", ctypes.c_float * STATE_DIM),
        ("gnss_reacquire_reset_count", ctypes.c_uint32),
        ("gnss_velocity_reanchor_count", ctypes.c_uint32),
        ("gnss_outlier_reject_count", ctypes.c_uint32),
        ("mag_fused_count", ctypes.c_uint32),
        ("mag_reject_count", ctypes.c_uint32),
        ("vel_subset_count", ctypes.c_uint32),
        ("last_gnss_resid_pos_ne_m", ctypes.c_float),
        ("last_gnss_resid_vel_ne_mps", ctypes.c_float),
        ("last_gnss_gate_pos", ctypes.c_int32),
        ("last_gnss_gate_vel", ctypes.c_int32),
        ("time_since_gnss_s", ctypes.c_float),
    ]



class FwTvParams(ctypes.Structure):
    _fields_ = [
        ("kT", ctypes.c_float), ("kQ", ctypes.c_float), ("Jp", ctypes.c_float),
        ("wMax", ctypes.c_float), ("a", ctypes.c_float), ("b", ctypes.c_float),
        ("tauM", ctypes.c_float), ("Ix", ctypes.c_float), ("Iy", ctypes.c_float),
        ("Iz", ctypes.c_float), ("dMax", ctypes.c_float), ("dwMax", ctypes.c_float),
        ("m", ctypes.c_float), ("g", ctypes.c_float),
    ]


class FwPropState(ctypes.Structure):
    _fields_ = [("wf", ctypes.c_float), ("wt", ctypes.c_float),
                ("delta_f", ctypes.c_float), ("delta_t", ctypes.c_float)]


class FwTvAllocState(ctypes.Structure):
    _fields_ = [("prev", FwPropState), ("wf_est", ctypes.c_float),
                ("wt_est", ctypes.c_float)]


class FwTvAllocInput(ctypes.Structure):
    _fields_ = [
        ("alpha_roll_radps2", ctypes.c_float),
        ("alpha_pitch_radps2", ctypes.c_float),
        ("alpha_yaw_radps2", ctypes.c_float),
        ("throttle_pct", ctypes.c_float),
        ("gyro_body_radps", ctypes.c_float * 3),
        ("dt_s", ctypes.c_float),
        ("inertia_comp_mask", ctypes.c_int32),
    ]


class FwTvAllocOutput(ctypes.Structure):
    _fields_ = [
        ("valid", ctypes.c_int32),
        ("delta_f_rad", ctypes.c_float), ("delta_t_rad", ctypes.c_float),
        ("motor1_pct", ctypes.c_float), ("motor2_pct", ctypes.c_float),
        ("dw", ctypes.c_float),
        ("M_cmd", ctypes.c_float * 3), ("M_ff", ctypes.c_float * 3),
        ("w0_eff", ctypes.c_float), ("yaw_gain_sched", ctypes.c_float),
        ("sat_delta_f", ctypes.c_int32), ("sat_delta_t", ctypes.c_int32),
        ("sat_dw", ctypes.c_int32),
        ("next", FwTvAllocState),
    ]


class FwThrustAllocInput(ctypes.Structure):
    _fields_ = [
        ("specific_force_up_mps2", ctypes.c_float),
        ("thrust_axis_up_cos", ctypes.c_float),
        ("mass_kg", ctypes.c_float),
        ("max_total_thrust_n", ctypes.c_float),
        ("min_cos_tilt", ctypes.c_float),
    ]


class FwThrustAllocOutput(ctypes.Structure):
    _fields_ = [
        ("valid", ctypes.c_int32),
        ("total_thrust_n", ctypes.c_float),
        ("throttle_pct", ctypes.c_float),
        ("saturated_high", ctypes.c_int32),
        ("saturated_low", ctypes.c_int32),
    ]


class FwGncTrackInput(ctypes.Structure):
    """与 fw_gnc_api.h::fw_gnc_track_input_t 一一对应。"""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("quat_wxyz", ctypes.c_float * 4),
        ("omega_body_radps", ctypes.c_float * 3),
        ("q_target_wxyz", ctypes.c_float * 4),
        ("throttle_pct", ctypes.c_float),
        ("dt_s", ctypes.c_float),
        ("heading_hold_allowed", ctypes.c_int32),
    ]


class FwGncTrackOutput(ctypes.Structure):
    """与 fw_gnc_api.h::fw_gnc_track_output_t 一一对应。"""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("ch1_pct", ctypes.c_float),
        ("ch2_pct", ctypes.c_float),
        ("ch3_pct", ctypes.c_float),
        ("ch4_pct", ctypes.c_float),
        ("tvc_upper_deg", ctypes.c_float),
        ("tvc_lower_deg", ctypes.c_float),
        ("delta_f_deg", ctypes.c_float),
        ("delta_t_deg", ctypes.c_float),
        ("dw", ctypes.c_float),
        ("omega_ref_dps", ctypes.c_float * 3),
        ("alpha_ref_dps2", ctypes.c_float * 3),
        ("M_cmd", ctypes.c_float * 3),
    ]


class FwCoreCaps(ctypes.Structure):
    """与 fw_gnc_api.h::fw_core_caps_t 一一对应。"""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("core_abi_version", ctypes.c_uint32),
        ("ekf_abi_version", ctypes.c_uint32),
        ("ctrl_abi_version", ctypes.c_uint32),
        ("gnc_abi_version", ctypes.c_uint32),
        ("param_count", ctypes.c_uint32),
        ("vehicle_profile", ctypes.c_uint32),
        ("feature_bits", ctypes.c_uint32),
        ("build_id", ctypes.c_char * 64),
        ("nav_abi_version", ctypes.c_uint32),
    ]


class FwGncInput(ctypes.Structure):
    """与 fw_gnc_api.h::fw_gnc_input_t 一一对应（struct_size/abi_version 必须预填）。"""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("quat_wxyz", ctypes.c_float * 4),
        ("omega_body_radps", ctypes.c_float * 3),
        ("vel_ned_mps", ctypes.c_float * 3),
        ("pos_ned_m", ctypes.c_float * 3),
        ("alt_up_m", ctypes.c_float),
        ("rc_pwm", ctypes.c_uint16 * 16),
        ("link_up", ctypes.c_int32),
        ("failsafe_in_flight", ctypes.c_int32),
        ("target_alt_m", ctypes.c_float),
        ("dt_s", ctypes.c_float),
        ("heading_hold_allowed", ctypes.c_int32),
        ("pos_hold_degraded", ctypes.c_int32),
        ("gnss_outage_age_s", ctypes.c_float),
        ("apply_target_alt", ctypes.c_int32),
    ]


class FwGncOutput(ctypes.Structure):
    """与 fw_gnc_api.h::fw_gnc_output_t 一一对应。"""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("ch1_pct", ctypes.c_float),
        ("ch2_pct", ctypes.c_float),
        ("ch3_pct", ctypes.c_float),
        ("ch4_pct", ctypes.c_float),
        ("flight_mode", ctypes.c_int32),
        ("unlocked", ctypes.c_int32),
        ("tvc_upper_deg", ctypes.c_float),
        ("tvc_lower_deg", ctypes.c_float),
        ("omega_ref_dps", ctypes.c_float * 3),
        ("alpha_ref_dps2", ctypes.c_float * 3),
        ("M_cmd", ctypes.c_float * 3),
        ("delta_f_deg", ctypes.c_float),
        ("delta_t_deg", ctypes.c_float),
        ("dw", ctypes.c_float),
        ("mode_degraded_heading", ctypes.c_int32),
        ("mode_degraded_gnss", ctypes.c_int32),
        ("failsafe_in_flight", ctypes.c_int32),
        ("actuator_fault_bits", ctypes.c_uint32),
        ("target_alt_m", ctypes.c_float),
        ("specific_force_up_mps2", ctypes.c_float),
        ("thrust_cos_tilt", ctypes.c_float),
        ("throttle_cmd_pct", ctypes.c_float),
        ("alt_error_m", ctypes.c_float),
        ("target_vz_mps", ctypes.c_float),
        ("z_up_m", ctypes.c_float),
    ]


GNC_ABI_VERSION = 1
GNC_TRACK_ABI_VERSION = 1
NAV_ABI_VERSION = 1
NAV_MAX_DELTA_SAMPLES = 16
CORE_ABI_VERSION = 1

class FwNavInput(ctypes.Structure):
    """与 fw_nav_api.h::fw_nav_input_t 一一对应（struct_size/abi_version 必须预填）。"""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("dt_s", ctypes.c_float),
        ("accel_sum_body_mps2", ctypes.c_float * 3),
        ("gyro_sum_body_radps", ctypes.c_float * 3),
        ("sample_count", ctypes.c_int32),
        ("delta_count", ctypes.c_int32),
        ("delta_theta_rad", (ctypes.c_float * 3) * NAV_MAX_DELTA_SAMPLES),
        ("delta_vel_mps", (ctypes.c_float * 3) * NAV_MAX_DELTA_SAMPLES),
        ("delta_dt_s", ctypes.c_float * NAV_MAX_DELTA_SAMPLES),
        ("ahrs_quat_wxyz", ctypes.c_float * 4),
        ("ahrs_valid", ctypes.c_int32),
        ("gnss_valid", ctypes.c_int32),
        ("gnss_lat_1e7", ctypes.c_int32),
        ("gnss_lon_1e7", ctypes.c_int32),
        ("gnss_alt_mm", ctypes.c_int32),
        ("gnss_vel_ned_mmps", ctypes.c_int32 * 3),
        ("gnss_h_acc_mm", ctypes.c_uint32),
        ("gnss_v_acc_mm", ctypes.c_uint32),
        ("gnss_s_acc_mmps", ctypes.c_uint32),
        ("gnss_itow_ms", ctypes.c_uint32),
        ("baro_alt_up_m", ctypes.c_float),
        ("baro_valid", ctypes.c_int32),
        ("mag_body_ut", ctypes.c_float * 3),
        ("mag_valid", ctypes.c_int32),
        ("flow_vx_mps", ctypes.c_float),
        ("flow_vy_mps", ctypes.c_float),
        ("flow_range_m", ctypes.c_float),
        ("flow_quality", ctypes.c_int32),
        ("range_quality", ctypes.c_int32),
        ("flow_valid", ctypes.c_int32),
        ("unlocked", ctypes.c_int32),
        ("throttle_pct", ctypes.c_float),
    ]


class FwNavOutput(ctypes.Structure):
    """与 fw_nav_api.h::fw_nav_output_t 一一对应。"""

    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("quat_wxyz", ctypes.c_float * 4),
        ("euler_rpy_deg", ctypes.c_float * 3),
        ("lla_deg", ctypes.c_double * 3),
        ("pos_ned_m", ctypes.c_float * 3),
        ("vel_ned_mps", ctypes.c_float * 3),
        ("alt_up_m", ctypes.c_float),
        ("accel_bias_mps2", ctypes.c_float * 3),
        ("gyro_bias_radps", ctypes.c_float * 3),
        ("yaw_sigma_deg", ctypes.c_float),
        ("tilt_sigma_deg", ctypes.c_float),
        ("initialized", ctypes.c_int32),
        ("gnss_fix3d", ctypes.c_int32),
        ("gnss_epoch_count", ctypes.c_uint32),
        ("gnss_fix3d_count", ctypes.c_uint32),
        ("gnss_quality_fail", ctypes.c_uint32),
        ("gnss_duplicate", ctypes.c_uint32),
        ("gnss_age_s", ctypes.c_float),
        ("baro_fused", ctypes.c_int32),
        ("baro_innov_m", ctypes.c_float),
        ("heading_aiding", ctypes.c_int32),
        ("pos_degraded", ctypes.c_int32),
        ("imu_health", ctypes.c_int32),
        ("imu_health_flags", ctypes.c_uint32),
        ("gnss_queued", ctypes.c_uint32),
        ("gnss_pvt", ctypes.c_uint32),
        ("gnss_eoe", ctypes.c_uint32),
        ("gnss_overflow", ctypes.c_uint32),
        ("last_epoch_tow_ms", ctypes.c_uint32),
        ("steps", ctypes.c_uint32),
    ]


VEHICLE_NAMES = {0: "tandemvec", 1: "coaxial_vtvl", 2: "liquid_rocket"}
FEATURE_NAMES = {
    1 << 0: "raw_imu_increment",
    1 << 1: "gnss_outage_vel_hold",
    1 << 2: "gnss_outage_alt_hold",
    1 << 3: "long_outage_descent",
    1 << 4: "baro_health_gate",
    1 << 5: "gnss_vertical_gate",
    1 << 6: "gnc_available",
    1 << 7: "nav_available",
}

_LIB = None
_LIB_PATH: Path | None = None


def load(force_build: bool = False) -> ctypes.CDLL:
    """加载（必要时构建）共享库，并绑定函数原型。"""
    global _LIB, _LIB_PATH
    if _LIB is not None and not force_build:
        return _LIB

    env_lib = os.environ.get("FW_ALGO_LIB")
    if env_lib:
        path = Path(env_lib)
        if not path.exists():
            raise FileNotFoundError(f"FW_ALGO_LIB 指向的文件不存在：{path}")
    elif os.environ.get("FW_ALGO_NO_BUILD") == "1":
        path = build_dir() / library_name()
        if not path.exists():
            raise FileNotFoundError(f"未找到共享库：{path}（FW_ALGO_NO_BUILD=1 禁止构建）")
    else:
        path = build(force=force_build)

    lib = ctypes.CDLL(str(path))
    p = ctypes
    lib.fw_algo_abi_version.restype = p.c_int32
    lib.fw_algo_build_info.restype = p.c_char_p
    lib.fw_ekf_create.restype = p.c_void_p
    lib.fw_ekf_destroy.argtypes = [p.c_void_p]
    lib.fw_ekf_init.argtypes = [p.c_void_p, p.POINTER(FwEkfInit)]
    lib.fw_ekf_init.restype = p.c_int32

    f3 = p.POINTER(p.c_float)
    d3 = p.POINTER(p.c_double)
    lib.fw_ekf_time_update.argtypes = [p.c_void_p, f3, f3, p.c_float]
    lib.fw_ekf_time_update_two_sample.argtypes = [
        p.c_void_p, f3, f3, f3, f3, p.c_float]
    lib.fw_ekf_gnss_update.argtypes = [
        p.c_void_p, f3, d3, p.c_float, p.c_float, p.c_int32]
    lib.fw_ekf_gnss_update.restype = FwUpdateResult
    lib.fw_ekf_mag_update.argtypes = [p.c_void_p, f3, f3, p.c_float]
    lib.fw_ekf_mag_update.restype = FwUpdateResult
    lib.fw_ekf_baro_update.argtypes = [p.c_void_p, p.c_float, p.c_float]
    lib.fw_ekf_baro_update.restype = FwUpdateResult
    lib.fw_ekf_aiding_velocity_update.argtypes = [
        p.c_void_p, f3, p.c_float, p.c_float]
    lib.fw_ekf_aiding_velocity_update.restype = FwUpdateResult
    lib.fw_ekf_gravity_update.argtypes = [p.c_void_p, f3, p.c_float]
    lib.fw_ekf_gravity_update.restype = FwUpdateResult
    lib.fw_ekf_static_gyro_update.argtypes = [p.c_void_p, f3, p.c_float]
    lib.fw_ekf_static_gyro_update.restype = FwUpdateResult
    lib.fw_ekf_yaw_update.argtypes = [p.c_void_p, p.c_float, p.c_float]
    lib.fw_ekf_yaw_update.restype = FwUpdateResult
    lib.fw_ekf_reset_position_velocity.argtypes = [p.c_void_p, f3, d3]
    lib.fw_ekf_reset_velocity.argtypes = [p.c_void_p, f3, p.c_float]
    lib.fw_ekf_set_gnss_gates.argtypes = [
        p.c_void_p, p.c_int32, p.c_int32, p.c_int32]
    lib.fw_ekf_set_velocity_subset.argtypes = [
        p.c_void_p, p.c_int32, p.c_float, p.c_int32]
    lib.fw_ekf_set_imu_limits.argtypes = [p.c_void_p, p.c_float, p.c_float]
    lib.fw_ekf_set_q_accel_scale.argtypes = [p.c_void_p, p.c_float]
    lib.fw_ekf_set_lever_arm.argtypes = [p.c_void_p, f3]
    lib.fw_ekf_get_state.argtypes = [p.c_void_p, p.POINTER(FwNavState)]
    lib.fw_ekf_diag_json.argtypes = [p.c_void_p, p.c_char_p, p.c_int32]
    lib.fw_ekf_diag_json.restype = p.c_int32

    lib.fw_ctrl_default_params.argtypes = [p.POINTER(FwTvParams)]
    lib.fw_ctrl_state_hover.argtypes = [p.POINTER(FwTvParams), p.POINTER(FwTvAllocState)]
    lib.fw_tandemvec_alloc.argtypes = [p.POINTER(FwTvParams), p.POINTER(FwTvAllocState),
                                       p.POINTER(FwTvAllocInput)]
    lib.fw_tandemvec_alloc.restype = FwTvAllocOutput
    lib.fw_thrust_alloc.argtypes = [p.POINTER(FwThrustAllocInput)]
    lib.fw_thrust_alloc.restype = FwThrustAllocOutput

    # 控制链 / 能力 / 参数（fw_gnc_api.h）。旧库缺这些符号时降级，
    # 不阻塞只用 EKF/分配的脚本（向后兼容的读取侧）。
    try:
        lib.fw_core_caps.argtypes = [p.POINTER(FwCoreCaps)]
        lib.fw_core_caps.restype = None
        lib.fw_gnc_create.restype = p.c_void_p
        lib.fw_gnc_destroy.argtypes = [p.c_void_p]
        lib.fw_gnc_destroy.restype = None
        lib.fw_gnc_step.argtypes = [p.c_void_p, p.POINTER(FwGncInput),
                                    p.POINTER(FwGncOutput)]
        lib.fw_gnc_step.restype = p.c_int32
        lib.fw_gnc_track_step.argtypes = [p.c_void_p, p.POINTER(FwGncTrackInput),
                                          p.POINTER(FwGncTrackOutput)]
        lib.fw_gnc_track_step.restype = p.c_int32

        nav_handle = p.POINTER(p.c_void_p)
        lib.fw_nav_create.argtypes = [nav_handle]
        lib.fw_nav_create.restype = p.c_int32
        lib.fw_nav_destroy.argtypes = [p.c_void_p]
        lib.fw_nav_reset.argtypes = [p.c_void_p]
        lib.fw_nav_reset.restype = p.c_int32
        lib.fw_nav_step.argtypes = [p.c_void_p, p.POINTER(FwNavInput),
                                    p.POINTER(FwNavOutput)]
        lib.fw_nav_step.restype = p.c_int32
        lib.fw_param_count.restype = p.c_uint32
        lib.fw_param_name.argtypes = [p.c_uint32]
        lib.fw_param_name.restype = p.c_char_p
        lib.fw_param_find.argtypes = [p.c_char_p]
        lib.fw_param_find.restype = p.c_int32
        lib.fw_param_get.argtypes = [p.c_uint32, p.POINTER(p.c_float)]
        lib.fw_param_get.restype = p.c_int32
        lib.fw_param_set.argtypes = [p.c_uint32, p.c_float]
        lib.fw_param_set.restype = p.c_int32
    except AttributeError:  # pragma: no cover - 旧库
        pass

    ver = lib.fw_algo_abi_version()
    if ver != ABI_VERSION:
        raise RuntimeError(f"fw_algo ABI 版本不匹配：库={ver}，Python={ABI_VERSION}")
    _LIB = lib
    _LIB_PATH = path
    return lib


def library_path() -> Path:
    load()
    assert _LIB_PATH is not None
    return _LIB_PATH


def build_info() -> str:
    lib = load()
    return lib.fw_algo_build_info().decode("utf-8", "replace")


def selftest() -> int:
    """最小自检：5 s 静态传播 + 一次 GNSS 更新，检查状态有限且 ABI 匹配。"""
    import numpy as np

    lla = np.array([28.2 * np.pi / 180.0, 112.9 * np.pi / 180.0, 50.0])
    from .ekf import Ekf

    with Ekf() as ekf:
        ekf.initialize((0.0, 0.0, -9.79), (0.0, 0.0, 0.0), lla=lla)
        for _ in range(1000):
            ekf.time_update((0.0, 0.0, -9.79), (0.0, 0.0, 0.0), 0.005)
        r = ekf.gnss_update((0.0, 0.0, 0.0), lla, 0.005, 0.0, False)
        st = ekf.state()
        ok = (r["fused"] and np.all(np.isfinite(st["lla"]))
              and np.all(np.isfinite(st["vel"])) and ekf.diag()["abi"] == ABI_VERSION)

    # 控制/分配层（ABI v2）
    from .control import TandemVecAllocator, thrust_alloc

    alloc = TandemVecAllocator()
    a0 = alloc.step(0.0, 0.0, 0.0, throttle_pct=50.0, dt=0.005)
    thr = thrust_alloc(9.79, 1.0, 0.7, 13.73)
    ctrl_ok = (a0["valid"] and abs(a0["motor1_pct"] - 50.0) < 0.1
               and thr["valid"] and abs(thr["throttle_pct"] - 49.9126) < 0.2)
    ok = ok and ctrl_ok
    print("selftest:", "PASS" if ok else "FAIL", "| fused:", r["fused"],
          "| ctrl:", ctrl_ok)
    return 0 if ok else 1


if __name__ == "__main__":  # pragma: no cover
    print(f"lib : {library_path()}")
    print(f"info: {build_info()}")
    if "--selftest" in sys.argv:
        sys.exit(selftest())
