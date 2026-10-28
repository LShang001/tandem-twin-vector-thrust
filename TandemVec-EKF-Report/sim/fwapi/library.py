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

ABI_VERSION = 1
STATE_DIM = 15


def repo_root() -> Path:
    """仓库根目录（含 TandemVec_FCS 与 TandemVec-EKF-Report）。"""
    return Path(__file__).resolve().parents[3]


def fcs_root() -> Path:
    return repo_root() / "TandemVec_FCS"


def build_dir() -> Path:
    """共享库输出目录：优先 BIN_DIR/FW_ALGO_BIN_DIR，否则系统临时目录。"""
    env = os.environ.get("FW_ALGO_BIN_DIR") or os.environ.get("BIN_DIR")
    d = Path(env) if env else Path(tempfile.gettempdir()) / "tv_fw_algo"
    d.mkdir(parents=True, exist_ok=True)
    return d


def library_name() -> str:
    return "fw_algo.dll" if sys.platform.startswith("win") else "libfw_algo.so"


def _sources() -> list[Path]:
    root = fcs_root()
    return [
        root / "test_host" / "fw_algo" / "fw_algo_api.cpp",
        root / "test_host" / "fw_algo" / "fw_algo_api.h",
        root / "lib" / "navigation-main" / "src" / "ekf_15_state.h",
        root / "lib" / "units" / "src" / "convang.cpp",
        root / "lib" / "units" / "src" / "convangacc.cpp",
        root / "lib" / "units" / "src" / "convangvel.cpp",
    ]


def build(force: bool = False) -> Path:
    """编译共享库并返回路径；源码未变且库存在时直接复用。"""
    out = build_dir() / library_name()
    srcs = _sources()
    if out.exists() and not force:
        newest = max(p.stat().st_mtime for p in srcs if p.exists())
        if out.stat().st_mtime >= newest:
            return out
    root = fcs_root()
    inc = [
        f"-I{root / 'include'}",
        f"-I{root / 'src'}",
        f"-I{root / 'lib' / 'eigen' / 'src'}",
        f"-I{root / 'lib' / 'navigation-main' / 'src'}",
        f"-I{root / 'lib' / 'units' / 'src'}",
        f"-I{root / 'lib' / 'ICM42688' / 'src'}",
        f"-I{root / 'test_host'}",
    ]
    flags = [
        "-std=c++17",
        "-O2",
        "-D_USE_MATH_DEFINES",
        "-DEKF_NAV_TELEMETRY",
        "-DEIGEN_DONT_VECTORIZE",
        "-DEIGEN_DONT_PARALLELIZE",
    ]
    cmd = [os.environ.get("CXX", "g++"), *flags, *inc, *map(str, srcs), "-shared"]
    if sys.platform.startswith("win"):
        # 自包含 MinGW 运行库：Python 进程的 PATH 上没有 libstdc++/libgcc。
        cmd += ["-static"]
    cmd += ["-o", str(out)]
    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True)
    except FileNotFoundError as exc:  # pragma: no cover - 环境缺失
        raise RuntimeError("找不到 g++；请先安装 MinGW 或设置 CXX。") from exc
    except subprocess.CalledProcessError as exc:  # pragma: no cover
        raise RuntimeError(f"fw_algo 构建失败：\n{exc.stderr}") from exc
    return out


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
    print("selftest:", "PASS" if ok else "FAIL", "| fused:", r["fused"])
    return 0 if ok else 1


if __name__ == "__main__":  # pragma: no cover
    print(f"lib : {library_path()}")
    print(f"info: {build_info()}")
    if "--selftest" in sys.argv:
        sys.exit(selftest())
