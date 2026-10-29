# -*- coding: utf-8 -*-
"""fwapi — 固件算法直调通道。

@file    __init__.py
@module  analysis/固件算法直调（包入口）
@brief   导出 fwapi.Ekf 与构建/路径辅助函数。

让仿真/调试脚本直接调用固件里的真实算法实现（当前为 15 态 ESKF），
不再另写一份 Python 版本。

@details 共享库由固件 Ekf15State 编译而来，本包只做加载与类型映射。
@frame   NED / FRD（透传固件约定）。
@units   SI。

快速开始：
    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path("TandemVec-EKF-Report/sim").resolve()))
    import fwapi

    ekf = fwapi.Ekf()
    ekf.initialize(accel=(0, 0, -9.79), gyro=(0, 0, 0), lla=(0.4921, 1.9700, 50.0))
    ekf.time_update_two_sample(dth1, dth2, dv1, dv2, 0.005)
    print(ekf.state()["lla"], ekf.state()["vel"])

模块：
    fwapi.Ekf       固件 EKF 句柄
    fwapi.build_info() / fwapi.library_path()  查看加载的库
"""
from .library import (  # noqa: F401
    ABI_VERSION,
    FwEkfInit,
    FwNavState,
    FwUpdateResult,
    build,
    build_info,
    library_path,
    load,
)
from .ekf import Ekf  # noqa: F401
from .control import (  # noqa: F401
    TandemVecAllocator,
    default_params,
    params_to_dict,
    set_params,
    thrust_alloc,
)
from .nav import Nav  # noqa: F401
from .gnc import (  # noqa: F401
    Gnc,
    GncTrack,
    caps,
    param_dump,
    param_get,
    param_names,
    param_set,
)

__all__ = [
    "ABI_VERSION",
    "Ekf",
    "Gnc",
    "GncTrack",
    "Nav",
    "caps",
    "param_dump",
    "param_get",
    "param_names",
    "param_set",
    "TandemVecAllocator",
    "default_params",
    "params_to_dict",
    "set_params",
    "thrust_alloc",
    "FwEkfInit",
    "FwNavState",
    "FwUpdateResult",
    "build",
    "build_info",
    "library_path",
    "load",
]
