# -*- coding: utf-8 -*-
"""python -m fwapi：打印库路径/构建信息并跑最小自检。

@file    __main__.py
@module  analysis/固件算法直调（命令行入口）
@brief   加载 fw_algo 共享库，打印构建信息，执行最小自检并返回退出码。
@details 自检内容：初始化 → 5 s 静态传播 → 一次 GNSS 更新，检查状态有限、
         GNSS 融合成功且 ABI 版本匹配。用于确认 Python 侧看到的就是固件实现。
@frame   NED / FRD（透传固件约定）。
@units   SI。
"""
from .library import build_info, library_path, selftest

print(f"lib : {library_path()}")
print(f"info: {build_info()}")
raise SystemExit(selftest())
