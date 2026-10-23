"""GCS 后端包标记。

@file    __init__.py
@module  GCS/包入口
@brief   标识 server 目录为 Python 包，供 `python -m uvicorn server.main:app` 导入
@details 空模块：不执行任何初始化副作用（导入 server.main 时才创建全局单例 g）。
@theory  N/A
@frame   N/A
@units   N/A
"""
