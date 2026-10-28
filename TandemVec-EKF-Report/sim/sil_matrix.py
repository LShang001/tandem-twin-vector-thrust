# -*- coding: utf-8 -*-
"""sil_matrix.py — fw_sil 二进制的通用矩阵驱动（研究设施，不含任何算法）。

@file    sil_matrix.py
@module  analysis/仿真矩阵驱动
@brief   统一"启动 fw_sil → 绑定核 → 解析 SIL_REAL_JSON"的样板，避免每个
         研究脚本各写一份 subprocess/正则/并行代码。
@details 用法：
             from sil_matrix import run_matrix, save_jsonl
             rows = run_matrix(cases, workers=10, cores=[2,3,5,...])
         每个 case = {"exe":..., "args":[...], "seed":int}；结果 dict 含原始 JSON
         字段，另加 seed/args。
@note    只做进程驱动与结果解析；算法全部在固件 SIL 内，本文件不得加入滤波/
         控制/传播公式。
"""
from __future__ import annotations

import concurrent.futures as cf
import json
import os
import re
import subprocess

import psutil

JRE = re.compile(r"SIL_REAL_JSON (\{.*\})")


def exe_in_temp(name: str) -> str:
    """把 Git Bash 的 /tmp/<dir>/<exe> 换成 Windows 可执行绝对路径。"""
    import tempfile

    return os.path.join(tempfile.gettempdir(), name)


def run_case(exe: str, args, seed: int, core: int | None = None,
             timeout_s: float = 300.0) -> dict:
    """运行一个 SIL 案例并返回解析后的 JSON 记录。"""
    cmd = [exe, *args, "--seed", str(seed)]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    if core is not None:
        try:
            psutil.Process(proc.pid).cpu_affinity([core])
        except Exception:
            pass
    out = proc.communicate(timeout=timeout_s)[0]
    rec = None
    for line in out.splitlines():
        m = JRE.search(line)
        if m:
            rec = json.loads(m.group(1))
    if rec is None:
        tail = "\n".join(out.splitlines()[-12:])
        raise RuntimeError(f"SIL 未输出结果（seed={seed}, core={core}）：\n{tail}")
    rec["seed"] = seed
    rec["args"] = list(args)
    return rec


def run_matrix(cases, workers: int = 10, cores=None,
               timeout_s: float = 300.0) -> list[dict]:
    """并发执行案例；cores 按 worker 轮转绑定，保证同 seed 的多个臂同核。"""
    cores = list(cores or [])
    rows: list[dict] = []

    def _work(idx_case):
        idx, c = idx_case
        core = cores[idx % len(cores)] if cores else None
        return run_case(c["exe"], c["args"], int(c["seed"]), core, timeout_s)

    with cf.ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(_work, ic): ic for ic in enumerate(cases)}
        for fut in cf.as_completed(futures):
            rows.append(fut.result())
    return rows


def save_jsonl(path: str, rows: list[dict]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")


def md5_of(path: str) -> str:
    import hashlib

    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()
