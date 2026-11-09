#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    calibration.py
@module  GCS/后端 传感器校准向导
@brief   把散在 DBG 命令里的磁罗盘校准流程，变成有步骤、有进度、有质量判据的向导
@details 为什么需要向导：固件的 `magcal` 命令本身是完整的（start/stop/fit/guided/
         save/load），但使用者要记住：
           ① 必须在地面未解锁状态操作；
           ② 自由采集要绕三轴缓慢旋转且样本 ≥30；
           ③ 采完要 fit，通过质量门才自动应用；
           ④ 应用后必须 save，否则重启回到旧参数；
           ⑤ 判"标得好不好"要看 residualRms / eigRatio / coverageRatio / fieldMag
             这几个数字，每个都有合理范围。
         这些知识不该要求使用者背下来——向导把它变成分步提示 + 自动判读。

         本模块只做两件事：**解析固件输出**、**判断质量**。命令发送与界面在
         main.py 与 web/src/calib.mjs。把解析独立出来是为了可单测——固件输出
         格式一变，测试立刻能发现，而不是等到现场标定失败才知道。

@theory  椭球拟合（硬铁偏移 + 软铁 3×3）+ 残差/特征值比/覆盖率质量判据
@frame   N/A
@units   磁场 uT、残差无量纲、比例无量纲
@note    质量阈值是工程经验值，不是唯一正解；判读时以 fieldMag 与本地地磁
         是否接近作为最强的交叉验证（长沙约 48–50 uT）。
@test    tests/test_calibration.py：逐行解析、缺字段、质量门各档判定
@see     TandemVec_FCS/src/communication.cpp `magCalPrintStats` / `debugMagCalCommand`
"""

from __future__ import annotations

import re
from typing import Any

# ---- 质量判据（工程经验值）----
MIN_SAMPLES = 30          # 固件自身要求
MAX_RESIDUAL_RMS = 0.05   # 椭球拟合残差，越小越好
EIG_RATIO_OK = (0.75, 1.35)   # 三个轴特征值比，接近 1 说明软铁畸变小
MIN_COVERAGE = 0.60       # 姿态覆盖率，太小说明只在一个方向采过

# 长沙地区地磁总场参考（IGRF 量级），用于交叉验证硬铁/缩放是否离谱
FIELD_MAG_REF_UT = 49.0
FIELD_MAG_TOL_UT = 8.0


def parse_magcal_line(line: str, state: dict[str, Any]) -> bool:
    """@brief 从一行 DBG 输出里提取磁标定字段，命中就更新 state
    @return 是否命中并更新了 state

    @note 固件的输出格式见 magCalPrintStats：
          `[DBG]   samples = 42`、`[DBG]   residualRms = 0.0123` 等。
          这里按 "键 = 值" 的通用形式解析，不强绑定某几行——固件加字段时
          解析不会崩，只是多一个键。
    """
    m = re.match(r'\s*\[DBG\]\s+([A-Za-z][A-Za-z0-9 _()]*?)\s*=\s*([-+0-9.eE]+)\s*(\w*)', line)
    if not m:
        return False
    key = m.group(1).strip()
    try:
        val = float(m.group(2))
    except ValueError:
        return False
    unit = m.group(3) or ''
    norm = {
        'samples': 'samples',
        'residualRms': 'residual_rms',
        'eigRatio (Q)': 'eig_ratio',
        'eigRatio': 'eig_ratio',
        'coverageRatio': 'coverage',
        'fieldMag (raw)': 'field_mag_ut',
        'fieldMag': 'field_mag_ut',
    }.get(key)
    if norm is None:
        return False
    state[norm] = val
    if norm == 'samples':
        state['samples'] = int(val)
    if unit:
        state['unit'] = unit
    return True


def assess_magcal(state: dict[str, Any]) -> dict[str, Any]:
    """@brief 按质量判据评估一次磁标定结果
    @param state parse_magcal_line 累积出来的字典
    @return {ok, issues[], score, checks[]}

    @note score 是 0–100 的粗略打分，只用于横向比较两次标定的相对好坏，
          不能当成"合格证"——最终判据以 issues 为空为准。
    """
    checks: list[dict[str, Any]] = []
    issues: list[str] = []

    def add(name, value, ok, msg_ok, msg_bad):
        checks.append({'name': name, 'value': value, 'ok': ok,
                       'msg': msg_ok if ok else msg_bad})
        if not ok:
            issues.append(msg_bad)

    n = state.get('samples')
    if n is None:
        issues.append('未读到采样数（采集未开始，或固件未回传）')
        add('样本数', '--', False, '', '样本数缺失')
    else:
        add('样本数', n, n >= MIN_SAMPLES,
            f'{n} 个样本，足够拟合',
            f'{n} 个样本 < {MIN_SAMPLES}，请继续缓慢旋转采集')

    r = state.get('residual_rms')
    if r is not None:
        add('拟合残差', f'{r:.4f}', r <= MAX_RESIDUAL_RMS,
            f'残差 {r:.4f}，拟合良好',
            f'残差 {r:.4f} > {MAX_RESIDUAL_RMS}，疑似磁干扰或样本分布太差')

    e = state.get('eig_ratio')
    if e is not None:
        lo, hi = EIG_RATIO_OK
        add('特征值比', f'{e:.3f}', lo <= e <= hi,
            f'特征值比 {e:.3f}，软铁畸变在可接受范围',
            f'特征值比 {e:.3f} 超出 {lo}–{hi}，软铁畸变偏大')

    c = state.get('coverage')
    if c is not None:
        add('姿态覆盖', f'{c:.3f}', c >= MIN_COVERAGE,
            f'覆盖率 {c:.3f}，三个方向都采到了',
            f'覆盖率 {c:.3f} < {MIN_COVERAGE}，只在一个方向采过，换个姿态再采')

    f = state.get('field_mag_ut')
    if f is not None:
        ok = abs(f - FIELD_MAG_REF_UT) <= FIELD_MAG_TOL_UT
        add('总场强度', f'{f:.1f} uT', ok,
            f'总场 {f:.1f} uT，与本地地磁参考接近',
            f'总场 {f:.1f} uT 偏离参考 {FIELD_MAG_REF_UT}±{FIELD_MAG_TOL_UT} uT，'
            f'检查附近磁源或换地点重标')

    n_ok = sum(1 for ck in checks if ck['ok'])
    score = int(round(100.0 * n_ok / len(checks))) if checks else 0
    return {'ok': not issues and bool(checks), 'issues': issues,
            'score': score, 'checks': checks}


# ---- 向导步骤定义 ----
# 每步：id / 标题 / 要发的 DBG 命令 / 操作提示 / 这步在等什么样的输出
MAG_WIZARD = [
    {
        'id': 'status',
        'title': '① 查看当前标定状态',
        'cmd': 'magcal',
        'hint': '先确认当前参数与采样数，作为改前基线',
        'wait_s': 1.5,
    },
    {
        'id': 'guided',
        'title': '② 两圈引导校准',
        'cmd': 'magcal guided',
        'hint': '按顺序做两圈：先保持水平缓慢转一整圈，再让机头朝下缓慢转一整圈。'
                '过程中不要靠近铁磁物、电机线、扬声器。',
        'wait_s': 120.0,
    },
    {
        'id': 'assess',
        'title': '③ 判读结果',
        'cmd': 'magcal',
        'hint': '读取残差 / 特征值比 / 覆盖率 / 总场，判断这次标定是否可用',
        'wait_s': 2.0,
    },
    {
        'id': 'save',
        'title': '④ 保存到 Flash',
        'cmd': 'magcal save',
        'hint': '这一步务必执行——不保存的话断电重启会回到旧参数',
        'wait_s': 2.0,
    },
]

WIZARD_INDEX = {s['id']: i for i, s in enumerate(MAG_WIZARD)}
