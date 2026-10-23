# -*- coding: utf-8 -*-
"""
datalog.py — 实时遥测记录（CSV）与回放数据源

@file    datalog.py
@module  GCS/数据记录与回放
@brief   把遥测快照写 CSV，并按行回放为快照（复用前端渲染管线）
@details 输入：main.py 的 g.snap（已解码、带单位后缀的字段）；处理：固定列顺序追加写、
         缺列填空；回放时逐行读 CSV 并按后端推送周期广播；
         输出：output/*.csv 与 replay 快照。
         不变量：RECORD_FIELDS 与列顺序保持一致（前端/工具按列名消费）。
@theory  N/A
@ref     COMM-001 §4（遥测 50Hz 推送）、GCS-001 §上位机（记录/回放页）
@frame   N/A
@units   列名自带单位后缀；t_ms 为 ms（相对记录起点的快照时间）
@note    回放与实时遥测共用同一 telemetry 消息结构，前端零分支

- CsvRecorder: 把遥测快照 dict 追加写 CSV（列固定，缺列填空；时间戳 t_ms）
- ReplaySource: 读 CSV 按行推送快照（供前端复用仪表渲染管线回放）
"""
import csv
import os
import time

# 记录列（AnoCom 可得字段全集；与黑匣子通道语义对齐，缺失字段填空）
RECORD_FIELDS = [
    't_ms', 'roll_deg', 'pitch_deg', 'heading_deg',
    'acc_x_ms2', 'acc_y_ms2', 'acc_z_ms2',
    'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps',
    'vel_n_ms', 'vel_e_ms', 'vel_up_ms',
    'rel_n_m', 'rel_e_m', 'rel_h_m',
    'alt_bar_m', 'alt_fu_m',
    'flight_mode', 'unlocked',
    'ctrl_roll', 'ctrl_pitch', 'ctrl_thr_pct', 'ctrl_yaw',
    'ctrl_roll_dps2', 'ctrl_pitch_dps2', 'ctrl_yaw_dps2',
    'target_roll_deg', 'target_pitch_deg', 'target_yaw_rate_dps',
    'rc1', 'rc2', 'rc3', 'rc4', 'rc5', 'rc6', 'rc7', 'rc8',
    'p1_mpa', 'p2_mpa', 'bat_voltage_v',
    'gps_fix', 'gps_sats', 'gps_lat', 'gps_lon', 'gps_alt_m',
]


class CsvRecorder:
    """遥测 CSV 记录器：按 RECORD_FIELDS 固定列追加写，缺列填空。

    @brief  打开/关闭 CSV 文件并按快照写一行
    @param  path 输出 CSV 路径（output/ 下）
    @note   t_ms 为快照时间戳（ms），由调用方保证单调
    """
    def __init__(self, path):
        self.path = path
        self._f = None
        self._w = None

    @property
    def active(self):
        return self._f is not None

    def start(self):
        if self._f:
            return
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self._f = open(self.path, 'w', newline='', encoding='utf-8')
        self._w = csv.DictWriter(self._f, fieldnames=RECORD_FIELDS,
                                 extrasaction='ignore', restval='')
        self._w.writeheader()

    def write(self, snap: dict):
        if not self._w:
            return
        row = {k: _fmt(v) for k, v in snap.items()}
        self._w.writerow(row)

    def stop(self):
        if self._f:
            self._f.close()
            self._f = None
            self._w = None


def _fmt(v):
    if isinstance(v, bool):
        return '1' if v else '0'
    if isinstance(v, float):
        return f'{v:.4f}'
    return v


def list_recordings(dir_path):
    """列出记录目录下 .csv 文件（按修改时间倒序）"""
    if not os.path.isdir(dir_path):
        return []
    out = []
    for fn in sorted(os.listdir(dir_path)):
        if fn.endswith('.csv'):
            p = os.path.join(dir_path, fn)
            out.append({'name': fn, 'size': os.path.getsize(p),
                        'mtime': os.path.getmtime(p)})
    out.sort(key=lambda x: x['mtime'], reverse=True)
    return out


class ReplaySource:
    """CSV 回放：next_row() 逐行返回 dict（含 t_ms）"""

    def __init__(self, path):
        self.path = path
        self._f = None
        self._r = None
        self._t0 = None

    def open(self):
        self._f = open(self.path, 'r', newline='', encoding='utf-8')
        self._r = csv.DictReader(self._f)
        self._t0 = None

    def next_row(self):
        row = next(self._r, None)
        if row is None:
            return None
        out = {}
        for k, v in row.items():
            if v == '':
                out[k] = None
            else:
                try:
                    out[k] = float(v)
                except ValueError:
                    out[k] = v
        if self._t0 is None and out.get('t_ms') is not None:
            self._t0 = out['t_ms']
        return out

    def close(self):
        if self._f:
            self._f.close()
            self._f = None
