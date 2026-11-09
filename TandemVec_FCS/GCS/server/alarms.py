#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file    alarms.py
@module  GCS/后端 报警与事件引擎
@brief   把散落各处的告警条件收敛成一套带状态机与历史记录的规则引擎
@details 为什么放在后端而不是前端（原来 checkAlarms 在 main.mjs 里）：
         ① 告警必须**独立于当前页面**——切到别的页就不报警，是最危险的失效模式；
         ② 事件历史要能事后查，前端内存里的历史一刷新就没；
         ③ 多机管理以后要按机区分告警，状态天然属于服务端。

         状态机只有两态：正常 / 触发。规则从"正常"变"触发"时记一条事件并推送，
         保持"触发"期间只在首次推送（否则 50 Hz 会刷屏），回到"正常"时也记一条
         ——恢复本身是重要信息，出问题时"什么时候好的"和"什么时候坏的"一样要用。

         冷却时间（cooldown_s）只抑制**重复推送**，不影响事件记录：抖动的量
         （比如电压在阈值附近来回）不该把事件表刷满，但也不该让判断失真。

@theory  N/A（阈值判据见各规则的 note）
@frame   N/A
@units   电压 V、姿态 deg、卫星 个、时间 s
@note    阈值刻意做成模块级常量而不是散在判断里，便于统一调整与核对。
@test    tests/test_alarms.py：上升沿触发、同态不重复、恢复记录、冷却抑制
@see     TandemVec_FCS/GCS/web/src/alarms.mjs（展示层）
"""

from __future__ import annotations

import time
from typing import Any, Callable

# ---- 阈值：集中在这里，便于核对与调整 ----
# 3S 锂电（本项目用法）：满 12.6 V，报警取 11.1 V（约 3.7 V/cell），
# 严重取 10.5 V（约 3.5 V/cell，继续放电会伤电池且没电返航）。
BAT_WARN_V = 11.1
BAT_CRIT_V = 10.5
# 卫星数：<6 时定位精度不可靠；不锁定时只是提示，不是错误。
MIN_SATS_WARN = 6
# 姿态：横滚/俯仰超过 45° 已不是正常飞行姿态（悬停构型下尤其反常）。
ATT_LIMIT_DEG = 45.0
# 链路：遥测超过这么久没更新，视为中断。
LINK_TIMEOUT_S = 2.0
# GPS 水平精度：>5 m 时不适合做定点/自动飞行。
GPS_SACC_WARN_M = 5.0

# 历史保留条数：够查一次飞行的问题，又不至于吃内存
MAX_EVENTS = 500


class AlarmEngine:
    """@brief 规则引擎：每次喂入遥测快照，输出新增/恢复的事件"""

    def __init__(self):
        self.rules: dict[str, dict[str, Any]] = {}
        self.active: dict[str, dict[str, Any]] = {}   # key -> 当前活动告警
        self.events: list[dict[str, Any]] = []        # 历史事件（新的在后）
        self._last_push: dict[str, float] = {}
        self._last_tele = 0.0
        self._build_rules()

    # ---------------- 规则定义 ----------------
    def _add(self, key, level, title, fn, cooldown_s=10.0, note=''):
        self.rules[key] = {
            'key': key, 'level': level, 'title': title, 'fn': fn,
            'cooldown_s': cooldown_s, 'note': note,
        }

    def _build_rules(self):
        """@brief 注册全部规则。

        @note fn(snap, ctx) 返回 (是否触发, 描述文本) 或 False。
              ctx 里带 now/link_age_s 这类引擎侧信息，规则本身不碰时钟。
        """
        # 电池：两档阈值合并成一条规则，描述里区分严重程度，避免同一时刻
        # 两条规则各报一次（界面会显得很吵）
        def _bat(snap, ctx):
            v = snap.get('bat_voltage_v')
            if v is None or v <= 0:
                return False
            if v < BAT_CRIT_V:
                return f'电池严重欠压 {v:.2f} V（<{BAT_CRIT_V} V，立即降落）'
            if v < BAT_WARN_V:
                return f'电池电压偏低 {v:.2f} V（<{BAT_WARN_V} V）'
            return False

        self._add('battery', 'crit' if False else 'warn', '电池电压', _bat,
                  cooldown_s=15.0,
                  note=f'低于 {BAT_WARN_V} V 报警，低于 {BAT_CRIT_V} V 严重')

        def _sats(snap, ctx):
            n = snap.get('gps_sats')
            if n is None:
                return False
            if n < MIN_SATS_WARN:
                return f'卫星数不足 {n}（<{MIN_SATS_WARN}）'
            return False

        self._add('sats', 'warn', 'GNSS 卫星', _sats, cooldown_s=30.0)

        def _gpsfix(snap, ctx):
            fix = snap.get('gps_fix')
            if fix is None:
                return False
            # 0/1 无定位；2 为 2D；3 起才是 3D
            if int(fix) < 3:
                return f'GNSS 未获得 3D 定位（fix={fix}）'
            return False

        self._add('gps_fix', 'warn', 'GNSS 定位', _gpsfix, cooldown_s=30.0)

        def _sacc(snap, ctx):
            sacc = snap.get('gps_sacc')
            if sacc is None:
                return False
            if sacc > GPS_SACC_WARN_M:
                return f'水平定位精度差 {sacc:.1f} m（>{GPS_SACC_WARN_M} m）'
            return False

        self._add('gps_sacc', 'warn', 'GNSS 精度', _sacc, cooldown_s=30.0)

        def _att(snap, ctx):
            r = snap.get('roll_deg')
            p = snap.get('pitch_deg')
            if r is None and p is None:
                return False
            r = abs(float(r or 0.0))
            p = abs(float(p or 0.0))
            if max(r, p) > ATT_LIMIT_DEG:
                return f'姿态超限 横滚 {r:.1f}° 俯仰 {p:.1f}°（>{ATT_LIMIT_DEG:.0f}°）'
            return False

        self._add('attitude', 'crit', '姿态超限', _att, cooldown_s=5.0)

        def _link(snap, ctx):
            age = ctx.get('link_age_s')
            if age is None:
                return False
            if age > LINK_TIMEOUT_S:
                return f'遥测中断 {age:.1f} s（>{LINK_TIMEOUT_S} s）'
            return False

        self._add('link', 'crit', '链路上行', _link, cooldown_s=5.0)

        def _disarm(snap, ctx):
            u = snap.get('unlocked')
            if u is None:
                return False
            return '飞行器已解锁' if u else False

        self._add('armed', 'info', '解锁状态', _disarm, cooldown_s=0.0)

    # ---------------- 推理 ----------------
    def feed(self, snap: dict, link_age_s: float | None = None,
             now: float | None = None) -> list[dict]:
        """@brief 喂一次快照，返回本次新产生的事件列表（含恢复事件）

        @param snap 遥测快照
        @param link_age_s 距上次收到遥测的秒数（链路规则用）
        @param now 注入时钟，便于测试固定时间
        """
        now = time.time() if now is None else now
        ctx = {'now': now, 'link_age_s': link_age_s}
        out: list[dict] = []

        for key, rule in self.rules.items():
            try:
                res = rule['fn'](snap, ctx)
            except Exception:
                continue
            triggered = bool(res)
            text = res if isinstance(res, str) else (rule['title'] if triggered else '')

            was = key in self.active
            if triggered and not was:
                # 上升沿：记事件并推送
                ev = {
                    'ts': now, 'key': key, 'level': rule['level'],
                    'title': rule['title'], 'text': text, 'kind': 'raise',
                }
                self.active[key] = ev
                self.events.append(ev)
                self._last_push[key] = now
                out.append(ev)
            elif triggered and was:
                # 持续中：更新描述（读数会变），但只在冷却后才会再推送
                self.active[key]['ts'] = self.active[key].get('ts', now)
                self.active[key]['text'] = text
            elif (not triggered) and was:
                # 下降沿：记恢复事件
                ev = {
                    'ts': now, 'key': key, 'level': 'info',
                    'title': rule['title'], 'text': f'{rule["title"]} 已恢复',
                    'kind': 'clear',
                }
                self.active.pop(key, None)
                self.events.append(ev)
                out.append(ev)

        # 事件表截断（保留最近 MAX_EVENTS 条）
        if len(self.events) > MAX_EVENTS:
            del self.events[:len(self.events) - MAX_EVENTS]
        return out

    def snapshot(self) -> dict:
        """@brief 当前状态：活动告警 + 最近事件 + 规则说明"""
        return {
            'active': list(self.active.values()),
            'events': self.events[-100:],
            'rules': [
                {'key': r['key'], 'level': r['level'], 'title': r['title'],
                 'note': r['note'], 'cooldown_s': r['cooldown_s']}
                for r in self.rules.values()
            ],
        }

    def clear_events(self):
        """@brief 清空历史事件（活动告警不动——它们仍然真实存在）"""
        self.events.clear()
