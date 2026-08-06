# -*- coding: utf-8 -*-
"""py-web-lab 消息协议（client ⇄ server，WebSocket JSON）。

client → server:
  {"cmd": "init"}                          # 全量状态 + 参数快照
  {"cmd": "reset", "mode": "cruise"|"vtol"|"pose"}
  {"cmd": "set", "S": {"thr": .., "dt": .., ...}}   # 白名单字段
  {"cmd": "pulse", "omega": {"x": .., "y": .., "z": ..}}   # 角速度扰动注入
  {"cmd": "step", "dt": 0.016}             # 推进一帧（内部按 maxStep 分子步）

server → client:
  {"type": "state", "cmd": <回显>, "S": {...}, "F": {...}, "dyn": {...}, "aero": {...}, "t": ..}
  {"type": "params", "P": {...}}           # init 响应附加
  {"type": "error", "msg": "..."}
"""
import math
import numpy as np

# ------------------------------------------------------------
#  数值/布尔校验辅助（review #1/#2/#6：统一有限性与范围校验）
# ------------------------------------------------------------
def _num(v, lo=None, hi=None):
    """转换为有限 float，可带范围校验；失败抛 ValueError"""
    x = float(v)
    if not math.isfinite(x):
        raise ValueError(f'not finite: {v!r}')
    if lo is not None and x < lo:
        raise ValueError(f'below {lo}: {x}')
    if hi is not None and x > hi:
        raise ValueError(f'above {hi}: {x}')
    return x


def _flag(v):
    """严格布尔校验（review #6：bool('false') 陷阱）"""
    if not isinstance(v, bool):
        raise ValueError(f'not bool: {v!r}')
    return v


# S 字段白名单：字段 → (转换器, 说明)
#   * 执行器输出（dtAct/dfAct/dwAct）由控制律计算，不对外开放（review #2）
#   * paused：仿真推进开关（step 不推进，仅回状态）
S_FIELDS = {
    'thr': ('num', 0.0, 1.0),
    'dt': ('num', -math.pi, math.pi),
    'df': ('num', -math.pi, math.pi),
    'dw': ('num', -math.pi, math.pi),
    'sasMode': ('int', 0, 3),
    'ctrl': 'ctrl',          # 控制律：sas/indi/lqr/adrc
    'aero': 'flag', 'lockXY': 'flag', 'vtolMode': 'flag',
    'useBtrue': 'flag', 'altHold': 'flag',
    'altRef': ('num', 0.0, 100.0),
    'intTh': ('num', None, None), 'intPhi': ('num', None, None),
    'intAlt': ('num', None, None),
    'paused': 'flag',
}

# 参数快照（前端 HUD/界面需要的子集；全量见 aircraft-model.json）
PARAM_KEYS = [
    'm', 'g', 'Ix', 'Iy', 'Iz', 'a', 'b', 'bspan', 'Sw', 'c',
    'kT', 'kQ', 'Jp', 'wMax', 'tauM',
    'vTrim', 'aTrim', 'thrTrim', 'dtTrim', 'dfTrim',
    'groundZ', 'rho', 'vMin',
    'sasQ', 'sasTh', 'sasI', 'sasP', 'sasPhi', 'sasIPhi', 'sasR',
    'rateKq', 'rateKr', 'rateKp', 'intThMax', 'intPhiMax', 'dMax', 'dwMax',
    'vtolAttKp', 'btrueK',
    'altKpH', 'altKpI', 'altKpV', 'altVZMax', 'maxStep',
]

STATE_KEYS = ['thr', 'dt', 'df', 'dw', 'dtAct', 'dfAct', 'dwAct',
              'sasMode', 'ctrl', 'aero', 'lockXY', 'vtolMode', 'useBtrue',
              'altHold', 'altRef', 'intAlt', 'intTh', 'intPhi',
              'paused', 'wf', 'wt', 'omega', 'quat', 'time']


def _snap_S(sim):
    S = sim.S
    return {k: S[k] for k in STATE_KEYS}


def _snap_F(sim):
    F = sim.F
    return {'vel': dict(F['vel']), 'vWorld': dict(F['vWorld']),
            'pos': dict(F['pos']), 'euler': dict(F['euler'])}


def _snap_dyn(sim):
    return dict(sim.dyn)


def _snap_aero(sim):
    return dict(sim.aero)


def state_payload(sim, cmd=None):
    """状态响应；cmd 回显请求类型，供客户端区分 step/set（review #4）"""
    payload = {'type': 'state',
               'S': _snap_S(sim), 'F': _snap_F(sim),
               'dyn': _snap_dyn(sim), 'aero': _snap_aero(sim),
               't': sim.S['time']}
    if cmd is not None:
        payload['cmd'] = cmd
    return payload


def handle(sim, msg):
    """处理一条 client 消息，返回待发送的响应（None = 不响应）"""
    if not isinstance(msg, dict) or 'cmd' not in msg:
        return {'type': 'error', 'msg': 'bad message'}
    cmd = msg['cmd']

    def err(msg_text):
        return {'type': 'error', 'cmd': cmd, 'msg': msg_text}

    if cmd == 'init':
        resp = state_payload(sim, cmd)
        resp['params'] = {k: sim.P[k] for k in PARAM_KEYS if k in sim.P}
        return resp

    if cmd == 'reset':
        mode = msg.get('mode', 'cruise')
        if mode == 'vtol':
            sim.reset_vtol()
        elif mode == 'pose':
            sim.reset_pose_only()
        elif mode == 'cruise':
            sim.reset_full()
        else:
            return err(f'bad reset mode: {mode}')
        return state_payload(sim, cmd)

    if cmd == 'set':
        fields = msg.get('S', {})
        if not isinstance(fields, dict):
            return err('S must be object')
        try:
            for k, v in fields.items():
                if k not in S_FIELDS:
                    raise ValueError(f'unknown field: {k}')
                if v is None:
                    continue
                spec = S_FIELDS[k]
                if spec == 'flag':
                    sim.S[k] = _flag(v)
                elif spec == 'ctrl':
                    if v not in ('sas', 'indi', 'lqr', 'adrc'):
                        raise ValueError(f'bad ctrl: {v}')
                    sim.S[k] = v
                else:
                    conv, lo, hi = spec
                    if conv == 'int':
                        val = int(v)
                        if val != v and not isinstance(v, bool):
                            raise ValueError(f'not int: {v!r}')
                        if lo is not None and (val < lo or val > hi):
                            raise ValueError(f'out of range: {v}')
                        sim.S[k] = val
                    else:
                        sim.S[k] = _num(v, lo, hi)
        except (TypeError, ValueError) as exc:
            return err(f'bad value: {exc}')
        return state_payload(sim, cmd)

    if cmd == 'pulse':
        w = msg.get('omega', {})
        if not isinstance(w, dict):
            return err('omega must be object')
        try:
            for k in ('x', 'y', 'z'):
                if k in w and w[k] is not None:
                    sim.S['omega'][k] = _num(w[k], -100.0, 100.0)
        except (TypeError, ValueError) as exc:
            return err(f'bad omega: {exc}')
        return state_payload(sim, cmd)

    if cmd == 'step':
        try:
            dt = _num(msg.get('dt', 0.016), 1e-6, 1.0)
        except (TypeError, ValueError) as exc:
            return err(f'bad dt: {exc}')
        if not sim.S['paused']:
            import controllers
            n = max(1, int(np.ceil(dt / sim.P['maxStep'])))
            h = dt / n
            for _ in range(n):
                sim.step(h, controllers.apply)
        return state_payload(sim, cmd)

    if cmd == 'shutdown':
        # 由 main.py 拦截处理（响应后进程退出）；此处兜底不落地
        return {'type': 'bye', 'msg': 'server shutting down'}

    return err(f'unknown cmd: {cmd}')
