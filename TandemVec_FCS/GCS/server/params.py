# -*- coding: utf-8 -*-
"""
params.py — 参数显示元数据（分组/单位/范围/说明）

★ 名称与固件 ano_params.cpp 注册表严格同名（att_roll.kp 等）——
  上位机先经 0xE0 CMD 0x03 拉取固件 E2 信息帧（ID→名称/类型），
  再用本表按名称补充分组/单位/范围；未命中名称自动落入"其他"组。
  固件参数 ID 顺序变更不影响本表（按名称匹配）。
"""
import re

# 环 → 分组/显示名/单位说明
PID_GROUPS = [
    ('att_roll', '姿态外环（deg 域）', 'Roll 外环：姿态角误差 → 目标角速率'),
    ('att_pitch', '姿态外环（deg 域）', 'Pitch 外环：姿态角误差 → 目标角速率'),
    ('att_yaw', '姿态外环（deg 域）', 'Yaw 外环：ratchet hold（打杆恒速，回中锁航向）'),
    ('rate_roll', '角速率内环（deg 域）', 'Roll 内环：速率误差 → 虚拟角加速度'),
    ('rate_pitch', '角速率内环（deg 域）', 'Pitch 内环：速率误差 → 虚拟角加速度'),
    ('rate_yaw', '角速率内环（deg 域）', 'Yaw 内环（差速）：速率误差 → 虚拟角加速度'),
    ('alt_pos', '垂直串级', '高度外环：高度误差 → 目标垂直速度'),
    ('alt_vel', '垂直串级', '垂直速度内环 → 目标垂直加速度'),
    ('pos_n', '水平位置环', '北向位置外环 → 目标北向速度'),
    ('pos_e', '水平位置环', '东向位置外环 → 目标东向速度'),
    ('vel_n', '水平速度环', '北向速度内环 → 目标北向加速度'),
    ('vel_e', '水平速度环', '东向速度内环 → 目标东向加速度'),
]

FILTER_GROUPS = [
    ('speed_filter_alpha', '角速率滤波', '内环输入角速率滤波 alpha'),
    ('angle_out_filter_alpha', '外环输出滤波', '姿态外环输出滤波 alpha'),
    ('output_filter_alpha', '内环输出滤波', '内环输出（执行器指令）滤波 alpha'),
    ('speed_filter_alpha2', '角速率滤波2', '二级滤波 alpha（级联二阶，抑 30-60Hz 桨振动；2026-08-09）'),
]

# 字段 → 元数据（单位/范围/步进/说明）
# ★ 参数名 ≤20B（协议 PAR_NAME 定长）：filter_alpha 的线上名 = "{loop}.falpha"
FIELD_META = {
    'kp': dict(unit='', min=0.0, max=100.0, step=0.01, desc='比例增益'),
    'ki': dict(unit='', min=0.0, max=2.0, step=0.01, desc='积分增益（连续域，dt 显式传入）'),
    'kd': dict(unit='', min=0.0, max=100.0, step=0.01, desc='微分增益（作用于每拍差分，未除以 dt）'),
    'out_min': dict(unit='', min=-1000.0, max=0.0, step=0.1, desc='输出下限'),
    'out_max': dict(unit='', min=0.0, max=1000.0, step=0.1, desc='输出上限'),
    'int_limit': dict(unit='', min=0.0, max=1000.0, step=1.0, desc='积分项输出上限（内部反推状态界）'),
    'threshold': dict(unit='', min=0.0, max=1000.0, step=0.1, desc='积分分离阈值'),
    'filter_alpha': dict(unit='', min=0.0, max=1.0, step=0.01, desc='微分滤波系数', wire_name='falpha'),
    'enabled': dict(unit='', min=0.0, max=1.0, step=1.0, desc='环是否参与控制（只读）', readonly=True),
    # ---- FPV 摇杆曲线（★2026-08-11 RATE_MODE 接入，Betaflight 三参数模型）----
    'rc_rate': dict(unit='', min=0.0, max=2.5, step=0.01, desc='全局灵敏度（满杆基准 200°/s）'),
    'rc_expo': dict(unit='', min=0.0, max=1.0, step=0.01, desc='输入中心曲线（压中心斜率）'),
    'rc_super': dict(unit='', min=0.0, max=0.99, step=0.01, desc='输出边缘曲线（双曲增益，<1 防极点）'),
}

# 滤波数组 → 线上短名（固件 ALPHA_ENTRY，≤20B）
FILTER_WIRE = {
    'speed_filter_alpha': 'spd_alpha',
    'angle_out_filter_alpha': 'ang_alpha',
    'output_filter_alpha': 'out_alpha',
    'speed_filter_alpha2': 'spd2_alpha',
}

# 按环的输出单位修正
RATE_UNIT = 'deg/s'   # 姿态外环 out_min/out_max
ANG_ACC_UNIT = 'deg/s²'  # 角速率内环 out_min/out_max/int_limit
ACC_UNIT = 'm/s²'     # 速度内环
VEL_UNIT = 'm/s'      # 位置环 out
THR_UNIT = 'm/s'      # alt_pos out（垂直速度目标）


def _loop_out_unit(loop):
    if loop.startswith('att_'):
        return RATE_UNIT
    if loop.startswith('rate_'):
        return ANG_ACC_UNIT
    if loop.startswith('alt_pos'):
        return VEL_UNIT
    if loop.startswith('alt_vel'):
        return ACC_UNIT
    if loop.startswith('pos_'):
        return VEL_UNIT
    if loop.startswith('vel_'):
        return ACC_UNIT
    return ''


def build_meta():
    """生成 {name: meta} 完整元数据表（117 参数，name = 线上名）"""
    meta = {}
    for loop, group, desc in PID_GROUPS:
        for field, fm in FIELD_META.items():
            wire = fm.get('wire_name', field)
            name = f'{loop}.{wire}'
            m = dict(fm)
            m['name'] = name
            m['group'] = group
            m['loop'] = loop
            m['field'] = field
            m['type'] = 'uint8' if field == 'enabled' else 'float'
            m['desc'] = desc + ' · ' + m['desc']
            if field in ('out_min', 'out_max'):
                m['unit'] = _loop_out_unit(loop)
            if loop.startswith('rate_'):
                if field == 'kp':
                    m['unit'] = 's⁻¹'
                elif field == 'ki':
                    m['unit'] = 's⁻²'
                    m['max'] = 100.0
                elif field == 'kd':
                    # PositionPID 的速率环 D 项仍按每拍差分计算，未除以 dt；
                    # 因此当前参数量纲与 kp 相同。kd 默认为 0，后续若改为
                    # 真正的时间导数实现，必须同时迁移数值与此元数据。
                    m['unit'] = 's⁻¹'
                elif field == 'int_limit':
                    m['unit'] = ANG_ACC_UNIT
                    m['max'] = 10000.0
                    m['desc'] = desc + ' · 积分输出贡献 |ki·∫e dt| 的限幅'
                elif field == 'threshold':
                    m['unit'] = RATE_UNIT
                elif field == 'out_min':
                    m['min'] = -10000.0
                elif field == 'out_max':
                    m['max'] = 10000.0
            meta[name] = m
    for arr, group, desc in FILTER_GROUPS:
        wire = FILTER_WIRE.get(arr, arr)
        for i in range(3):
            name = f'{wire}[{i}]'
            meta[name] = dict(name=name, group=group, loop=arr, field=str(i),
                              unit='', min=0.0, max=1.0, step=0.01,
                              desc=f'{desc}（通道 {i}）', type='float')
    # ★ 2026-08-11 FPV 摇杆曲线（RATE_MODE，Betaflight 三参数模型）
    rc_desc = {'rc_rate': '全局灵敏度（满杆基准 200°/s）',
               'rc_expo': '输入中心曲线（压中心斜率）',
               'rc_super': '输出边缘曲线（双曲增益）'}
    for arr, group in (('rc_expo', '摇杆曲线'), ('rc_super', '摇杆曲线')):
        for i in range(3):
            name = f'{arr}[{i}]'
            fm = FIELD_META[arr]
            meta[name] = dict(name=name, group=group, loop=arr, field=str(i),
                              unit='', min=fm['min'], max=fm['max'], step=fm['step'],
                              desc=f"{rc_desc[arr]}（{'roll/pitch/yaw'[i]} 通道）", type='float')
    fm = FIELD_META['rc_rate']
    meta['rc_rate'] = dict(name='rc_rate', group='摇杆曲线', loop='rc_rate', field='rate',
                           unit='', min=fm['min'], max=fm['max'], step=fm['step'],
                           desc=rc_desc['rc_rate'], type='float')
    # ★ 2026-08-10 惯量逆解交叉耦合前馈使能掩码（标量；A/B 在线开关）
    meta['inertia_comp_mask'] = dict(name='inertia_comp_mask', group='其他', loop='',
                                     field='', unit='', min=0, max=3, step=1,
                                     type='uint8',
                                     desc='惯量逆解前馈掩码（bit0 陀螺耦合 / bit1 转子陀螺；0=全关）')
    return meta


META = build_meta()


def group_order():
    """分组显示顺序（前端按此渲染分组头）"""
    order = []
    for _, g, _ in PID_GROUPS:
        if g not in order:
            order.append(g)
    for _, g, _ in FILTER_GROUPS:
        if g not in order:
            order.append(g)
    order.append('其他')
    return order


def meta_for(name):
    """按名称取元数据；未命中返回 None（落入前端"其他"组）"""
    return META.get(name)


def is_float_name(name):
    m = meta_for(name)
    if m:
        return m['type'] == 'float'
    # 未知参数按 float 猜测（固件 E2 会给出真实类型，前端以固件为准）
    return True


# 固件注册表同序参数名列表（校验用：保证 121 个线上名与固件一致）
def expected_names():
    """按固件 ano_params.cpp 注册表顺序返回 121 个线上参数名（跨实现校验）"""
    names = []
    for loop, _, _ in PID_GROUPS:
        for field, _ in FIELD_META.items():
            if field.startswith('rc_'):
                continue  # ★ 2026-08-11 FPV 摇杆曲线字段不随 PID 环展开（尾部独立注册）
            wire = FIELD_META[field].get('wire_name', field)
            names.append(f'{loop}.{wire}')
    # ★ 固件注册表尾部顺序（ano_params.cpp:99-103）：前 3 组滤波(108-116) →
    #   inertia_comp_mask(117) → spd2_alpha[0..2](118-120)。2026-08-10 全面
    #   审查修复：原实现把 spd2_alpha 排 117-119、mask 排 120——名字→ID 映射
    #   错位，`param set inertia_comp_mask 1` 会静默写进 spd2_alpha[2]（高危）
    for arr, _, _ in FILTER_GROUPS:
        if arr == 'speed_filter_alpha2':
            continue  # spd2_alpha 在 mask 之后（固件注册表顺序）
        wire = FILTER_WIRE.get(arr, arr)
        for i in range(3):
            names.append(f'{wire}[{i}]')
    names.append('inertia_comp_mask')
    wire2 = FILTER_WIRE.get('speed_filter_alpha2', 'speed_filter_alpha2')
    for i in range(3):
        names.append(f'{wire2}[{i}]')
    # ★ 2026-08-11 FPV 摇杆曲线（固件 ano_params.cpp 尾部同序，id 121-127）
    names.append('rc_rate')
    for i in range(3):
        names.append(f'rc_expo[{i}]')
    for i in range(3):
        names.append(f'rc_super[{i}]')
    return names
