"""
@file    caps.py
@module  GCS/能力清单
@brief   GCS 全部命令的机器可读清单：模块归属、参数、回复类型、说明
@details 三件用途：①`GET /api/caps` 与 `tools/gcs_ctl.py caps` 让调用方
（人、agent、脚本）不必读源码就知道能做什么、怎么传参；
②新增功能时先在这里登记，再实现——清单即扩展入口；
③`verify_against_source()` 与 main.py 实际分派比对，防清单与实现漂移
（清单写了但没实现、实现了但没登记，两种都报错）。
@theory  N/A
@frame   N/A
@units   N/A
@ref     GCS-001 §上位机（命令与页面）、PROTOCOL-001、docs/GCS-扩展指南.md
"""
from __future__ import annotations

import pathlib
import re

# ----------------------------------------------------------------------
#  参数类型简写：n=数值 f=浮点 s=字符串 l=列表 b=布尔 o=对象
# ----------------------------------------------------------------------
ARG_TYPES = {'n': 'number', 'f': 'float', 's': 'string', 'l': 'list', 'b': 'bool', 'o': 'object'}


def _a(name: str, typ: str, desc: str, required: bool = False) -> dict:
    """@brief 构造一条参数描述（保持清单书写紧凑）"""
    return {'name': name, 'type': ARG_TYPES[typ], 'desc': desc, 'required': required}


def _c(name: str, desc: str, args: list | None = None, reply: str | None = None,
       readonly: bool = True) -> dict:
    """@brief 构造一条命令描述
    @param readonly True=只读查询（不改飞控/不改本机状态）；False=有副作用
    @note  readonly 是给自动化工具有用的护栏信息：只读命令可以放心批量试，
           有副作用的命令（改参数/重启/解锁）必须单个确认。
    """
    return {'cmd': name, 'desc': desc, 'args': args or [],
            'reply': reply, 'readonly': readonly}


# ----------------------------------------------------------------------
#  模块清单：一个模块 = 一个功能域，成组出现便于按域查询与扩展
#  新增功能：①在这里给所属模块加一条命令 ②在 main.py 实现分派 ③跑守卫比对
# ----------------------------------------------------------------------
MODULES: list[dict] = [
    {
        'id': 'link', 'name': '链路连接',
        'desc': '串口/蓝牙链路建立与状态；协议切换（AnoCom / MAVLink）',
        'cmds': [
            _c('connect', '建立串口链路并开始解析遥测',
               [_a('port', 's', '串口名（COM5）或 UDP 端点（udp:14550 / udp:14550@192.168.1.100:14550）', True),
                _a('baud', 'n', '波特率，默认取后端 DEFAULT_BAUD'),
                _a('protocol', 's', 'anocom 或 mavlink，默认 anocom')],
               'conn', readonly=False),
            _c('disconnect', '断开链路', reply='conn', readonly=False),
            _c('list_ports', '列出本机可用串口'),
            _c('device_info', '读设备信息帧（0x34）'),
            _c('hex_on', '开启原始字节十六进制观察', reply='hex_state', readonly=False),
            _c('hex_off', '关闭十六进制观察', reply='hex_state', readonly=False),
        ],
    },
    {
        'id': 'scope', 'name': '波形与统计',
        'desc': '时序缓冲查询：历史波形、选区统计（RMS/峰峰值/主频）',
        'cmds': [
            _c('scope_history', '取若干通道的最近历史点（后端 hist 缓冲）',
               [_a('channels', 'l', '通道名数组，如 ["roll_deg","pitch_deg"]', True)],
               'scope_history'),
            _c('scope_stats', '选区统计：RMS/峰峰值/均值/标准差/峰因素 + 主频与频带',
               [_a('channel', 's', '通道名', True),
                _a('t0', 'f', '起始 Unix 秒（省略=全部）'),
                _a('t1', 'f', '结束 Unix 秒')],
               'scope_stats'),
        ],
    },
    {
        'id': 'analysis', 'name': '频域与调参分析',
        'desc': '频谱、多通道共模对比、时频图、阶跃响应',
        'cmds': [
            _c('spectrum', '单通道频谱（Welch 分段平均 + 汉宁窗）',
               [_a('channel', 's', '通道名', True),
                _a('nperseg', 'n', '窗长样本数，默认 256')],
               'spectrum'),
            _c('spectrum_multi', '多通道频谱对比，判共模还是单轴问题',
               [_a('channels', 'l', '最多 4 个通道名', True),
                _a('nperseg', 'n', '窗长，默认 256')],
               'spectrum_multi'),
            _c('spectrogram', '时频图（STFT），看某频率何时出现',
               [_a('channel', 's', '通道名', True),
                _a('nperseg', 'n', '窗长，默认 128'),
                _a('hop', 'n', '帧移，默认 64'),
                _a('max_cols', 'n', '最大列数，默认 160（超限用最大值池化）')],
               'spectrogram'),
            _c('tuning_channels', '列出可用于分析的通道（样本数足够的）', reply='tuning_channels'),
            _c('tuning_analyze', '阶跃响应分析：上升时间/超调/稳态误差 + 振动诊断',
               [_a('cmd_ch', 's', '指令通道名', True),
                _a('fb_ch', 's', '反馈通道名', True),
                _a('min_delta', 'f', '最小阶跃幅度，0=自动'),
                _a('window_s', 'f', '分析窗长秒，默认 3.0')],
               'tuning_result'),
        ],
    },
    {
        'id': 'params', 'name': '参数（AnoCom 链路）',
        'desc': 'AnoCom 0xE0–0xE2 参数服务：全量读取 + 按名写入 + 恢复默认',
        'cmds': [
            _c('param_read_all', '逐 ID 读取固件全部参数（带重试，慢）'),
            _c('param_read', '按名或 ID 读取单个参数',
               [_a('name', 's', '参数名（推荐）'), _a('id', 'n', '参数 ID')], 'log'),
            _c('param_write', '按名或 ID 写入参数，过包线校验后下发并等校验帧',
               [_a('name', 's', '参数名（推荐）'), _a('id', 'n', '参数 ID'),
                _a('value', 'f', '目标值', True)],
               'param_written_pending', readonly=False),
            _c('param_restore', '恢复出厂默认参数', readonly=False),
            _c('param_save', '请求固件保存参数（仅回校验，不落 Flash）', readonly=False),
        ],
    },
    {
        'id': 'mavlink', 'name': 'MAVLink 双向通道',
        'desc': 'MAVLink v2 参数读写与命令下发（与 QGC/MP 同一套协议）',
        'cmds': [
            _c('mav_status', 'MAVLink 服务状态（已收参数条数等）', reply='mav_status'),
            _c('mav_param_list', '请求飞控广播全部参数（PARAM_REQUEST_LIST）', readonly=False),
            _c('mav_param_set', '写单个参数（PARAM_SET，固件写后立即 apply 生效）',
               [_a('name', 's', '参数名 ≤15B', True),
                _a('value', 'f', '目标值', True)],
               'mav_tx', readonly=False),
            _c('mav_cmd', '下发 MAV_CMD 命令（如 176 设模式 / 400 解锁 / 246 重启）',
               [_a('command', 'n', 'MAV_CMD 编号', True)] +
               [_a(f'p{i}', 'f', f'命令参数 {i}') for i in range(1, 8)],
               'mav_tx', readonly=False),
        ],
    },
    {
        'id': 'mission', 'name': '航点任务',
        'desc': '任务上传（拉取式协议）；固件当前回 MISSION_UNSUPPORTED',
        'cmds': [
            _c('mission_upload', '上传航点：先发 COUNT，之后按对端请求逐条应答',
               [_a('items', 'l', '航点数组 [{lat,lon,alt}]', True)],
               'mission_state', readonly=False),
            _c('mission_clear', '清空飞控任务', reply='mission_state', readonly=False),
        ],
    },
    {
        'id': 'vehicle', 'name': '多机管理',
        'desc': '一条链路承载多台机（按 sysid 分离）',
        'cmds': [
            _c('vehicles', '列出已发现的机号与最后活跃时间', reply='vehicles'),
            _c('vehicle_select', '切换当前界面选中的机',
               [_a('sysid', 'n', '机号', True)], 'vehicle_selected', readonly=False),
        ],
    },
    {
        'id': 'inspector', 'name': 'MAVLink Inspector',
        'desc': '已收消息的字段值与时间序列',
        'cmds': [
            _c('inspector', '消息列表：类型/频率/字段快照', reply='inspector'),
            _c('inspector_series', '取指定字段的历史序列用于绘图',
               [_a('keys', 'l', '形如 ["ATTITUDE.roll"] 的字段键', True)],
               'inspector_series'),
        ],
    },
    {
        'id': 'calib', 'name': '传感器校准',
        'desc': '磁力计等校准向导（经 DBG 通道驱动固件）',
        'cmds': [
            _c('calib_wizard', '取校准步骤清单', reply='calib_wizard'),
            _c('calib_step', '执行某一步校准',
               [_a('id', 's', '步骤 id（来自 calib_wizard）', True)],
               'calib_progress', readonly=False),
            _c('calib_assess', '评估已采集的校准数据', reply='calib_assess'),
            _c('calib_reset', '清空本轮采集', reply='calib_state', readonly=False),
        ],
    },
    {
        'id': 'alarm', 'name': '报警与事件',
        'desc': '规则引擎状态与事件历史',
        'cmds': [
            _c('alarm_state', '当前报警/阈值/事件列表', reply='alarm_state'),
            _c('alarm_clear_events', '清空事件历史', reply='alarm_state', readonly=False),
        ],
    },
    {
        'id': 'blackbox', 'name': '黑匣子',
        'desc': '固件 Flash 飞行记录：统计、分段、导出',
        'cmds': [
            _c('flash_stat', 'Flash 记录统计（经 DBG）'),
            _c('flash_findseg', '列出飞行段'),
            _c('flash_export', '分片导出原始记录并解码',
               [_a('start', 'n', '起始页'), _a('count', 'n', '页数，默认 2048')],
               readonly=False),
        ],
    },
    {
        'id': 'record', 'name': '记录与回放',
        'desc': 'CSV 记录、tlog/CSV 回放（含倍速与原始时间戳）',
        'cmds': [
            _c('record_start', '开始记录遥测到 CSV',
               [_a('file', 's', '文件名，默认带时间戳')], 'record_state', readonly=False),
            _c('record_stop', '停止记录', reply='record_state', readonly=False),
            _c('list_recordings', '列出已录制文件', reply='recordings'),
            _c('replay_start', '回放 CSV/tlog（清空时序缓冲后按原始时间戳灌入）',
               [_a('file', 's', '文件名或绝对路径', True)],
               'replay_state', readonly=False),
            _c('replay_speed', '回放倍速：0=最快，1.0=原始',
               [_a('speed', 'f', '倍速，默认 0.5')], 'replay_speed', readonly=False),
            _c('replay_stop', '停止回放', reply='replay_state', readonly=False),
        ],
    },
    {
        'id': 'debug', 'name': '固件调试通道',
        'desc': 'DBG 文本台（参数/IMU/调度/磁标定等命令的入口）',
        'cmds': [
            _c('dbg_enter', '进入固件 DBG 文本台', reply='dbg_mode', readonly=False),
            _c('dbg_exit', '退出 DBG 回到遥测', reply='dbg_mode', readonly=False),
            _c('dbg_cmd', '向 DBG 发一条命令行（如 health / tasks / imuraw）',
               [_a('line', 's', '整行命令', True)], readonly=False),
        ],
    },
]

# 额外能力：不是 WS 命令，但同样是"agent 可调用"的入口（HTTP 面）
HTTP_ENDPOINTS = [
    {'method': 'GET', 'path': '/api/caps', 'desc': '本清单（能力自描述）', 'readonly': True},
    {'method': 'GET', 'path': '/api/status', 'desc': '后端运行状态与链路统计', 'readonly': True},
    {'method': 'GET', 'path': '/api/snap', 'desc': '当前遥测快照（一次性 JSON，不订阅）', 'readonly': True},
    {'method': 'POST', 'path': '/api/cmd', 'desc': '执行任意 WS 命令并同步返回其回复消息', 'readonly': None},
    {'method': 'GET', 'path': '/download/{name}', 'desc': '下载 output/ 下的 CSV', 'readonly': True},
]


def all_cmds() -> dict[str, dict]:
    """@brief 命令名 → 命令描述（跨模块合并）"""
    out: dict[str, dict] = {}
    for m in MODULES:
        for c in m['cmds']:
            out[c['cmd']] = {**c, 'module': m['id']}
    return out


def module_of(cmd: str) -> str | None:
    """@brief 查某命令所属模块 id"""
    return all_cmds().get(cmd, {}).get('module')


def as_dict() -> dict:
    """@brief 完整能力清单（HTTP /api/caps 的返回体）"""
    return {
        'modules': MODULES,
        'http': HTTP_ENDPOINTS,
        'n_modules': len(MODULES),
        'n_cmds': len(all_cmds()),
    }


def source_cmds(main_py: pathlib.Path | None = None) -> set[str]:
    """@brief 从 main.py 的实际分派里抽出命令名集合（用于一致性校验）
    @details 只认 `elif cmd == 'x':` 这一种写法——分派是 if/elif 链，
             所以正则就是判据本身；将来改成注册表后这里也要跟着改，
             那时校验改为读注册表即可（清单与实现同源，校验自动消失）。
    """
    p = main_py or (pathlib.Path(__file__).parent / 'main.py')
    src = p.read_text(encoding='utf-8')
    # 分派链首个分支是 `if cmd == "，其余是 elif——两种都要认
    return set(re.findall(r"(?:if|elif) cmd == '([a-z_0-9]+)':", src))


def verify_against_source(main_py: pathlib.Path | None = None) -> list[str]:
    """@brief 比对「清单声明」与「实际实现」，返回问题列表（空=一致）
    @return 每条问题一句中文，直接可以打给用户看
    """
    declared = set(all_cmds())
    actual = source_cmds(main_py)
    problems = []
    for c in sorted(declared - actual):
        problems.append(f'清单声明了 {c}，但 main.py 里没有对应分派（写了没实现）')
    for c in sorted(actual - declared):
        problems.append(f'main.py 实现了 {c}，但能力清单没登记（实现了没登记）')
    # 参数名也要对：清单里的参数必须真的被该分支用到，否则调用方会传无效参数
    p = main_py or (pathlib.Path(__file__).parent / 'main.py')
    src = p.read_text(encoding='utf-8')
    # 一次性把所有分支切成 {命令名: 分支体}：
    # 分派链首个分支是 if、其余是 elif，逐个 split 会因为写法不同而漏掉第一个。
    parts = re.split(r"\n    (?:if|elif) cmd == '([a-z_0-9]+)':", src)
    branches = {parts[i]: parts[i + 1] for i in range(1, len(parts), 2)}
    for m in MODULES:
        for c in m['cmds']:
            if c['cmd'] not in actual or c['cmd'] not in branches:
                continue
            seg = re.split(r"\n    (?:if|elif) cmd == '|\n    else:|\nasync def |\ndef |\n@app\.",
                           branches[c['cmd']])[0]
            used = set(re.findall(r"msg\.get\('([a-z_0-9]+)'", seg))
            # 参数也可以经辅助函数间接取：_param_id_of(msg) 会读 name/id，
            # 只按字面 msg.get 判会误报（param_read/param_write 就是这样）。
            if '_param_id_of(msg)' in seg:
                used |= {'name', 'id'}
            for a in c['args']:
                if a['name'] not in used:
                    problems.append(
                        f"{c['cmd']} 声明了参数 {a['name']}，但该分支从未读取它")
    return problems
