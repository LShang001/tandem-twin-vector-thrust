# TandemVec GCS — 纵列双发矢量推力飞行器地面站

实机飞控（TandemVec_FCS）配套上位机：Python 后端 + Web 前端，串口走 **AnoCom 匿名地面站协议**（与官方 Ano 上位机生态兼容，同一飞控两套上位机可换用）。

## 快速开始

```bash
# 依赖（首次）
py -3.12 -m pip install -r requirements.txt

# 启动（start.bat 等价：自动开浏览器 http://127.0.0.1:8091/）
cd TandemVec_FCS/GCS
py -3.12 -m uvicorn server.main:app --host 127.0.0.1 --port 8091
```

## 功能一览

| 页面 | 功能 |
|------|------|
| 仪表盘 | 人工地平线（横滚/俯仰/航向）、HUD（速度/高度/油门/电压/氧压）、模式/解锁/GPS/融合状态灯、RC 8 通道条、控制输出条、GNSS 详情、告警条（低压/星数/解锁变化） |
| 3D 姿态 | Three.js 纵列双发构型简模（前摆座绕 z / 尾摆座绕 y / 差速反桨），姿态同步 + 旋翼转速动画 |
| 实时曲线 | 8 组预设通道（姿态/角速率/加速度/速度/位置/控制量/TVC/压力），滚动窗口 20s、暂停/清空 |
| 黑匣子 | `flash stat/findseg/export` 飞行段列表 → 段导出 → 表格预览 + 分组绘图（姿态/加速度/角速率/速度/水平轨迹/TVC）+ CSV 导出 |
| 参数 | AnoCom 0xE0/0xE1 在线读写 117 参数（12 PID 环 × 9 字段 + 9 滤波 alpha），名称/类型自 E2 信息帧，分组/范围本地元数据，校验帧确认写入，恢复默认 |
| 控制台 | DBG 交互终端（`help/flash*/datalog/ws*/ver`）+ 实时遥测 CSV 记录 + 回放 |

## 连接与模式

- **串口**：Serial6 地面站口（固件 `SERIAL6_BAUDRATE`，默认 **2 000 000**；数传模块换波特率后在此改）
- **两种模式互斥**（固件行为）：
  - **遥测模式**（默认）：AnoCom 4 组轮发（IMU/姿态/高度/模式/速度/PWM/位置/压力/GPS），200Hz 组轮 → 各帧约 50Hz
  - **调试模式**（DBG）：发 `DBG\n` 进入，遥测暂停，控制台/黑匣子命令接管；`exit` 退出恢复遥测
- 黑匣子流程自动进入 DBG 模式（后端 `_dbg_ensure` 兜底），退出后自动恢复遥测

## 协议要点（与固件逐字节对齐，见 `server/anocom.py`）

- 帧：`AB 05 FF | func | len LE | data | SC | AC`（SC=逐字节和，AC=累加和）
- 缩放：姿态 int16×100；加速度 cm/s²；陀螺 ÷16.384；速度/位置 ×100；经纬度 ×1e7
- **本工程占用约定**：0x0D 帧 `fc_voltage/fc_current` = 氧压 P1/P2（`bat_voltage=12.6` 占位无真实采样）；0x20 传 `raw_rc_values`（us，非 0.01%）；0x07/0x08 向下取反（向上为正）
- 参数读写：0xE0 CMD 0x01 参数个数 / 0x02 读值 / 0x03 读信息（E2）/ 0x10 恢复默认；0xE1 写入 → 0x00 校验帧（SC/AC 回传）确认

## 测试

```bash
cd TandemVec_FCS/GCS && py -3.12 -m pytest tests/ -q
```

覆盖：协议编解码（校验和/缩放/占用约定）、黑匣子 I/P/S/E 帧解析（差分还原/段切分/CRC）、参数序列化、遥测聚合链路、校验帧匹配、CSV 记录回放（FakeSerialLink，无硬件）。

## 目录结构

```
GCS/
├── server/
│   ├── main.py         # FastAPI 入口 + WS 分发 + 串口编排（遥测/DBG 双模式）
│   ├── serial_link.py  # 串口抽象（pyserial / FakeSerialLink 测试注入）
│   ├── anocom.py       # AnoCom 编解码（与固件逐字节对齐）
│   ├── blackbox.py     # W25N01GV 黑匣子解析 + DBG 会话（export 剥离文本行）
│   ├── params.py       # 参数显示元数据（按名称匹配固件注册表）
│   └── datalog.py      # CSV 记录 / 回放
├── web/                # 前端（深色玻璃拟态主题，three-r170 vendored）
├── tests/              # pytest（无硬件可跑）
├── start.bat
└── requirements.txt
```

## 与固件的配合

- 参数在线读写需固件 2026-08-08 后版本（`ano_params.cpp`，117 参数注册表）
- 黑匣子 S/E 帧（飞行段）需固件 d39f5c5 后版本
- 实机联调：数传或 USB-TTL 接 Serial6 → 2M 波特率 → 连接 → 遥测即现；参数写入前确认解锁状态（地面检查规程）

## 实机联调记录（2026-08-08，COM10 = WeAct 板载 DAP-Link VCP）

| 功能 | 状态 | 备注 |
|------|------|------|
| 遥测流（姿态/HUD/RC/GNSS） | ✅ | 2M 下 200Hz 轮发完整 |
| 设备探测（0xE3 + 参数个数） | ✅ | 117 参数在位 |
| 参数读取 117/117 | ✅ | 逐 ID 等待 + 补拉（2M 偶发丢命令 ~5%，重试吸收） |
| 参数写入 + 校验帧确认 + 恢复默认 | ✅ | att_roll.kp 2.5→2.6→读回→restore→2.5 全链路 |
| DBG 控制台（flash stat/findseg） | ✅ | 需先 `exit` 复位残留 DBG 状态 |
| 黑匣子分片导出 | ⚠️ | **DAP-Link VCP 2M 大流量（>8KB）丢字节**（4 页小片 80% 完整、32 页必丢）——固件/上位机逻辑均验证正确，是 VCP 虚拟串口带宽限制；**大导出请用数传模块 USB 口或独立 USB-TTL** |

**链路限制**：DAP-Link VCP 在 2M 下小流量（参数/文本）可靠、大流量（连续 2048B 页流）丢包率 ~5-20% 且不稳定。CLI/Web 端导出已实现 **4 页分片 + 长度校验 + 3 次重试**（`cli.py _flash_export` / `main.py _flash_export_chunked_async`），在可靠链路上自动完整，在 VCP 上尽力而为。

**固件侧配套修复（2026-08-08 烧录生效）**：
- DBG 检测 peek 修复：AnoCom 上行帧不再被 DBG 检测吞掉（旧固件参数读写从未真正生效的根因）
- export 循环分块写 + 持续喂狗：2M 满速发送不再触发 IWDG 复位
- P 帧 21 增量修正（p2 通道曾被 CRC 污染）
