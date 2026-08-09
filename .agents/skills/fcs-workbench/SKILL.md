---
name: fcs-workbench
description: 纵列双发矢量推力飞行器飞控联调工作台——固件参数在线读写（CLI/后端名字寻址）、DBG 诊断（任务统计/在线辨识）、黑匣子导出与超调分析、在线飞行数据记录。当用户要调参（改 kp/ki/kQ）、分析实测飞行数据（超调/震荡）、读黑匣子、查看任务调度或在线辨识结果、记录飞行遥测时使用。项目特有命令与踩坑见正文。
license: MIT
metadata:
  author: LShang001
  version: 1.0.0
compatibility: Requires Windows + Python 3.12 + 飞控固件（AnoCom 0xE0/0xE1 参数链路、DBG 控制台、W25N01GV 黑匣子）
---

# FCS 联调工作台（纵列双发矢量推力飞行器）

飞控联调的全套命令与流程。**串口唯一约束**：后端（GCS 页面/桌面窗口）与 CLI 都独占 COM 串口——两者互斥，用 CLI 时后端需停（反之亦然）。

## 前置

- 工作目录：`TandemVec_FCS/GCS`（CLI/bb_tool 相对此目录）
- Python：`py -3.12`
- 串口：默认 COM10 @ 2000000（波特率唯一权威源 = 固件 `state_data.h` 的 `SERIAL6_BAUDRATE`）
- 后端启动：`py -3.12 -m uvicorn server.main:app --port 8091`（后台运行；重启后端前先杀旧进程，见踩坑 #1）

## 核心命令

### 参数在线读写（优先 CLI，名字寻址）

```bash
# 读（含 id/名称/类型/分组）
py -3.12 server/cli.py --port COM10 param get rate_roll.kp
# 写（0x00 校验帧确认，成功打 ✓）
py -3.12 server/cli.py --port COM10 param set rate_roll.kp 0.28
# 列表 / 恢复默认（RAM 生效）
py -3.12 server/cli.py --port COM10 param list
py -3.12 server/cli.py --port COM10 param restore
```

- **必须按名字寻址，绝不猜参数 ID**（曾把 id=28 rate_roll.ki 当 rate_pitch.kp 写过，踩坑 #2）
- 名字→ID 映射 = `GCS/server/params.py::expected_names()` 下标，有守护测试锁定（rate_roll.kp=27、rate_pitch.kp=36、att_pitch.kp=9）
- 后端运行中也可写（名字寻址 WS）：`{"cmd":"param_write","name":"rate_roll.kp","value":0.28}`
- **在线写只改 RAM**——重启/断电回烧录值；确认好手感后必须同步改 `include/FlightCtrlParams.h` 并重烧固化（调参唯一固化入口，踩坑 #3）

### DBG 诊断（后端运行中，经 WS dbg_cmd）

| 命令 | 用途 |
|------|------|
| `tasks` | 任务调度统计（实际/名义 Hz、耗时、间隔抖动、迟到、CPU%）——打印后清零，连发两次得两个窗口 |
| `id` | 在线辨识：b=I名义/I实际、扰动 d、激励标志、建议 Kp（角速度模式大幅方波激励后才更新） |
| `gpsproto` | GNSS 解析协议状态（kUbx/kNmea/kAuto + NMEA fix/sv/hdop/pdop/句分布/溢出/配置） |
| `gpsproto <ubx\|nmea\|auto> [baud]` | 切换解析协议（可选带波特率重设） |
| `nmea` | NMEA 备用链路详情（位置/速度/PDOP） |
| `ubxcfg` / `ubxcfg save` / `ubxcfg nmea` | UBX 接收机自动配置（重跑 / 固化 / UBX+NMEA 混合输出） |
| `ubxcfg status` / `ubxcfg rst` | 上次配置结果 / GNSS 软复位 |
| `ubxcfg msg <句> <rate\|off>` | 接收机消息速率（NMEA 句 gga/rmc/gsa/gsv/vtg/gll/zda/gns/gst/... + UBX 诊断消息 pvt/eoe/dop/sat/status/svin/timegps） |
| `ubxcfg core <on\|off>` | GGA+RMC+GSA 一键开关（最小 NMEA 集） |
| `ubxcfg nav5 <dyn 2\|4\|fix 0\|1\|2\|elev 0-90\|pdop 0-100>` | 导航引擎：动态模型/定位模式/最低仰角/pDOP 门限（2026-08-09 修复 mask 错位，此前 Airborne 从未生效） |
| `ubxcfg itfm <on\|off>` | CW 干扰检测（城市/图传频段干扰导致漂移时诊断） |
| `ubxcfg ant` | 天线状态查询（短路/开路，CFG-ANT） |
| `ubxcfg nmver <23\|40\|41\|410\|411>` | NMEA 版本（CFG-NMEA，北斗/伽利略需 4.10+） |
| `ubxcfg nmtalker <gp\|gl\|gn\|ga\|gb\|none>` | 主 talker ID（多星座组合解=GN） |
| `ubxcfg nmfilter <on\|off>` | NMEA 输出滤波（变化才输出） |
| `ubxcfg proto <ubx\|nmea\|both>` | 串口输出协议掩码（接收机侧独立切换） |
| `ver` | 固件版本/时钟 |
| `ws <r> <g> <b>` / `wsseq` / `wsstat` / `wsmode` / `wsfault` | WS2812 灯效测试/驱动状态 |
| `gpio` / `tim4` | GPIO 寄存器 / TIM4 DMA 状态 |
| `flash export` / `datalog <secs>` | 黑匣子导出 / 强制记录 |

进 DBG 模式遥测会暂停（互斥），诊断完必须发 `exit`（后端 `dbg_exit`）恢复。

### GNSS 双协议联调（2026-08-09 库级）

固件默认 `ubx.SetProtocol(kAuto)`（UBX 优先 + 失效兜底 NMEA）。GNSS 出问题时的排查顺序：

```bash
gpsproto                 # 1) 解析侧状态：protocol/fix/sv/pdop/句分布
ubxcfg status            # 2) 接收机配置结果（prt/rate/msg/nav5/gnss/sbas/verify）
ubxcfg core on           # 3) 接收机开 NMEA 输出（GGA+RMC+GSA @1Hz，双协议备份）
ubxcfg proto both        #    （或单独看纯 NMEA 流：ubxcfg proto nmea + gpsproto nmea）
```

- **kAuto 双保险场景**：UBX 配置失败/非 u-blox 模块 → `ubxcfg proto nmea` + `gpsproto nmea 38400` 转纯 NMEA（波特率按模块实际）
- **NMEA 流验证**：`nmea` 看位置/速度是否刷新；`gpsproto` 的 `sent(gga/rmc/gsa/unk)` 分布看句子是否在收、`bad_ck` 看波特率/接线
- **DETA100 模式无 NMEA**：Serial4 被 DETA100 独占，`ubx.Pump` 不运行、NMEA tap 自动失效（正常）
- 接收机侧配置只改 RAM——固化用 `ubxcfg save`

### 黑匣子（`server/bb_tool.py`，经后端导出链路）

```bash
py -3.12 server/bb_tool.py --list              # 列出飞行段（需后端运行 + 串口已连）
py -3.12 server/bb_tool.py --latest --analyze  # 导出最新段 → output/blackbox_segN.csv + 超调分析
py -3.12 server/bb_tool.py --seg 2 --pages 512 # 指定段/页数
```

- 默认 512 页（≈8 分钟飞行数据，约 40s 导出）；**全量 2048 页约 2.5-3 分钟**（2026-08-09 分片优化后，旧版 8-15 分钟）
- 通道自描述（S 帧随段头写入），当前 13 核心通道：姿态/角速率/摆角指令(tvc1/2)/差速 dw/油门/目标姿态
- `--analyze` 检测 roll/pitch 阶跃-保持超调（≥3° 阶跃）；**超调分析需要角度模式飞行数据**（角速度模式无姿态阶跃，踩坑 #5）

### 在线飞行数据记录

```bash
# 飞一圈自动存（20Hz 遥测 CSV，兜底用）
py -3.12 server/cli.py --port COM10 record start -d 90 -o output/flight1.csv
```

- 20Hz 是 WS 节流上限——**只够看趋势/兜底，量超调必须用黑匣子 200Hz**（踩坑 #6）

## 典型工作流

### 1. 调参闭环（推荐顺序）

```
改参 → 飞 → 数据 → 分析 → 固化
1. CLI 名字寻址写参数（或改 FlightCtrlParams.h 烧录）
2. 用户试飞（记录：cli record -d 90 兜底）
3. 落地后 bb_tool --latest 导出黑匣子
4. 分析（超调/震荡/响应）→ 结论
5. 手感确认后同步固化 FlightCtrlParams.h（RAM 值会丢！）
```

### 2. 超调实测

角度模式（CH9 < 1500）阶跃 10-20°、保持 2s × 每轴 3-5 次 → `bb_tool --latest --analyze` → 读超调百分比。理论参考：串级 ζ = ½√(ω_r/kp_a)，内环 kp=0.28 时 ~0.9（几乎不超调）。

### 3. 震荡定位

黑匣子导出 → 看 gyro 曲线找振荡段 → 量频率：高频（>2Hz）嫌疑滤波/采样，低频（<1Hz）嫌疑执行器滞后。差速通道另查：抖油门震荡 = 有效增益 kp·(w0_实际/w0_分配)² 的瞬态失配（τm 观测器已修复，改分配相关代码前先读 flight_control.cpp 层2 注释）。

### 4. 在线辨识标定

角速度模式大幅方波激励（α RMS > 2 rad/s² 才更新，每轴 15-20s）→ 落地 → DBG `id` → b 值收敛后按建议 Kp 调整。**b 是激励频率下的集总测量（含执行器滞后衰减），建议值只能当方向不能当数值**（曾按 0.55 建议调导致震荡，踩坑 #7）。

## 踩坑记录（最高信号）

1. **改后端/固件后必须杀旧进程再启动**：8091 旧 uvicorn 残留 = "界面全新、逻辑全旧"。查端口：`netstat -ano | grep :8091`
2. **参数 ID 必须名字寻址**：params.py::expected_names() 下标即 ID；猜 ID 会写错参数（rate_roll.ki 事故 2026-08-09）
3. **在线写参数只改 RAM**：重启回默认；固化唯一入口 = include/FlightCtrlParams.h（kFlightCtrlParamsDefaults）
4. **黑匣子全量导出很慢**（2048 页 8-15 分钟）：默认 512 页先看；导出中断会卡 DbgSession（已修复 enter/exit 复位，但别依赖）
5. **超调分析要角度模式数据**：角速度模式姿态不受指令约束
6. **在线 20Hz 量不了超调**：黑匣子 200Hz 是定量分析硬需求；在线记录只做兜底/趋势
7. **在线辨识 b 是集总测量**（惯量+执行器滞后混合）：不可反推 I_实际=I_名义/b 直接改惯量；建议 Kp 取当前值与建议值之间的保守值
8. **复验编译用隔离构建目录**：`PLATFORMIO_BUILD_DIR=".pio/build-tvc" pio run`（多 Agent 并发冲突）——**目录路径必须纯 ASCII**（`D:/pio-build-nmea`），中文路径会让 ld.exe `cannot open output file`（对象编译正常、仅链接失败，2026-08-09）
9. **黑匣子 tvc2 是摆角指令**（2026-08-09 后）：旧的 SERVO6 角度传感器反馈通道已弃（死通道）
10. **串口互斥**：CLI 与后端不能同时开 COM；CLI 操作完或后端重启都要先确认端口释放
11. **数字滤波相位滞后是裕度杀手**（2026-08-09 实测）：本机有物理减震底座，振动已被隔离——滤波只加滞后（4.2°@1Hz 稳 / 6.9° 振铃）。当前滤波近全关（α1=0.4/二级直通/输出 0.9）。调滤波前先确认振动源是否物理隔离，噪声收益要量化后再换滞后
12. **GNSS NMEA 解析三坑**（2026-08-09 库级集成）：① 解析器用 minmea（MIT）——MicroNMEA 是 LGPL 2.1（商业闭源静态链接有传染约束）；② u-blox 真实 RMC 尾部 `,,A` 会让官方 `minmea_parse_rmc` 失败（库内已自实现）；③ `gpsproto` 里 `sent(gga/rmc/gsa)` 分布全 0 = 波特率不对或接收机没开 NMEA 输出（`ubxcfg core on`）

## 数据源选择

- **定量分析（超调/震荡/增益验证）→ 黑匣子 200Hz**
- **实时监视/手感对应 → 在线遥测（20Hz）**
- 两者互补：飞行中 `cli record -d` 兜底 + 落地后 `bb_tool` 精分析
