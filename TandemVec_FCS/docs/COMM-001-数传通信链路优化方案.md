# COMM-001 — 数传通信链路优化方案（2026-08-10）

> 状态：**已实施完成并烧录（2026-08-10）**。A/B/C 三层全部落地，实机验证：921600 + 双环频率分布精确符合设计（快环 100Hz/位置 50Hz/RC 25Hz/显示 25Hz/GPS·氧压 10Hz），零坏帧；test_host ap 套件 5 组断言 + pytest 35 passed；固件编译 SUCCESS + Verified OK。
> 目标：提高稳定性（降丢帧）、降低延迟（端到端）、提高效率（打包 CPU + 带宽）。
> 硬约束：**官方 Ano 上位机在用——帧格式/功能码/字段缩放严格不变，只改发送频率与内部实现**。
> 随附改动（当时口径）：ano_vars.cpp tx[19]→tx[20] 越界修复（-Warray-bounds 消除）；PID 固化 att 2.8/2.8/5.0 + rate_yaw 0.20（迁移前混合域实机稳定值）。2026-08-11 行为保持迁移后，等价值为 rate_yaw `11.459156 s⁻¹`，0x21 帧仍发送 `rad/s²`，协议不变。

## 1. 背景与现状问题

| # | 问题 | 证据 |
|---|------|------|
| P1 | 2M 波特率在 DAP-Link VCP 上丢帧 ~15% | 任务调度统计实测（2026-08-09）；GCS 链路统计 |
| P2 | `sendData` 无长度上限检查，`len>256` 栈溢出隐患 | `lib/AnoComProtocol/AnoComProtocol.cpp` sendData |
| P3 | 打包低效：每帧 = memcpy + 2 次校验遍历 + 1 次 write；组2 五帧 = 5 次 write | 同上 + `communication.cpp` 组发送 |
| P4 | 全量遥测 50Hz 均等轮发——快变字段（姿态/控制/执行器）刷新率不够，慢变字段（GPS/氧压）发重复旧值 | `communication.cpp` group_index 4 组轮转 |
| P5 | 数据快照 20ms 采集一次，组3 发送时数据已陈旧 15ms | `new_cycle_data_collection` 逻辑 |
| P6 | GCS `read(1024)` 攒批延迟 ~68ms（15KB/s 流量下等满 1024B 才回调） | `serial_link.py` |
| P7 | 前端推送 20Hz 节流，快环提频后显示端仍只有 20Hz 刷新 | `main.py TELEMETRY_PERIOD=0.05` |

**带宽账本（现状）**：全量 14 帧 ≈ 300B/20ms = 15KB/s ≈ 60kbps @ 2M，利用率 ~7.5%。

## 2. 决策记录（grilling 逐项确认）

| 决策 | 结论 |
|------|------|
| D1 波特率 | **2M → 921600**（数传模块支持；固件其他串口全是 921600 且稳定） |
| D2 范围 | **A 打包层 + B 节奏层 + C 链路层全做** |
| D3 快环频率 | **100Hz**（200Hz 与前端 20-50Hz 推送无差异，带宽最小） |
| D4 慢环频率 | 位置 50Hz（快速飞行分析）、RC 25Hz、GPS/氧压 10Hz、其余显示帧 25Hz |
| D5 GCS 参数 | **read_size 1024→256 + 推送 20Hz→50Hz 全改** |
| D6 兼容性 | 官方上位机在用 → **帧格式/缩放零改动**，烧录后做兼容验证 |

## 3. 目标频率表（D3+D4）

| 帧 | 频率 | 帧字节(含8B开销) | 带宽 |
|----|------|------------------|------|
| 快环：0x01 IMU + 0x03 欧拉 + 0x21 控制量 + 0xF1 执行器 | **100Hz** | 72B/10ms | 7.2KB/s |
| 0x08 位置 | 50Hz | 20B/20ms | 1.0KB/s |
| 0x05 高度 + 0x06 模式 + 0x07 速度 + 0x20 PWM + 0x40 RC + 0x0A 目标姿态 + 0x0B 目标速度 | 25Hz | ~114B/40ms | 2.85KB/s |
| 0x30 GPS + 0x0D 电压/氧压 | 10Hz | ~49B/100ms | 0.5KB/s |
| **合计** | — | — | **≈ 11.5KB/s ≈ 92kbps** |

921600 利用率 ≈ **12.5%**（余量巨大，未来扩帧/提频无压力）。

**实现结构**：AnoCom 任务保持 200Hz（5ms tick）不动——每 2 tick 发一次快环帧组，慢环帧按各自周期在剩余 tick 穿插。调度器零改动。

## 4. 分层改动明细

### A. 打包层（`lib/AnoComProtocol/` + `communication.cpp`）

**A1. sendData 长度保护（修复隐患）**
- `AnoComProtocol.cpp sendData`：`if (len > ANO_MAX_DATA_LEN) len = ANO_MAX_DATA_LEN;`
- 同时给 `receiveData` 的 `_rxBuffer[4]|_rxBuffer[5]` 长度解析加同样上限（防恶意/损坏帧超限写）。

**A2. 校验合并单循环 + 边拷边算**
- `calculateSumCheck` + `calculateAddCheck` 两次独立遍历 → 单循环同时累加 sum 与 add；
- memcpy 并入循环（`sum += b; add += sum;` 边拷边算）。
- 帧小（<100B）绝对收益有限，但 700 帧/s × 3 遍历 → 1 遍历，打包热路径减负明确。

**A3. 组内多帧合并单次 write**
- `AnoComProtocol` 新增 `buildFrame(buf, off, dest, func, data, len)` 返回帧长（与 sendData 同组装逻辑）；
- `sendData` 保持接口不变（改为调 buildFrame + write）；
- `communication.cpp` 每组拼帧到本地缓冲（快环 ~72B / 慢环 ~114B，栈上 256B 足够），**一次 write**。
- 收益：write 调用 5→1，写竞争窗口缩小（2M/921600 下 100B 组 = 0.5/1.1ms 连续传输，不易被上行/中断打散）。

### B. 节奏层（`communication.cpp` handleAnoCom）

**B1. 双环结构替换 group_index 4 组轮转**
- 快环：每 2 tick（100Hz）采集并发送 0x01/0x03/0x21/0xF1；
- 慢环：按频率表分档计数器轮转（位置 50Hz 即每 4 tick 一次，25Hz 每 8 tick，10Hz 每 20 tick——统一"tick 计数器取模"实现，无额外调度器）。
- 0xF2 AnoVars 独立节流逻辑不动（已有 1-200Hz watch）。

**B2. 快变字段每 tick 采集（新鲜度 20ms→5ms）**
- 快环字段（姿态/控制量/执行器）从 `new_cycle_data_collection` 20ms 快照改为**每 tick 采集**；
- 慢环字段保持分档采集（GPS/氧压 10Hz 采集即可，位置每 20ms）。
- 陈旧度：组3 旧逻辑 15ms → 新逻辑快环 0ms / 慢环 ≤ 其周期。

### C. 链路层（GCS）

| 文件 | 改动 |
|------|------|
| `src/state_data.h:275` | `SERIAL6_BAUDRATE 2000000 → 921600`（唯一权威源） |
| `GCS/server/cli.py:42` | `DEFAULT_BAUD = 2000000 → 921600` |
| `GCS/server/serial_link.py` | 默认 baud 921600；`read_size 1024→256`（攒批 68ms→17ms） |
| `GCS/server/main.py:52` | `TELEMETRY_PERIOD 0.05 → 0.02`（推送 20Hz→50Hz，跟上快环） |
| 前端 | 确认连接参数不硬编码波特率（如有则同步） |

## 5. 验证计划

| 步骤 | 手段 | 通过标准 |
|------|------|----------|
| V1 打包回归 | `test_host` 新增：帧长/校验和/长度保护边界（超长截断）用例 | 全绿 |
| V2 GCS 回归 | `pytest tests/ -q` | 31+ passed（含新用例） |
| V3 编译 | `pio run`（PLATFORMIO_BUILD_DIR 隔离） | SUCCESS + Verified OK |
| V4 帧率分布 | 烧录后 `cli.py --port COM10 sniff all -t 5` | 快环 100Hz、位置 50Hz、慢环 10/25Hz 计数符合 |
| V5 链路健康 | `cli.py --port COM10 link` | 对比 2M 基线（遥测帧率 ~700/s、成功率 5/5）；921600 下坏帧率显著下降 |
| V6 兼容验证 | 官方 Ano 上位机连接 | 姿态/通道/参数页显示正常（帧格式未变，预期直接通过） |
| V7 录制分析 | `cli.py record` + 快速摆动 | CSV 中姿态/执行器 100Hz 采样率可见 |

## 6. 红线核对（AGENTS.md 行为保持清单）

- ✅ 帧格式/功能码/字段缩放：**零改动**（A 层只改内部组装，B 层只改频率，C 层只改参数）
- ✅ 遥测轮发任务 200Hz 调度不变（双环在任务内实现）
- ✅ 控制律/四元数 RK4/差速公式/滤波：完全不触碰
- ✅ 0xF1 执行器帧（手册用户自定义帧）频率提至 100Hz——官方上位机忽略未知帧，无冲突

## 7. 回滚方案

- 波特率：`state_data.h` 一行改回 2000000（固件）+ `cli.py DEFAULT_BAUD`（GCS）
- 其余改动均 git 可回退；B 层双环若实测异常，恢复 group_index 4 组轮转版本即可（保留旧函数对比）

## 8. 实施顺序

1. A 层（AnoComProtocol.cpp + test_host 用例）→ 2. B 层（communication.cpp 双环）→ 3. C 层（三处波特率 + read_size + 推送节流）→ 4. V1/V2 回归 → 5. 烧录 → 6. V4-V7 实机验证
