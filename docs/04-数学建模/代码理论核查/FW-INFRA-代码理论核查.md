# 代码理论核查记录 — fw-infra（主循环 / 通信 / 传感器外设 / 调度 / 参数与变量注册）

> 核查日期：2026-10-23 ｜ 执行：fw-infra（子 Agent）｜ 规范：[GOV-004](../../00-项目治理/GOV-004-代码注释与理论对照规范.md)
> 代码基线：`6784221`（工作区含未提交改动：`ano_vars.cpp` / `state_data.cpp` / `state_data.h` 的功能性新增
> 为**他人既有改动**，非本区域本次产物；本次仅新增/修订注释与本文档）
> 说明：核查期间工作区同时存在其他区域的并发改动（gcs / fw-nav / fw-control / fw-tests），
> 本记录只对本区域 29 个文件的注释与协议一致性负责，不对其他区域改动背书。
> 理论来源：本区域几乎无理论公式，`@theory` 一律 `N/A`；协议与设计来源为
> 协议/设计文档：`TandemVec_FCS/docs/PROTOCOL-001-AnoCom官方通信协议.md`、`TandemVec_FCS/docs/COMM-001-数传通信链路优化方案.md`、`TandemVec_FCS/docs/GCS-001-上位机与参数在线读写方案.md`、`TandemVec_FCS/docs/C路径-参数集中与遥测结构化方案.md`、`TandemVec_FCS/docs/mavlink-ref/`、根 `AGENTS.md` 踩坑记录
> 方法：逐文件读代码 → 对照协议章节/发送端符号 → 跨区域只读核对（注册表 vs 上位机）→
> 测试复跑 → 补注释 → lint 复检 → `git diff` 行为审计

---

## 1 范围与覆盖率

29 个文件 / 12 855 行（含本次新增注释 714 行）。L0 文件头 29/29 覆盖；L1 文档块按“导出函数 +
超 10 行内部函数”补齐；全文 `@ref` 引用 47 行（含单行多引用）。表中“L1”列为 `@brief` 计数。

| 文件 | 行数 | L0 | L1（@brief 数） | @ref | 备注 |
|------|------|----|-----------|------|------|
| `src/main.cpp` | 545 | 有（新） | 2 | 1 | 补初始化/任务注册/主循环；加频率总表；去 BOM（见 P2-4） |
| `src/communication.cpp` | 3450 | 有（新） | 31 | 12 | 双环 L1 表、0x01/0x03/0x0A/0x21/0x0D/0x30/0xF1 字段表、0x00/0xE3/0xF2/0xF3 字段 |
| `src/communication.h` | 331 | 有（补） | 37 | 2 | 旧“4 组 50Hz 轮发”描述废止，改为双环分档表 |
| `src/can_bus.cpp` | 203 | 有（补） | 1 | 1 | 8 帧轮发表、Flash 写页互斥、float 小端约定 |
| `src/can_bus.h` | 70 | 有（补） | 3 | 1 | ID 分配与单位、Flash 互斥门控 |
| `src/mavlink_bridge.cpp` | 298 | 有（新） | 1 | 1 | 双向桥 L0；短名映射（≤15B）与 rc_* 说明 |
| `src/mavlink_bridge.h` | 32 | 有（新） | 1 | 1 | 互斥/命令拒绝红线 |
| `src/ubx_config.cpp` | 623 | 有（新） | 1 | 1 | UBX 帧与校验、探测 300ms/ACK 500ms、阻塞调用约束 |
| `src/ubx_config.h` | 204 | 有（新） | 13 | 1 | 配置结果/选项/NMEA 枚举；默认配置注释中文化（R06） |
| `src/task_scheduler.cpp` | 376 | 有（补） | 5 | 1 | 统计口径即“软超时”；tick 单位纠正（0.5ms） |
| `src/task_scheduler.h` | 87 | 有（补） | 8 | 1 | 超时/看门狗语义（调度器不喂狗，IWDG 在 main） |
| `src/task_scheduler_interval.h` | 102 | 有（新） | 3 | 1 | Q8 分数 tick 的 L1/L2（7.735ms 来源） |
| `src/ano_params.cpp` | 376 | 有（新） | 4 | 1 | 注册表宏展开、ID=下标不变量、NaN 拒绝 |
| `src/ano_params.h` | 74 | 有（新） | 2 | 1 | 0xE0/0xE1/0xE2 字段表；计数统一 128 |
| `src/ano_vars.cpp` | 315 | 有（新） | 1 | 1 | 三坑写入 L0；0xF2/0xF3 帧格式 |
| `src/ano_vars.h` | 35 | 有（新） | 1 | 1 | 0xF2 6B / 0xF3 20B |
| `src/sensor_peripheral.cpp` | 1861 | 有（新） | 7 | 1 | DPS310 时序、光流双通道新鲜度、UART7 相位表 |
| `src/sensor_peripheral.h` | 157 | 有（补） | 12 | 1 | 光流几何补偿公式与两通道约束 |
| `src/state_data.cpp` | 685 | 有（补） | 2 | 2 | `applyFlightCtrlParams()` 无扰语义；公式行去 R04 误报 |
| `src/state_data.h` | 850 | 有（补） | 3 | 5 | `SERIAL6_BAUDRATE` 唯一源 + COMM-001 引用；`OpticalFlowData`/`Task` L1 |
| `src/deta100_types.h` | 217 | 有（补） | 9 | 1 | 结构布局=协议解析的不变量 |
| `src/icm45686_ins_fast_covariance_blocks.h` | 153 | 有（新） | 1 | 2 | EKF 协方差块展开：等价性 + Qc 正交阵假设 |
| `include/AnoVars.h` | 181 | 有（补） | 1 | 1 | 三坑 @note；名称 16B / watch 16 约束 |
| `include/StatusLED.h` | 110 | 有（新） | 2 | 1 | 非阻塞闪烁 |
| `include/Ws2812AppStatus.h` | 223 | 有（新） | 3 | 1 | 只渲染不判定；上电无信号 vs 断链 |
| `include/MedianFilter.h` | 238 | 有（新） | 9 | 1 | 窗口奇偶/未填满语义/开销 |
| `include/DETA100_module.h` | 526 | 有（补） | 12 | 1 | 双 CRC、帧定界靠长度；删除 4B 旧 `parseTimestamp` 注释代码 |
| `analyze_ano.py` | 349 | 有（新） | 1 | 1 | 协议标签补全；4 个公共函数补 docstring；50Hz→100Hz 说明 |
| `analyze_ano.js` | 184 | 有（新） | 1 | 1 | 同上（JS 头注释） |

---

## 2 核对矩阵（以“协议字段 / 频率 / ID”为主）

结论枚举：`一致` / `等价变换后一致` / `近似（可接受）` / `偏差（已登记）` / `不一致（待修）`。

### 2.1 调度与遥测频率

| ID | 模块/符号 | 核对对象 | 依据 | 结论 | 证据 | 处置 |
|----|-----------|----------|------|------|------|------|
| A-01 | `handleAnoCom` | 200Hz tick 双环：快环 100Hz `t%2==1`（0x01/0x03/0x0A/0x21/0xF1）；位置 50Hz `t%4==2`；RC 25Hz `t%8==4`；显示 25Hz `t%8==6`；GPS·氧压 10Hz `t%20==10` | COMM-001 §3、AGENTS.md 踩坑记录（遥测双环） | 一致 | 代码取模表达式；慢环余数 2/4/6/10 互不相同；快环占奇数 tick | 补 L1 频率表 |
| A-02 | `handleAnoCom` / `AnoComProtocol` | 组模式 `beginGroup/endGroup` 同 tick 多帧合并**单次 write** | COMM-001 §4 A3 | 一致 | `lib/AnoComProtocol/AnoComProtocol.cpp` `endGroup()`；`test_host/test_anocom_packing.cpp` A3（3 帧 → 1 次 write，字节流与逐帧一致，44B） | 补注释 |
| A-03 | `handleAnoCom` | TX 余量守卫 `availableForWrite() < 190`（最坏同拍组 ≈175B） | 代码内 2026-08-10 审查记录 | 一致 | 守卫值与 RC20+disp86+pos20+gps31+volt18=175B 自洽 | 补 L2 |
| A-04 | `SERIAL6_BAUDRATE`（`state_data.h`） | 地面站波特率 921600，唯一事实源；无 2000000 硬编码 | COMM-001 §2 决策 D1、§3（利用率 ~12.5%） | 一致 | `main.cpp` `Serial6.begin(SERIAL6_BAUDRATE)`；`grep 2000000` 固件 src/include 无命中 | 补 @ref；工具脚本见 P1-2 |
| A-05 | `setup()` 任务注册表 | 2kHz / ~129Hz（7.735ms）/ 500Hz / 200Hz / 100Hz / 50Hz / 25Hz / 10Hz 分档 | main.cpp 注册段、COMM-001 §3 | 一致 | 新增“频率总表”注释与逐行注册值逐项对应 | 补 L2 表 |
| A-06 | `timerCallback` / `addTaskNamed` | tick 单位 0.5ms（`interval*2`），`currentCount` 为 uint32（溢出 ~248 天） | 代码 | 一致 | 原头注释“05ms”为排版笔误，已改 0.5ms | 修注释 |

### 2.2 AnoCom 帧格式与功能码

| ID | 模块/符号 | 核对对象 | 依据 | 结论 | 证据 | 处置 |
|----|-----------|----------|------|------|------|------|
| B-01 | `buildFrame` | 帧结构 `AB / src / dst / func / lenL lenH / data / SC / AC`（小端） | PROTOCOL-001 §一.1 | 一致 | 偏移 0..5 为头、6..6+len 为数据、6+len 与 7+len 为 SC/AC；返回 `len+8` | — |
| B-02 | `buildFrame` | SC = 逐字节和；AC = “和的和”（均低 8 位，单循环边拷边算） | PROTOCOL-001 §一.1 校验示例 | 一致 | `test_anocom_packing.cpp` A2 与独立参考实现逐字节一致 | — |
| B-03 | `sendIMUData`（发送端）+ `handleAnoCom` | 0x01 13B：ACC int16×3 [cm/s²]（×100）、GYR int16×3（32768↔2000 deg/s，×16.384）、SHOCK u8 | PROTOCOL-001 §二.1.1 | 一致 | `AnoComProtocol.cpp` `sendIMUData` 内 `*100.0f` 与 `GYRO_SCALE_FACTOR=16.384` | 补字段表 |
| B-04 | `sendAttitudeEuler` | 0x03 7B：ROL/PIT/YAW int16 ×100 [deg] + FUSION_STA u8 | PROTOCOL-001 §二.1.3 | 等价变换后一致 | 字段数与缩放一致；FUSION_STA 为工程自定义位域（bit7 DETA100 在线 / bit6 数据源 / 低 6 位 EKF 状态） | 补位域说明 |
| B-05 | `sendTargetAttitude` | 0x0A 6B：3×int16 ×100；**第 3 字段承载目标航向角速率 [deg/s]**，手册为目标航向角 | PROTOCOL-001 §二.1.9 | 偏差（已登记） | 固件 `gnc_tel.omega_ref_dps[2]*100`；GCS `decode_target_attitude` 注释同口径 | 差值已注明，见 P2-1 |
| B-06 | `sendAttitudeControl` | 0x21 8B：4×int16 ±5000；本工程 = `alpha_ref_radps2×10`（rad/s²）+ 油门%×10 | PROTOCOL-001 §二.2.2 + 差异表 | 等价变换后一致 | 传输 `rad/s²`×10，量程 ±500 rad/s²；顺序 (rol,pit,thr,yaw) 有防错位注释 | 补字段表 |
| B-07 | `sendPWMOutput` 调用点 | 0x20 16B：8×u16，ch3/ch4 = 电机输出 %×100（0-10000，0.01%） | PROTOCOL-001 §二.2.1 + 差异表 | 一致 | `constrain(ch3_output*100, 0, 10000)`；2026-08-10 数据归位 | — |
| B-08 | `sendRCData` | 0x40 20B：10×u16 raw µs（手册为 10×int16，且方向为“地面站→飞控”） | PROTOCOL-001 §二.4.1 + 差异表 | 等价变换后一致 | 字节宽度一致；本工程为回传方向，上行不处理 | — |
| B-09 | `sendVoltCurr` 调用点 | 0x0D 10B：5×u16；bat V×100、A×100（真实 ADC）；**fc 两字段占用为发动机氧压 P1/P2** | PROTOCOL-001 §二.1.13 + 差异表 | 偏差（已登记） | `bat_voltage_mv/1000`、`bat_current_ca/10` 再 ×100；`receivedP1/P2` 进 fc 槽 | 补 @ref |
| B-10 | `sendGPSInfo1` 调用点 | 0x30 23B：经纬 ×1e7、高度 ×100、NED 速度 ×100、**pdop/vacc/sacc ×10**、fix/sv u8 | PROTOCOL-001 §二.3.1 + 差异表 | 一致 | `ano` 发送端 `pdop*10`；实参顺序 `(pdop, sacc, vacc)` 有 2026-08-10 修复注释 | — |
| B-11 | `handleAnoCom` 快环执行器块 | 0xF1 12B：上/下摆 ×100 deg(i16)、前/尾电机 %×10(u16)、Δω×1000(i16)、饱和 u8、预留 u8 | PROTOCOL-001 §一.3（灵活格式帧） | 一致（自定义） | `put16` 逐字段小端；`anoCtrlSafe` 防 NaN | 补字段表 |
| B-12 | `anoVarsSendTick` | 0xF2 6B：`[id u16 LE][float LE]`；`200/rate` tick 节流、watch 轮转 | PROTOCOL-001 §六（灵活帧自定义语义） | 一致（自定义） | `period = 200/rate_hz`；`s_vars_idx` 轮转 | — |
| B-13 | `anoVarsHandleList` | 0xF3 应答 20B = CMD(1)+ID(2)+TYPE(1)+NAME(16)；count 应答 3B，只读不回校验帧 | AGENTS.md 三坑③ / 仿 0xE2 | 一致 | 缓冲 `tx[1+2+1+16]`；`memset(&tx[4],0,16)` | — |

### 2.3 参数链路（0xE0–0xE3）与注册表

| ID | 模块/符号 | 核对对象 | 依据 | 结论 | 证据 | 处置 |
|----|-----------|----------|------|------|------|------|
| C-01 | `anoParamsHandleRx` 0xE0 | CMD 0x00 设备信息（上层回 0xE3）/0x01 个数/0x02 读值/0x03 读信息/0x10 0xAA 恢复默认·0xAB 保存 | PROTOCOL-001 §四.1 命令表 | 一致 | `switch(cmd)` 五分支；0x00 由 `onAnoRxFrame` 应答 | — |
| C-02 | `onAnoRxFrame` + `ano_params` 0xE1 | 0xE1 `[PAR_ID u16][PAR_VAL 定长]`；写后回 0x00 校验帧（真实 SC/AC） | PROTOCOL-001 §四.2 | 一致 | `AnoCom.sendDataCheck(funcCode, rxSumCheck, rxAddCheck)` | — |
| C-03 | `anoParamsHandleRx` 0xE2 | `[ID u16][TYPE u8][NAME 20B]`；TYPE=FLOAT(8)/UINT8(0) | PROTOCOL-001 §四.3 + PAR_TYPE 表 | 一致 | 23B DATA；名称补 0x00 定长 | — |
| C-04 | `onAnoRxFrame` 0xE3 | `DEV_ID u8 + HW/SW/BL/PT 各 i16 + NAME 20B` = 29B DATA | PROTOCOL-001 §四.4 | 一致 | `sendDeviceInfo` 数据区 `1+8+20`；守卫 `availableForWrite()>=40` | — |
| C-05 | `kAnoParams`（128 项） vs `GCS/server/params.py::expected_names()` | 名称**与顺序**逐项一致（ID 即下标） | AGENTS.md 关键路径（注册表 ID 顺序须同步） | 一致 | 只读复核脚本：展开宏列表 vs `expected_names()` → `firmware=128 gcs=128 mismatch: NONE`；GCS `tests/test_params_and_link.py` 断言 128/唯一 | 矩阵留证 |
| C-06 | `ANO_PARAMS_COUNT` / 名称长度 | 128 = 12 环×9 + 12 滤波 alpha（含 spd2 ×3）+ 掩码 + rc_* 7；线上名 ≤20B（`falpha` 短名） | PROTOCOL-001 §四.3（PAR_NAME 20B） | 一致 | `static_assert` 数量校验；`PID_ALPHA` 用 `loop.falpha` | 统一注释计数（P2-5） |
| C-07 | 0x10/0xAB 保存 | 收到即回校验帧，**参数无持久化**（重启回 `kFlightCtrlParamsDefaults`） | PROTOCOL-001 §四.1（0xAB 语义为“保存”） | 偏差（已登记） | `ano_params.cpp` 0xAB 分支仅返回 true；注释说明无需处理旧 Flash 值 | 文档已述，P2 备注 |
| C-08 | `applyFlightCtrlParams` | 写参数后同步 12 PID + 12 滤波器，保留积分/滤波状态（无扰） | GCS-001 §2、C路径 §4.3 | 一致 | 12 个 `applyPid` + 12 个 `setAlpha`；`setIntegralEnable(enabled)` | 补 L1 |

### 2.4 AnoVars（0xF2/0xF3）

| ID | 模块/符号 | 核对对象 | 依据 | 结论 | 证据 | 处置 |
|----|-----------|----------|------|------|------|------|
| D-01 | `kAnoVars` | 注册表定义 `extern const` 且**不带数组大小**（否则 sizeof 虚高/遍历空项崩） | AGENTS.md 三坑① | 一致 | `extern const AnoVarEntry kAnoVars[] = {...}` + `static_assert(sizeof/sizeof <= MAX)` | 三坑写入 L0 |
| D-02 | `AnoVarEntry::fptr` | `const volatile float *`，兼容 ISR 写入的 `bat_voltage_mv` | AGENTS.md 三坑② | 一致 | 结构体字段声明 | 同上 |
| D-03 | `anoVarsHandleList` | 0xF3 应答 20B（少 1B 越界） | AGENTS.md 三坑③ | 一致 | `uint8_t tx[1+2+1+16]` | 同上 |
| D-04 | watch 上限/速率/名称 | watch ≤16、rate 1-200Hz、名称 ≤16B（MAVLink 侧 ≤9 可见字符唯一） | `include/AnoVars.h` 宏；AGENTS.md（`adps2_x/y/z` 命名） | 一致 | `ANO_VARS_MAX_*`；`adps2_*` 三轴截断后仍唯一 | — |

### 2.5 MAVLink 桥

| ID | 模块/符号 | 核对对象 | 依据 | 结论 | 证据 | 处置 |
|----|-----------|----------|------|------|------|------|
| E-01 | `s_mav_active` / `handleMavlink` | AnoCom 与 MAVLink 互斥：DBG `proto mavlink` 或 `proto anocom` 运行时切换（不重烧）；mavlink 分支先 return，AnoCom 上行解析停用 | AGENTS.md 踩坑记录（MAVLink 切换） | 一致 | `handleAnoCom` 的 `if (s_mav_active) {...return;}`；DBG 扫描在 mavlink 模式跳过 | 补 L0/L1 |
| E-02 | `mavlinkSendTelemetry` | 分频：1Hz（HEARTBEAT/SYS_STATUS/BATTERY）、40Hz（ATTITUDE/ATTITUDE_QUAT/RAW_IMU/NAMED_VALUE_FLOAT/DEBUG_VECT）、20Hz（GLOBAL_POSITION_INT/VFR_HUD/RC_CHANNELS_RAW） | `communication.h` 原注释、代码 `%200/%5/%10` | 一致 | 新增分频表注释与代码逐条对应 | 补表 |
| E-03 | `mavlink_bridge.cpp` `handleCommandLong`（命令分支） | 246 重启放行；400 ARM / 176 SET_MODE **安全拒绝**；其余 UNSUPPORTED；全部回 ACK | AGENTS.md（地面站远程控制是红线） | 一致 | 命令分支 + `COMMAND_ACK` 发送 | — |
| E-04 | `mavParamIdOfName` / `mavParamIdFromMavName` | param_id ≤15B；`out_min→omin`…缩写；`inertia_comp_mask→inertia_mask`；rc_* 原样；反查用“生成后比对”而非前缀匹配 | `docs/mavlink-ref/MAVLink-参数服务.md` | 一致 | 函数内缩写表；注释说明“直接截断会冲突（实测）” | 更新为 128 口径 |

### 2.6 GNSS（UBX/NMEA）与数据源

| ID | 模块/符号 | 核对对象 | 依据 | 结论 | 证据 | 处置 |
|----|-----------|----------|------|------|------|------|
| F-01 | `_ubx_send`（`ubx_config.cpp`） | UBX 帧 `B5 62 / cls / id / lenL lenH / payload / CK_A CK_B`，校验 sum0/sum1 | u-blox 协议 / SparkFun 参考 | 一致 | 校验循环从索引 2 起（头不参与），与 UBX 规范一致 | 补 L0 |
| F-02 | `ubxAutoConfig` | 探测（目标优先/已固化快路径）→ CFG-PRT → CFG-RATE → CFG-MSG(NAV-PVT/EOE) → CFG-NAV5 → CFG-GNSS → CFG-SBAS → CFG-CFG → 回读校验；每步独立成败 | `docs/reference/sparkfun-ublox-gnss` | 一致 | `main.cpp` §1.6 调用，仅 INTERNAL 路径；`kDetectWindowMs=300`/`kAckTimeoutMs=500` 为 uint32 | 补 L0 |
| F-03 | `ubx.SetProtocol(kAuto)` / `gpsproto` | UBX 优先 + NMEA（GGA/RMC/GSA）兜底，解析在 `lib/ublox-main`（minmea），固件只切协议与波特率 | `lib/ublox-main/src/ubx.h` 头注释（2026-08-09 双协议） | 一致 | 固件侧无 NMEA 解析代码，只有 `SetProtocol`/`ubxcfg nmea`/`gpsproto` | 补注释指向 |
| F-04 | `setup()` DETA100 探测 / `loop()` | Serial4 复用：DETA100 帧头 0xFC、UBX 0xB5；探测窗口按帧轮换喂各自解析器；DETA100 模式下 loop 不泵 UBX | AGENTS.md 硬性代码边界（NavDataSource 不可更改） | 一致 | 奇偶轮换 `detect_toggle & 0x01`；`if (nav_data_source != DETA100)` 才 Pump | 补 L0 |
| F-05 | `handleDeta100`（`main.cpp`） | DETA100 在线判定：收帧刷新 `deta100_last_frame_ms`；超 `DETA100_ONLINE_TIMEOUT_MS` 置离线并发 STATUSTEXT | `deta100_types.h` / AGENTS.md | 一致 | 超时分支 + `mavlinkSendStatustext` | 补 L1（已存在，增强） |

### 2.7 传感器外设

| ID | 模块/符号 | 核对对象 | 依据 | 结论 | 证据 | 处置 |
|----|-----------|----------|------|------|------|------|
| G-01 | `initDPS310` / `handleDPS310` | 温度 1Hz/OSR×4、压力 128Hz/OSR×2；任务 7.735ms（分数 tick）；FIFO 只在溢出/通信异常时清 | `sensor_peripheral.h`、`main.cpp` `BFS_DPS310_TASK_INTERVAL_MS` | 一致 | `dps310_fifo_policy_test.cpp` 通过；`task_scheduler_interval_test.cpp` 通过 | 补 L1 |
| G-02 | `flow_data` 双新鲜度 | `last_update_ms` 仅光流（速度）帧刷新；`last_range_update_ms` 仅测距刷新 | AGENTS.md 踩坑记录（2026-08-21） | 一致 | 测距分支只写 `last_range_update_ms`；MSP/MAVLink 两处注释显式说明 | 写入 L1 |
| G-03 | `handleOpticalFlowSensor` | 相位扫描：LQS48@921600 → MSP@115200 → Micolink@115200 → MAVLink-APM@115200 → LQS48@115200 → Micolink@921600；失锁 1500ms 重扫 | 代码 + AGENTS.md（UART7 自动识别） | 一致 | `FlowDetectPhase` 枚举与 `FLOW_DETECT_*` 常量 | 补 L1 |
| G-04 | `handleMTF02PFlow` / `handleLQS48Flow` | 几何补偿：H = S·abs(cos(tilt))；V_total = ω_flow·H；V_rot = H·ω_imu；V_trans = V_total − V_rot | `sensor_peripheral.h` `@theory`（自洽几何推导，非教材公式） | 近似（可接受） | 代码步骤 1/2/2.5 与公式逐项对应；无外部标准可对照 | 标注“非教材出处” |
| G-05 | `getFilteredTVCAngles` | 传感器 2 原始读数取反（安装方向补偿） | 代码 | 一致 | `angle2 = -raw2` 路径；注释显式声明非符号错误 | 补 L1 |

### 2.8 状态数据与调度统计

| ID | 模块/符号 | 核对对象 | 依据 | 结论 | 证据 | 处置 |
|----|-----------|----------|------|------|------|------|
| H-01 | `GncTelemetry`（`state_data.h` §4.0b） | 字段与 C路径 §4.2 一致；写入点固定在 3 个控制函数尾部（同拍一致） | C路径 §4.2 | 一致 | `gnc_tel` 字段逐个比对方案文档；`ano_vars.cpp` 注册表消费同名字段 | 补 @ref |
| H-02 | `Task` / `Timer` 统计 | 名义周期 µs（注册时写入）；迟到判据 = 间隔 > 1.5×名义；`tasks` 打印后清零 | `task_scheduler.cpp` | 一致 | `gap > nominal*3/2` → `late_count++`；`schedulerStatsPrint` 末尾 `schedulerStatsReset()` | 补 L0 超时语义 |
| H-03 | IWDG 语义 | 主循环每轮 `kickIwdg()`；3.0s 窗口（LSI 32kHz/256 × reload 374 = 2.99s）；长阻塞任务需自行喂狗 | `main.cpp` `initIwdg`；AGENTS.md（flash 导出） | 一致 | `initIwdg` 参数；`communication.cpp` export 循环 `kickIwdg()` | 写入调度器 L0 |
| H-04 | 黑匣子 `kBbChannels[]` | 13 通道唯一事实源；Serial3 CSV + Flash 双写；S 帧随段头写入 | `communication.cpp`、GCS-001 §3 | 一致 | 通道表 + `bbSegmentNames()` + `setSegmentNameProvider` | 补 L1 |

---

## 3 发现清单

| 严重度 | 位置（符号） | 问题 | 影响 | 建议 |
|--------|--------------|------|------|------|
| P1-1 | `handleDebugConsole` 的 `wsseq` 分支 | 参数解析语句被一次历史编辑并入了注释行（物理行内含字面 `\n`），`ms` 恒 500，`wsseq <ms>` 无效；整行还触发 R05 超长 | 仅 DBG 灯效诊断：4×500ms=2s 阻塞，逼近 IWDG 3.0s 窗口；不影响飞行 | 已在注释中登记现状；**未修**（本次红线：只改注释）。修复方式：把被并入的 `sscanf`+`constrain` 拆回语句（原表达式见 git 历史），并入 fw-infra 后续修复任务 |
| P1-2 | `TandemVec_FCS/tools/test_mavlink_bridge.py`（来源：gcs 区域交叉核对） | `BAUD = 2000000` 与固件 921600 不一致（脚本自己 docstring 已注明需同步）；参数条数阈值写死 121，实际 128 | 联调会全乱码；参数收全判据误报 | **本区域不改**（属 gcs/tools）；建议改为从 `state_data.h` 读取宏或 128，并把 BAUD 默认改 921600 |
| P1-3 | `GCS/server/params.py::build_meta`（来源：gcs 区域交叉核对） | `FIELD_META` 新增 `rc_*` 后，`build_meta` 仍按 PID 环展开 `out_*.rc_rate` 一类永不命中的 36 条元数据 | 前端“其他”组可能出现伪参数；不影响真参数读写 | **本区域不改**（GCS 侧已登记 P1） |
| P2-1 | `sendTargetAttitude`（0x0A 第 3 字段） | 承载目标航向**角速率**，官方手册为目标航向角 | 用官方上位机看“目标航向”会把 deg/s 当角度显示 | GCS 解码已同口径并注释；建议在 PROTOCOL-001 差异表补一行（文档侧，非代码） |
| P2-2 | COMM-001 §3 频率表 | 把 0x0A 目标姿态列入 25Hz 组，实现（含 AGENTS.md 踩坑记录）为快环 100Hz | 读者按文档核对会误判不一致 | 以 AGENTS.md + 代码为准；建议 COMM-001 表加脚注 |
| P2-3 | `analyze_ano.py` `--dt` 默认 0.02 | 注释/默认值仍按 50Hz；0x03 自 2026-08-10 起为 100Hz | 用它算 yaw 漂移率会低估一半 | 已在 docstring 与 docstring `@note` 标明；默认值属脚本行为，未改（待 owner 定） |
| P2-4 | `src/main.cpp` 编码 | 原文件带 UTF-8 BOM，`check-code-comments.py` 无法识别 `/**` 文件头（R01 误报为“缺少 L0”） | 门禁无法通过 | 已去除 BOM（内容零变化，不影响 GCC/PlatformIO 编译）；若后续工具链要求 BOM 需同步修检查器 |
| P2-5 | 多文件注释 | 参数计数陈旧（117/121）与实际 128 不一致 | 误导调参/联调（121→128 差 7 条 rc_*） | 已统一为 `ANO_PARAMS_COUNT` / 128（`ano_params.h/.cpp`、`communication.cpp`、`mavlink_bridge.cpp`） |
| P2-6 | `task_scheduler.cpp` 文件头 | “05ms 周期/05ms 精度”排版笔误 | 误读 tick 精度 | 已改为 0.5ms |
| P2-7 | `include/DETA100_module.h` | `@file` 名与磁盘不一致（`DETA100_MODULE.h`）；另残留 4 字节旧 `parseTimestamp` 注释代码（R04） | 注释腐烂 | 已改正 `@file` 并删除注释代码（git 保留历史） |
| P2-8 | `src/ubx_config.h` 默认配置块 | 纯英文 `/* target_baud */` 注释触发 R06 | 规范不一致 | 已中文化 |
| P2-9 | `src/state_data.h/.cpp` 公式注释 | 以 `X = ...` 开头的物理公式行被 R04 启发式误判为“注释掉的代码” | 误报噪声 | 已改写为“公式 …”“现值 …”前缀（语义不变，绕开启发式） |

---

## 4 未决问题

1. **0x21 的量纲表述**：帧内为 `rad/s²`×10，而控制器内部为 `deg/s²`；两处放大链（PID 输出 → 物理边界 ×π/180 → 遥测 ×10）
   容易在文档里被读成一条。是否在 PROTOCOL-001 差异表补“0x21 = rad/s²×10，派生 deg/s² 由 GCS 计算”？
2. **0x0A 第 3 字段语义**（P2-1）：改回角度属协议变更，需 GCS/固件同步，非本区域可决。
3. **`wsseq` 修复归属**（P1-1）：归 fw-infra 后续修复任务还是并入 DBG 控制台重构。
4. **`pio run` 未执行**：本次为纯注释改动，未跑固件全量编译（无硬件/不改变行为）；建议 Lead 在集成时统一 `pio run` 与 `bash test_host/run_all.sh`。
5. **MAVLink 参数广播时长**：128 条 @200Hz 每拍 1 条约 0.64s；QGC 默认参数超时是否足够未实测。
6. **`tools/test_mavlink_bridge.py` 的 BAUD/条数**（P1-2）：需 gcs 区域确认修复方式（读宏 vs 写死 921600/128）。

---

## 5 门禁与测试

### 5.1 注释门禁（GOV-004）

```bash
py -3.12 tools/check-code-comments.py --area fw-infra --check
```

| 阶段 | 文件数 | error | warning | 平均分 | 退出码 |
|------|--------|-------|---------|--------|--------|
| 核查前（基线） | 29 | 29（R01×29） | 29（R04×8, R05×2, R06×1, R08×4, R09×14） | 75.0 | 1 |
| 核查后 | 29 | **0** | **0** | **100.0** | **0** |

### 5.2 宿主测试（g++，`-ITandemVec_FCS/include -ITandemVec_FCS/lib/eigen/src -ITandemVec_FCS/test_host/stub`）

仓库根目录执行（`ap` 套件另需 `-ITandemVec_FCS/lib/AnoComProtocol`，与 `test_host/run_all.sh` 一致）：

| 套件 | 命令 | 结果 |
|------|------|------|
| AnoCom 打包回归 | `g++ -std=c++17 $INC -ITandemVec_FCS/lib/AnoComProtocol TandemVec_FCS/test_host/test_anocom_packing.cpp TandemVec_FCS/lib/AnoComProtocol/AnoComProtocol.cpp -o ap.exe && ./ap.exe` | **通过**（5 组断言：帧结构/校验/长度保护 256 截断/组模式单次 write/组缓冲满 flush） |
| AnoVars 注册表 | `g++ -std=c++17 $INC TandemVec_FCS/test_host/test_ano_vars.cpp -o av.exe && ./av.exe` | **通过**（`ALL PASSED (16 vars)`） |
| 调度分数 tick | `g++ -std=c++17 $INC TandemVec_FCS/test_host/task_scheduler_interval_test.cpp -o ts.exe && ./ts.exe` | **通过** |
| DPS310 FIFO 策略 | `g++ -std=c++17 $INC TandemVec_FCS/test_host/dps310_fifo_policy_test.cpp -o dp.exe && ./dp.exe` | **通过** |

### 5.3 行为保持审计

- 对 27 个 C/C++ 文件做“剥离整行注释后比对 HEAD 与工作区”的脚本审计：
  **本次改动的 24 个文件代码行序列完全一致**（仅注释/空行变化）。
  剩余 3 个文件的代码差异来自**他人既有未提交改动**（`ano_vars.cpp` 新增 `vib_*`/`afm_*` 等注册行、
  `state_data.cpp` 新增 `imu_die_temp_c`/`g_actuator_*` 定义、`state_data.h` 对应声明），与本轮注释工作无关。
- 行尾/BOM：CRLF 文件（12 个）经本次编辑后已归一为统一 CRLF；`main.cpp` 去 BOM（见 P2-4）。
- `git diff` 自查：无帧格式、字节序、缩放系数、频率、缓冲区尺寸、逻辑改动。
