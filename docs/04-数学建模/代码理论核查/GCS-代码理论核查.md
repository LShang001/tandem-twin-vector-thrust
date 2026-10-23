# 代码理论核查记录 — 上位机 GCS 与工具（gcs）

> 核查日期：2026-10-23 ｜ 执行：gcs Agent ｜ 规范：[GOV-004](../../00-项目治理/GOV-004-代码注释与理论对照规范.md)
> 代码基线：main 工作区（含多 Agent 未提交改动；本记录只覆盖 `TandemVec_FCS/GCS/**` 与 `TandemVec_FCS/tools/**`）
> 协议来源：`TandemVec_FCS/docs/PROTOCOL-001-AnoCom官方通信协议.md`（帧格式/校验/遥测帧 DATA 布局/0xE0-0xE3 参数链路/本工程差异对照表）、`TandemVec_FCS/docs/COMM-001-数传通信链路优化方案.md`（双环频率、组模式、read_size、推送 50Hz）、`TandemVec_FCS/docs/GCS-001-上位机与参数在线读写方案.md`（参数在线读写、P 帧增量数）、`TandemVec_FCS/docs/C路径-参数集中与遥测结构化方案.md`（kFlightCtrlParams 集中与遥测结构化）、
> [CTL-001](../../05-控制与分配/CTL-001-控制架构与SAS.md)（SAS 速率环量纲、控制分配）、[MOD-002](../MOD-002-坐标系与符号约定.md)、AGENTS.md 踩坑记录
> 固件对照（只读，未修改）：`lib/AnoComProtocol/AnoComProtocol.cpp`、`src/communication.cpp`、`src/ano_params.cpp`、`lib/W25N01GV/src/W25N01GV_log.{h,cpp}`
> 方法：逐文件读代码 → 对照协议章节与固件发送/解析实现 → pytest 与机检复算 → 补 L0/L1/L2 → 机检复检

---

## 1 范围与覆盖率

| 子系统 | 文件数 | 行数（核查后） | L0 文件头 | L1 符号文档 | @ref |
|--------|-------:|---------------:|-----------|-------------|-----:|
| `GCS/server/**`（协议/链路/参数/黑匣子/CLI） | 10 | 3586 | 10/10 | 公共类与函数全覆盖（机检 R08=0） | 9 |
| `GCS/web/src/**`（前端 8 模块） | 8 | 1817 | 8/8 | 8 个导出函数文档块 + 局部函数注释 | 17 |
| `GCS/tests/**` | 4 | 688 | 4/4 | 测试名即语义（GOV-004 豁免 R08） | 3 |
| `GCS/app.py`（桌面启动器） | 1 | 206 | 1/1 | 3 个公共函数 | 1 |
| `TandemVec_FCS/tools/**`（核对/分析脚本） | 19 | 2132 | 19/19 | 19/19 | 34 |
| **合计** | **42** | **8429** | **42/42** | 机检 R08=0 | **64** |

- 机检基线（核查前）：42 error / 77 warning（R01=42、R08=47、R09=28、R04=2），平均分 70.8，0 个文件全绿。
- 核查后：**0 error / 0 warning，平均分 100.0，42/42 全绿**（命令见 §5）。
- 新增：L0 文件头 42 个（含 2 个空 `__init__.py` 包文档）、L1 文档块 47 个（R08 全消）、`@ref` 引用 63 处（本区现有 64 处）。
- 行为保持：`git diff -U0` 中删除行仅为旧 docstring/banner 注释，无代码语句增删（§5 自查）。

---

## 2 核对矩阵

> 代码位置一律用符号名（不写行号）。结论枚举：`一致` / `等价变换后一致` / `近似（可接受）` / `偏差（已登记）` / `不一致（待修）`。

### 2.1 AnoCom 帧格式与校验（`anocom.py` ↔ 固件 `AnoComProtocol`）

| ID | 要素 | 协议出处 | PC 实现（符号） | 固件实现（符号） | 结论 | 证据 |
|----|------|----------|------------------|-------------------|------|------|
| A-01 | 帧结构：`AB` + S_ADDR + D_ADDR + ID + LEN(u16 LE) + DATA + SC + AC | PROTOCOL-001 §1「通信帧格式介绍」 | `encode_frame`、`extract_frames` | `AnoComProtocol::buildFrame`（头 6B，帧尾 `buf[6+len]`=SC、`buf[7+len]`=AC） | 一致 | `tests/test_anocom.py::test_frame_checksum_reference` 逐字节断言头/长度/载荷/校验 |
| A-02 | SC = Σ(头 6B + DATA) & 0xFF；AC = Σ(SC) & 0xFF | PROTOCOL-001 §1「和校验/附加校验」 | `sum_check`、`add_check`、`encode_frame` | `buildFrame` 单循环 `sum += b; add += sum;`（COMM-001 A2） | 一致 | 同上测试 `add_check(f[:13])`；`test_find_frames_valid_and_corrupt` 覆盖坏字节 |
| A-03 | 帧同步（AB 05 FF）+ 小端长度 + 半帧保留 | PROTOCOL-001 §1 | `extract_frames`（返回 `consumed`，调用方删前缀） | `receiveData`（`_rxIndex` 续帧、`rxStalled` 自愈） | 一致（PC 侧不整缓冲清空） | `test_find_frames_valid_and_corrupt`：垃圾前缀/尾部半帧 |
| A-04 | 0x00 校验帧 DATA = ID_GET + SC_GET + AC_GET | PROTOCOL-001 §2.1（安全通信协议） | `_on_param_check` 等确认链路 | `AnoComProtocol::sendDataCheck`（`onAnoRxFrame` 回传源帧真实 SC/AC） | 一致 | GCS-001 §2；`test_params_and_link.py` 参数链路用例 |

### 2.2 遥测帧缩放（`anocom.py` 解码 ↔ `communication.cpp` 组帧）

| ID | 帧 | 线缆格式/缩放 | 协议出处 | PC 解码（符号） | 固件发送（符号） | 结论 |
|----|----|----------------|----------|------------------|-------------------|------|
| A-05 | 0x01 IMU | 3×int16 acc(cm/s²) + 3×int16 gyr(32768↔2000 deg/s) + u8 | §2.1 | `decode_imu`（÷100、÷16.384） | `sendIMUData`（acc×100、gyr×16.384、限幅 ±32767） | 一致 |
| A-06 | 0x03 欧拉 | 3×int16 ×100 + u8 FUSION_STA | §2.3 | `decode_euler`（÷100；bit7=DETA100 在线、bit6=数据源） | `sendAttitudeEuler`；`communication.cpp` 的 `fusionSta` 高位编码 | 一致（扩展位官方未用，不破坏官方语义） |
| A-07 | 0x05 高度 | 3×int32 ×100(cm) + u8 | §2.5 | `decode_altitude`（÷100 → m） | `sendAltitudeData`（×100） | 一致 |
| A-08 | 0x07 速度 | 3×int16 ×100(cm/s) | §2.7 | `decode_speed`（÷100） | `sendFlightSpeed`（n×100、e×100、−down×100） | 等价变换后一致（固件已取反为"向上为正"） |
| A-09 | 0x08 位置 | 3×int32 ×100(cm) | §2.8 | `decode_pos`（÷100） | `sendPosOffset`（−relative_down） | 等价变换后一致（同上取反） |
| A-10 | 0x0A 目标姿态 | 3×int16 ×100 | §2.10 | `decode_target_attitude`（÷100） | `sendTargetAttitude`（rollTarget×100、pitchTarget×100、`omega_ref_dps`×100） | 一致（第 3 字段本工程语义为目标偏航角速率，注释已写明） |
| A-11 | 0x0B 目标速度 | 3×int16 ×100 | §2.11 | `decode_target_speed` | `sendTargetSpeed` | 一致 |
| A-12 | 0x0D 电压电流 | 5×u16 ×100 | §2.13 + 差异对照表 | `decode_volt_curr`（÷100；bat=真实 ADC、fc 字段=P1/P2 氧压） | `sendVoltCurr(bat_voltage_mv/1000, bat_current_ca/10, P1, P2, 0)` | 一致；旧注释"bat 占位常量 12.6/16.8"陈旧，本轮已按固件实码修正 |
| A-13 | 0x20 PWM | 8×u16，单位 0.01% | §2 控制量输出 | `decode_pwm` | `sendPWMOutput(ch3×100, ch4×100, 0…)` | 一致（本工程仅前两通道有物理语义） |
| A-14 | 0x21 姿态控制量 | 4×int16 ×10；本工程 α_ref(rad/s²)×10、thr×10 | §2.2 +「2026-08-11 单位兼容说明」 | `decode_attitude_control`（÷10 → rad/s²，派生 `ctrl_*_dps2` = ×180/π） | `sendAttitudeControl(...)`，实参 `alpha_ref_radps2×10`、`throttlePercent×10` | 一致（线缆保持 rad/s²，显示域 deg/s²） |
| A-15 | 0x30 GPS | u8/u8 + i32×2(1e7) + i32(cm) + i16×3(cm/s) + u8×3(×10) | §3.1 + 差异对照表（÷10 而非手册文字 ÷100） | `decode_gps`（÷1e7、÷100、÷10） | `sendGPSInfo1`（pdop×10、vacc×10、sacc×10） | 一致；★ 形参命名与实参对调（形参 `…,vacc,sacc`、调用实参 `…,sacc,vacc`），字节流仍为 `[pdop, SACC, VACC]`＝手册 p[20..22]，两侧变量名与内容相反（注释已写明） |
| A-16 | 0x40 遥控 | 10×int16（手册 1000–2000 µs，0=无通信） | §5.1 + 差异对照表 | `decode_rc`（`<10h`） | `sendRCData(const uint16_t*)`（10×u16 LE） | 等价变换后一致（1000–2000 与 0 范围内有符号/无符号同值） |
| A-17 | 0xF1 执行器（本工程自定义灵活帧） | 12B：摆角 int16 ×100、电机 u16 ×10、Δω int16 ×1000、sat u8、预留 u8 | PROTOCOL-001 §1.3（0xF1–0xFA 灵活格式帧）+ 差异对照表 | `decode_actuator` | `communication.cpp` 的 `act[12]`（`put16` 小端） | 一致（顺序与缩放逐项对齐；sat bit0/1/2 = δf/δt/Δω） |

### 2.3 参数链路（`params.py` ↔ `ano_params.cpp`，只读核对固件）

| ID | 要素 | 出处 | PC 实现 | 固件实现 | 结论 | 证据 |
|----|------|------|---------|----------|------|------|
| B-01 | 参数 ID = 注册表顺序（当前 128 项） | GCS-001 §2；`ano_params.h` `ANO_PARAMS_COUNT 128` | `expected_names()` | `kAnoParams[ANO_PARAMS_COUNT]` | 一致 | `tests/test_params_and_link.py::test_param_count_matches_firmware`：len=128、`names[117]=='inertia_comp_mask'`、`names[121:128]` 为 rc_rate/rc_expo[0..2]/rc_super[0..2] |
| B-02 | 12 环 × 9 字段顺序（kp、ki、kd、out_min、out_max、int_limit、threshold、falpha、enabled） | 同上 | `PID_GROUPS` + `FIELD_META` + `expected_names` | `PID_FLOAT`/`PID_ALPHA`/`PID_ENABLED` 宏展开 | 一致 | 同测试；`FIELD_META['filter_alpha']['wire_name']='falpha'` 对应 `PID_ALPHA` 的 `#loop ".falpha"` |
| B-03 | 滤波数组短名 spd_alpha/ang_alpha/out_alpha/spd2_alpha | PAR_NAME 定长 20B（§4.3） | `FILTER_WIRE` | `ALPHA_ENTRY` | 一致 | 同测试的尾部顺序断言 |
| B-04 | 参数类型 float=8 / uint8=0 | §4.4 | `ANO_FLOAT=8`、`ANO_UINT8=0`、`is_float_name` | `AnoDataType`、`anoParamSerialize` | 一致 | `test_params_and_link.py` 类型用例 |
| B-05 | 0xE0 命令语义 0x01/0x02/0x03/0x10 | §4.1 | `cmd_read_param_count`、`cmd_read_param_value`、`cmd_read_param_info`、`cmd_param_restore_defaults`、`cmd_param_save` | `anoParamsHandleRx` 的 `switch(cmd)` | 一致 | `test_params_and_link.py` |
| B-06 | 0xE1 写入 = ID u16 LE + PAR_VAL（类型定长） | §4.2 | `cmd_write_param`（float 4B / u8 1B） | `anoParamDeserialize`（长度严格匹配 + `isfinite` 拒绝 NaN/Inf） | 一致 | 同测试写读回用例 |

### 2.4 黑匣子（`blackbox.py` ↔ 固件 `W25N01GV_log`，只读核对）

| ID | 要素 | 出处 | PC 实现 | 固件实现 | 结论 |
|----|------|------|---------|----------|------|
| C-01 | I 帧 63B：magic AA55 + 0x49 + seq u16 + t_ms u32 + N×f32 + CRC16 | `lib/W25N01GV/src/W25N01GV_log.h` | `IFRAME_SIZE = 9 + 4N + 2` | `W25N01GV_LOG_IFRAME_SIZE 63` | 一致（N=13） |
| C-02 | P 帧 33B：magic + 0x50 + seq u16 + dt u16 + (N−1)×i16 + CRC；差值 ×100 | 同上 + GCS-001 §3 | `PFRAME_SIZE = 7 + (N−1)*2 + 2`；`deltas / DELTA_SCALE`；末通道保持参考值 | `buildPFrame`（`N_DELTAS = (PFRAME_SIZE-9)/2 = 12`） | 一致（末槽为 CRC，不按 22 增量解析） |
| C-03 | S 帧 236B 定长：magic + 0x53 + seg u16 + t_ms u32 + 名表 224B（NUL 结尾）+ CRC | 同上 | `SFRAME_MAX=236`、`_sframe_channel_count` | `buildSFrame`（`SFRAME_MAX`） | 一致（名表自描述，列名不硬编码） |
| C-04 | E 帧 15B：magic + 0x45 + seg u16 + dur u32 + frames u32 + CRC | 同上 | `EFRAME_SIZE=15` | `buildEFrame` | 一致 |
| C-05 | CRC16-CCITT poly 0x1021、init 0xFFFF | 同上 | `crc16` | `crc16_ccitt` | 一致 |
| C-06 | 通道数动态推导（当前 13；旧段无 S 帧回退 22） | 同上（通道表 22→13 裁剪） | `_set_channel_count`、`extract_frames` | 固件通道表 | 一致（`tests/test_blackbox.py`） |

### 2.5 WebSocket/HTTP 字段、单位与频率（后端 ↔ 前端）

| ID | 要素 | 出处 | 后端 | 前端 | 结论 |
|----|------|------|------|------|------|
| D-01 | 遥测推送 50Hz | COMM-001 §4 C 层（TELEMETRY_PERIOD 0.05→0.02） | `TELEMETRY_PERIOD=0.02`、`_on_telemetry_rx` 节流 | `main.mjs` 解析 `telemetry`；`scope`/`dashboard`/`view3d` 订阅 bus | 一致（前端随事件刷新 ≈50Hz） |
| D-02 | hello 版本握手（防残留旧后端） | AGENTS.md 踩坑记录「改 GCS 后端后必须杀掉旧进程」 | `ws_endpoint` 发 `{'type':'hello','version':app.version}` | `main.mjs` `BACKEND_VERSION='0.3.0'` 比对并告警 | 一致（`main.py` `app=FastAPI(version='0.3.0')`） |
| D-03 | 字段单位/缩放 | `anocom.py` 各解码函数 | 快照字段后缀 `_deg/_dps/_ms2/_ms/_m/_pct/_v` | `scope.PRESETS`、`dashboard` 控件按后缀显示 | 一致（前端不做二次换算） |
| D-04 | 0x21 派生量 | PROTOCOL-001「2026-08-11 单位兼容说明」 | `decode_attitude_control` 同时给 `ctrl_*`(rad/s²) 与 `ctrl_*_dps2`(deg/s²) | `dashboard` 用 `ctrl_*_dps2`（α °/s²） | 一致 |
| D-05 | 记录 CSV 列 | GCS-001 §上位机（记录/回放） | `datalog.RECORD_FIELDS` 固定列 | 回放复用同一 `telemetry` 结构 | 一致 |
| D-06 | 变量（0xF2/0xF3）字段命名 | AGENTS.md 踩坑记录（AnoVars） | `_on_vars_list`（名 16B）、`_on_vars_value`（id u16 + f32 LE） | `dashboard` 变量表 | 一致 |

### 2.6 工具脚本核对口径

| ID | 脚本 | 核对对象 | 口径来源 | 结论 |
|----|------|----------|----------|------|
| E-01 | `verify_axis_map.py` | 传感器 RUB → 机体系 FRD：`bX=-sZ`、`bY=+sX`、`bZ=-sY`；`q_body_from_FLU` 绕 X 180° | AGENTS.md 轴映射踩坑、`sensor_imu.cpp` | 一致 |
| E-02 | `verify_mix_axes.py` | 模型系 `x'=-z_b`、`y'=+y_b`、`z'=+x_b`（det=+1）；`Mx'=-Ix·α_yaw` 等符号 | AGENTS.md 轴置换踩坑、CTL-001 §5 | 一致（脚本头标注"以实机直通为锚点"） |
| E-03 | `verify_frame_map.py` | 存档系（z=推力轴）↔ 模型系置换：`Mx'=-Mz`、`My'=+My`、`Mz'=+Mx` | MOD-002 §3、CTL-001 §5 | 一致（det=+1 手性检验） |
| E-04 | `verify_control_sign.py` | 俯仰符号链：角度域 Kp 2.8、速率环 16.042818 s⁻¹、Iy=0.022、力臂 0.315 | CTL-001 §4（速率环量纲 2026-08-11 迁移） | 一致（FAIL 时 exit 1，可作断言） |
| E-05 | `verify_allocation_sign.py`、`verify_torque_sign.py`、`verify_slope_direct.py` | B 矩阵与力矩/斜率符号（My→δt、Mx→Δω、δf→Mz） | THY-002 §2–3、THY-003 §2–3、CTL-001 §5 | 一致 |
| E-06 | `verify_w0_floor.py`、`verify_floor_consistency.py`、`verify_yaw_{authority,gain_schedule,vs_archive}.py` | w0 floor、调度 `s=(w0/w_hover)²`、差速权限与存档对照 | AGENTS.md 差速增益调度三层坑、CTL-001 §5 | 一致（脚本头已标注"历史回放/历史设计评估"，不代表现役链路） |
| E-07 | `audit_math.py` | TVC 15° 限幅与增益饱和匹配、差速限幅余量 | CTL-001 §5 | 一致（历史脚本，未含 τm 观测器） |
| E-08 | `test_mavlink_bridge.py` | MAVLink 参数桥短名（≤15B）与写后读回 | AGENTS.md MAVLink 踩坑记录 | 口径一致；**波特率写死 2000000，与 COMM-001 后 921600 不符**（P2） |
| E-09 | `dps_profile_parser.js`、`dps_profile_runner.js`、`nmea-host-test/**` | DPS310 诊断汇总、GNSS 双协议宿主回归 | 固件诊断宏、`lib/ublox-main` | 一致（仅测试/分析，不参与固件） |

---

## 3 发现清单

| 严重度 | 位置（符号） | 问题 | 影响 | 建议 |
|--------|--------------|------|------|------|
| — | — | 未发现 P0（帧格式/校验/缩放/参数 ID 与固件无安全级不一致） | — | — |
| P1 | `params.py::build_meta` | `FIELD_META` 加入 `rc_rate/rc_expo/rc_super` 后，`build_meta` 仍对每个 PID 环展开这些字段，生成 36 个永不命中的 `att_roll.rc_rate` 等条目（META 实际 164 项 vs 固件 128）；`expected_names()` 已用 `field.startswith('rc_')` 跳过，两函数口径不一致 | 无功能影响（前端按固件 E2 名查表），但误导核查、增大元数据表 | 给 `build_meta` 加同样的 `rc_*` 跳过；属行为相邻改动，本轮登记不动，建议随下次参数变更一起做并补测试 |
| P1 | 参数计数注释（固件 `ano_params.h`/`ano_params.cpp`/`communication.cpp`，本区 `params.py`） | 注释仍写"117/121 参数"，实际注册表 128（C 路径后两次追加：spd2_alpha 3 项、FPV 摇杆曲线 7 项） | 维护者误判参数总数，排查 ID 错位时走弯路 | GCS 侧已去掉写死数字并注明当前 128；固件侧属其他 Agent 范围，登记 |
| P2 | `anocom.py::decode_volt_curr` | 旧注释称 bat 电压/电流为"占位常量 12.6/16.8、无真实采样"；固件早已改为真实 ADC（`bat_voltage_mv/1000`、`bat_current_ca/10`） | 注释与实现相反，易误判遥测来源 | 本轮已修正注释（行为未动） |
| P2 | `main.py::_on_vars_value` | docstring 写"随 20Hz telemetry 推送"，实际 `TELEMETRY_PERIOD=0.02`（50Hz） | 频率口径错误 | 本轮已改为 50Hz |
| P2 | `test_mavlink_bridge.py` | `BAUD=2000000` 未随 COMM-001（921600）更新；PARAM 条数文案写死 121 | 直接运行会乱码/超时，统计文案与实际 128 不符 | 联调前改 BAUD；条数文案建议改为 `len(pm.expected_names())`（工具脚本非 CI 路径） |
| P2 | `GCS-001` 文档 | 文中"117 参数""pytest 26 项"为 2026-08-08 口径，当前 128 参数、35 项测试 | 文档滞后 | 建议随本记录交叉更新（不属本任务范围） |
| P2 | `TandemVec_FCS/tools` 历史脚本 | `verify_yaw_*`、`audit_math`、`verify_w0_floor` 保留迁移前数值口径（未含 τm 观测器/FPV 曲线） | 误当现役检查器会得出过时结论 | 已在脚本 L0 标注"历史回放/历史设计评估" |

---

## 4 未决问题

1. **0x30 GPS 形参命名与内容互换**：`sendGPSInfo1` 形参 `(…, pdop, vacc, sacc)`，`communication.cpp` 调用实参 `(…, pdop, sacc, vacc)`，函数体内 `data[21]=vacc×10`、`data[22]=sacc×10`。字节流与手册一致（data[21]=SACC、data[22]=VACC），但两处命名与内容相反。任何一侧"修正命名"都会翻转字节语义——改动前必须同时改调用与 GCS 解码；本记录保留现状，建议在固件通信层统一命名后一并验证。
2. **0x40 RC 有符号/无符号**：固件 `sendRCData(const uint16_t*)`，手册定义 int16。当前 1000–2000 与 0 范围内等价；若未来出现负值或 >32767 将不一致（现有 RC 语义不会出现）。
3. **`build_meta` 与 `expected_names` 口径不一致**（P1）：需确认前端/工具是否依赖 `META` 条目数量或键集合做校验（当前未见），再决定是否清理 36 个垃圾条目；清理时须同步 `tests/test_params_and_link.py`。
4. **文档滞后**：`GCS-001`（117 参数、26 项测试）与 `COMM-001`（README 中部分旧口径）需随本轮记录更新，避免下一轮核查再次踩"117/121"。

---

## 5 门禁与测试

- `py -3.12 tools/check-code-comments.py --area gcs --check`
  - 核查前：exit 1，**42 error / 77 warning**（R01=42、R08=47、R09=28、R04=2），平均分 70.8，0 文件全绿。
  - 核查后：exit 0，**0 error / 0 warning**，平均分 100.0，42/42 文件全绿，规则命中"无"。
- `cd TandemVec_FCS/GCS && py -3.12 -m pytest tests/ -q`
  - 核查前：`35 passed, 2 warnings`（0.28 s）
  - 核查后：`35 passed, 2 warnings`（0.29 s）
- 前端/脚本语法：`node --check web/src/*.mjs`（8/8 通过）、`node --check tools/dps_profile_{parser,runner}.js`（2/2 通过）。
- 行为保持自查：`git diff -U0 -- TandemVec_FCS/GCS TandemVec_FCS/tools` 删除行仅为旧 docstring/banner 注释，新增行均为注释/文档字符串；协议常量、解码缩放、WS 字段、前端逻辑、pytest 用例均未改动。

---

## 6 修订记录

| 日期 | 变更 |
|------|------|
| 2026-10-23 | 首版：42 文件 L0/L1 补齐；AnoCom 帧格式/缩放、参数链路、黑匣子、WS/前端、工具口径五组核对矩阵；P1×2、P2×5、未决 4 项 |
