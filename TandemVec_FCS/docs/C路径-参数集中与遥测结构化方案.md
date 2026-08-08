# C 路径方案 — 实机控制参数集中 + 遥测结构化（零行为变化重构）

> 状态：✅ 已实施（2026-08-08，pio run 双环境编译通过、test_host/run_all.sh 全绿、数值逐项核对一致）
> 触发：CascadeCtrl 对比分析后，用户选择 C 路径（借鉴新架构的"参数集中 + 遥测可观测"设计，不替换控制律）
> 关联：`docs/cascade-ctrl-接入规划.md`（B 路径规划，C 是其低风险前置）、`include/TandemVec_CtrlParams.h`（警示注释）
> ⚠️ 文中所引 CascadeCtrl 半成品架构（`TandemVec_CtrlParams/AttitudeCtrl/RateCtrl/CascadeCtrl.h`）已于 2026-08-08 废弃删除；本文作为 C 路径实施记录保留，参数/遥测集中成果仍在（`include/FlightCtrlParams.h` + `GncTelemetry`）。
> 核心约束：**控制律行为零变化**——不触碰任何数值、时序、滤波、调度、分配逻辑。

---

## 1 背景

实机控制链（flight_control.cpp + state_data.cpp 的 12 个 PositionPID 全局对象）经 5 轮实机调参验证，行为可靠。但存在三个组织性问题：

1. **参数分散**：12 个 PID 的增益散落在 `state_data.cpp` 构造字面量（144–206 行）+ `flight_control.cpp` `initPositionHold()` 运行时配置（509–525 行）+ 7 个控制滤波器 alpha（`state_data.cpp` 240–258 行）。调参 = 翻文件找行号，无"参数一览"。
2. **僵尸参数**：`yawAnglePID(0.8)` 构造后永不参与控制（`execute_yaw_controller` 中恒 `reset()`）——无效参数与有效参数混杂，调参时误导。
3. **遥测分散**：控制链中间量是 ~15 个零散全局 float（`state_data.cpp` 415–444 行），消费方（CAN/AnoCom/Serial8）各自按名引用；**无法整体观测"外环→内环→分配→执行器"每一层**——这正是调参阶段最需要的能力。

## 2 本质与边界

**本质**：C 路径是"参数组织 + 可观测性"重构，不是控制律替换。它把新架构（CascadeCtrl）的两个优点——参数集中（CtrlParams.h）与全链遥测（CascadeTelemetry）——以**实机语义**复刻到现有链上。

**边界（红线）**：
- ✋ 不修改任何控制数值（增益/限幅/滤波 alpha/调度系数，逐值搬移）
- ✋ 不修改控制时序与数据流（200Hz 帧内顺序）
- ✋ 不修改协议帧格式（CAN 帧、AnoCom 组帧、Serial8 调试行——消费方只改"读取来源"，不改"缩放系数"）
- ✋ 不修改 `PositionPID.h` 类本身（test_host 测试依赖它的行为）
- ✋ 不触碰 `TandemVec_CtrlParams.h` 的默认值实例（`kDefaultCascadeCtrlParams` 仍仅 test_host 用）

## 3 现状调查结论（接线点全景）

| 消费方 | 文件:行 | 引用量 | 缩放 |
|---|---|---|---|
| CAN 帧5 | `can_bus.cpp:156` | `throttlePercent`, `yaw_output` | 原样 |
| CAN 帧6 | `can_bus.cpp:161` | `roll_output`, `pitch_output` | 原样 |
| AnoCom 控制组 | `communication.cpp:420-431` | `roll/pitch/yaw_output`(×10), `yawRateTarget`(×100), `rollTarget/pitchTarget`(×100), `throttlePercent`(×10) | 缩放保留在消费方 |
| Serial8 调试 | `communication.cpp:741-743` | `error_roll_deg`, `error_pitch_deg` | 原样 |
| 黑匣子 Serial3 | `communication.cpp:780-840` | **不记录控制量**（仅姿态/IMU/INS/TVC反馈/发动机） | — |

**滤波器**（`state_data.cpp` 240–258 行）：`roll/pitch/yawSpeedFilter`(0.3)、`roll/pitch/yawAngleOutputFilter`(0.85)、`roll/pitchOutputFilter`(0.25)、`yawOutputFilter`(0.12，实机调出)。

**PID 构造全貌**（`state_data.cpp` 144–206 行）：12 个对象，构造参数仅 (kp,ki,kd)，其余用 PositionPID 默认（out=±100、intLimit=250、threshold=0、filter=0）；运行时配置仅在 `initPositionHold`（位置/速度环的 setOutputLimits/setIntegralLimit/setFilterCoefficient）。

**test_host 依赖**：`test_position_pid.cpp`（P1–P12）测 PositionPID 类本身；`test_flight_control_axis.cpp` 等**自建实例**（`test_flight_control_axis.cpp:165`），不依赖 src/ 全局——**test_host 全绿不受本次重构影响**（类不动即安全）。

## 4 设计

### 4.1 参数结构体（新建，语义按 PositionPID 实际参数）

> ⚠️ 不复用 `TandemVec_CtrlParams.h` 的类型：其 `alpha_max/int_max` 数值域（rad/s²）与 PositionPID 实际语义（deg 域、误差·拍域）不同，硬套会引入 57 倍类错误（见 cascade-ctrl-接入规划 §2）。

```cpp
// state_data.h 新增
struct PidTuneParams {
    float kp, ki, kd;        // 构造参数（值=现状字面量）
    float out_min, out_max;  // 输出限幅（PositionPID 构造参数 5/6；姿态环现状 ±100）
    float int_limit;         // 积分状态钳位（构造参数 7；现状 250）
    float threshold;         // 积分分离阈值（构造参数 8；现状 0）
    float filter_alpha;      // 微分滤波系数（构造参数 9；现状 0=直通）
    bool  enabled;           // 该环是否参与控制（yawAnglePID=false，显式标记僵尸参数）
};

struct FlightCtrlParams {
    // 姿态串级（6 环）——现状值（2026-08-07 实机收敛值）
    PidTuneParams att_roll,  att_pitch,  att_yaw;    // 2.5/0/0 | 2.5/0/0 | 0.8/0/0(enabled=false)
    PidTuneParams rate_roll, rate_pitch, rate_yaw;   // 0.25/0.0003/0 | 0.25/0.0003/0 | 0.20/0.001/0
    // 垂直（2 环）——现状值
    PidTuneParams alt_pos, alt_vel;                  // 1.0/0/0 | 5.0/0.00625/0
    // 水平位置/速度（4 环）——现状值（运行时配置见 §4.3）
    PidTuneParams pos_n, pos_e, vel_n, vel_e;        // 0.25 | 0.25 | 1.75/0.00125/10
    // 控制滤波器 alpha（7 个，现状值）
    float speed_filter_alpha[3];    // 0.3/0.3/0.3
    float angle_out_filter_alpha[3];// 0.85/0.85/0.85
    float output_filter_alpha[3];   // 0.25/0.25/0.12（yaw 为实机调出值）
};

extern const FlightCtrlParams kFlightCtrlParams;  // state_data.cpp 定义，全部当前值
```

**构造改造**：`state_data.cpp` 中 12 个 `PositionPID` 构造改为从 `kFlightCtrlParams` 读取对应字段（值逐字搬移，git diff 可核对）。`yawAnglePID` 构造保留但标记 `enabled=false`，注释"未启用（纯速率指令）"。

### 4.2 遥测结构体（新建，照 CascadeTelemetry 形态但按实机中间量）

```cpp
// state_data.h 新增
struct GncTelemetry {
    // 外环
    float error_deg[3];       // error_roll/pitch/yaw_deg（yaw 恒 0，无姿态回中）
    float omega_ref_dps[3];   // roll/pitchRateTarget + yawRateTarget（含滤波后）
    // 内环
    float alpha_ref[3];       // outputs.alpha_roll/pitch/yaw（输出滤波后）
    // 分配层（mix 层，调度后）
    float M_cmd[3];           // Mx/My/Mz（增益调度后、allocateMoments 前）
    float w0_eff;             // 工作点（含 w0_floor）
    float yaw_gain_sched;     // 差速增益调度系数（s_yaw_gain_sched）
    // 执行器指令
    float delta_f_deg, delta_t_deg, dw;
    bool  alloc_sat[3];       // 摆角/差速饱和标记（从 AllocationOutput 拷入）
};

extern GncTelemetry gnc_tel;  // state_data.cpp 定义（初值全 0）
```

**写入点**（3 处，全部"只读现有计算值"，不改任何计算）：
1. `execute_attitude_controller` 尾部：`error_deg[0/1]`、`omega_ref_dps[0/1]`、`alpha_ref[0/1]`
2. `execute_yaw_controller` 尾部：`error_deg[2]`、`omega_ref_dps[2]`、`alpha_ref[2]`
3. `mix_and_output_commands` 尾部：`M_cmd`、`w0_eff`、`yaw_gain_sched`、`delta_f/t`、`dw`、`alloc_sat`

**消费方改造**（数值不变，只换读取来源；缩放系数原地保留）：
- `can_bus.cpp:156,161`：`yaw_output→gnc_tel.alpha_ref[2]`，`roll_output→gnc_tel.alpha_ref[0]`，`pitch_output→gnc_tel.alpha_ref[1]`
- `communication.cpp:420-422`：同上三处
- `communication.cpp:431`：`yawRateTarget→gnc_tel.omega_ref_dps[2]`
- `communication.cpp:741-743`：`error_roll_deg→gnc_tel.error_deg[0]`，`error_pitch_deg→gnc_tel.error_deg[1]`

### 4.3 运行时配置收拢

`initPositionHold()` 中位置/速度环的 `setOutputLimits/setIntegralLimit/setFilterCoefficient` 改从 `kFlightCtrlParams` 读取（值不变）：
- `pos_n/pos_e`：out=±POS_CTRL_MAX_SPEED_CMD(1.5)、int_limit=0.75、filter=0.5
- `vel_n/vel_e`：out=±MAX_ACCEL_CMD(2.6)

### 4.4 旧全局 float 处置

`roll_output/pitch_output/yaw_output/yawRateTarget/rollRateTarget/pitchRateTarget/error_roll_deg/error_pitch_deg/error_yaw_deg` **删除**，由 `gnc_tel` 取代（编译期链接错误可发现所有漏改点）。
保留不动：`throttlePercent`、`rollTarget/pitchTarget`（非控制链中间量，且被多处直接消费，收拢无收益）。

## 5 影响面清单

| 文件 | 改动 |
|---|---|
| `src/state_data.h` | +`PidTuneParams`/`FlightCtrlParams`/`GncTelemetry` 定义、+2 extern；删除被取代的中间量 extern |
| `src/state_data.cpp` | +`kFlightCtrlParams` 实例定义（全部当前值）、+`gnc_tel` 定义；12 个 PID 构造改读实例；删除旧全局定义 |
| `src/flight_control.cpp` | `initPositionHold` 配置改读实例；3 个遥测写入点；其余**不动**（PID 对象名不变） |
| `src/can_bus.cpp` | 2 行改读 `gnc_tel` |
| `src/communication.cpp` | 5 处改读 `gnc_tel` |
| 不碰 | `PositionPID.h`、`TandemVec_*.h`、`test_host/`、协议帧格式、控制逻辑 |

## 6 风险与对策

| 风险 | 等级 | 对策 |
|---|---|---|
| 行为变化（数值误搬） | 高 | ① 构造参数逐值搬移，git diff 逐行核对；② 消费方只改来源不改缩放；③ 阶段化实施每步可独立编译验证；④ 实机地面检查（摆向/响应方向测试） |
| 漏改导致链接/行为异常 | 中 | 删除旧全局 → 编译期链接错误暴露所有漏改点（强保证） |
| 遥测写入时序（写入点在控制计算之后） | 低 | 写入点固定在各自函数尾部，同一 GNC 拍内数据一致（与 `s_yaw_gain_sched` 现有模式相同） |
| 范围蔓延 | 中 | 红线清单（§2）逐条对照；不做任何"顺手优化" |
| test_host 回归 | 无 | 测试自建实例、不依赖 src 全局；PositionPID.h 不动即安全 |

## 7 实施步骤（每步独立可验证）

1. **阶段 1（参数收拢）**：state_data.h/cpp 加结构体+实例，12 个构造改读 → `pio run` 编译通过
2. **阶段 2（运行时配置收拢）**：`initPositionHold` 改读实例 → 编译通过
3. **阶段 3（遥测结构）**：state_data.h/cpp 加 `gnc_tel` + 3 个写入点 → 编译通过
4. **阶段 4（消费方 + 删除旧全局）**：can_bus/communication 改读，删除旧 float → 编译通过（链接错误清空 = 无漏改）
5. **阶段 5（验证）**：`pio run` 全绿、`test_host/run_all.sh` 全绿（含 test_position_pid 12 项）、git diff 数值逐项核对、FCS README 安全流程地面检查
6. **阶段 6（文档）**：FCS AGENTS.md「核心参数」节更新（单一来源 → `kFlightCtrlParams`）、`README.md` 调参入口说明同步、方案文档归档为"已实施"

## 8 验收标准

- [ ] `pio run` 零警告编译通过
- [ ] `test_host/run_all.sh` 全绿（test_position_pid P1–P12 全过）
- [ ] git diff 中所有数值（增益/限幅/滤波 alpha）与重构前逐项一致
- [ ] CAN/AnoCom/Serial8 输出数值与重构前逐帧一致（可用 analyze_ano.py/遥测对比）
- [ ] 实机地面检查：解锁前后行为与重构前一致

## 9 开放决策项（实施前拍板）

| # | 问题 | 推荐 | 备选 |
|---|---|---|---|
| Q1 | 高度/位置环（6 个 PID）是否本轮一并收拢？ | ✅ 全部收拢（一次解决"调参入口分散"，多 6 个字段零额外风险） | 只收姿态环（6 个），后续再说 |
| Q2 | 被取代的旧全局 float 是否删除？ | ✅ 删除（编译期强保证无漏改） | 保留（消费方双写，改动最小但分散依旧） |
| Q3 | 滤波器 alpha 是否收拢进参数结构体？ | ✅ 收拢（yawOutputFilter 0.12 是实机调出值，必须进调参入口） | 保持现状 |
| Q4 | 遥测消费方本轮就改读 `gnc_tel`？ | ✅ 改（否则"遥测集中"落空） | 消费方不动，`gnc_tel` 仅新增并行层 |

---

### 附：为什么 C 优于直接接入 CascadeCtrl（B 路径）

C 与 B 的关系不是替代而是前置：C 完成后，实机参数有唯一入口（`kFlightCtrlParams`）、控制链每层可观测（`gnc_tel`），调参积累的数据直接成为 B 路径接入时 `TandemVec_CtrlParams.h` 数值的实机依据。而 B 的"限幅单位 57 倍差异、缺 NaN 防护/无扰切换、丢增益调度/floor/轴置换"等风险（见 cascade-ctrl-接入规划 §2/§4）决定了它不适合在调参收敛期执行。
