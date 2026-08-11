# CascadeCtrl 接入规划 — 将 TandemVec 级联控制链接入固件（统一调参入口）

> **状态：已废弃（2026-08-08 用户决策）** —— CascadeCtrl 半成品架构整体删除，
> 不执行本规划。替代方案已落地：
> ① C 路径（`docs/C路径-参数集中与遥测结构化方案.md`）——参数集中到
> `include/FlightCtrlParams.h`（唯一事实源）、遥测集中到 `GncTelemetry`；
> ② `TandemVec_AttitudeCtrl/RateCtrl/CascadeCtrl/CtrlParams.h` 已从仓库删除，
> 浮点四元数工具抽为 `include/Quat4f.h`。
> 本文档保留作为决策链记录（触发问题：CtrlParams.h 声称唯一调参入口却未编译进固件）。
> **2026-08-11 补记**：现役 PositionPID 速率环已迁移为 `deg/s → deg/s²`，`kp` 直接使用 `s⁻¹`；本文 `0.25/0.20` 等均为迁移前历史值，不得用于当前参数导入。
>
> 原始状态（2026-08-08 起草）：规划草案
> 关联：`include/TandemVec_CtrlParams.h`（警示注释）、`include/TandemVec_CascadeCtrl.h`、
> `src/flight_control.cpp`、`src/state_data.cpp`、`AGENTS.md`（第 71 行旧决定）
> 触发：实机首飞验证通过后进入调参优化阶段，发现 `TandemVec_CtrlParams.h`
> 注释声称"所有飞控增益集中在此文件"，但该文件未编译进固件——实机调参存在走错门的风险。

---

## 1 背景与问题

仓库内存在**两套控制实现**：

| | 新架构（CascadeCtrl 级联链） | 实机在用（flight_control.cpp 手写串联） |
|---|---|---|
| 文件 | `TandemVec_AttitudeCtrl.h` / `TandemVec_RateCtrl.h` / `TandemVec_CascadeCtrl.h` | `src/flight_control.cpp` + `src/state_data.cpp` 全局 `PositionPID` 对象 |
| 参数 | `TandemVec_CtrlParams.h`（`kDefaultCascadeCtrlParams`：att.kp=4.0/4.0/3.0，rate.kp=8/12/10） | `src/state_data.cpp`（roll/pitch：2.5+0.25，yaw：0.8+0.20，高度 1.0+5.0，位置 0.25+1.75） |
| 编译进固件 | **否**（仅 test_host/ 宿主机回归测试引用） | 是（正在飞行） |
| 验证程度 | 宿主机数学测试（`test_tandemvec_cascade.cpp` 14 项 + `test_tandemvec_sim.cpp` 闭环仿真） | 实机 5 轮调参（state_data.cpp 注释有完整震荡史） |

**踩坑点**：`TandemVec_CtrlParams.h` 头注释自称"所有飞控增益集中在此文件，便于一眼纵览全套参数"，但 `src/*.cpp` 的 include 链只有 `TandemVec_Config.h` / `TandemVec_Propulsion.h` / `TandemVec_ControlAllocation.h` / `TandemVec_OnlineID.h`。修改 `TandemVec_CtrlParams.h` 的增益并烧录后**实机行为不变**。

**历史原因**：提交 `62124e9`（移植重构）引入整套新架构头文件，其中**控制分配层**（`TandemVec_ControlAllocation.h`，BTRUE 在线 Jacobian）、**推进层**（`TandemVec_Propulsion.h`）、**在线辨识**（`TandemVec_OnlineID.h`）已迁移接入 `flight_control.cpp`；**姿态/速率环未迁移**——`PositionPID` 体系是实机验证过、带调参史的，`4d3229e` 还在继续打磨旧体系（修复 precise_scale、偏航改纯速率、倾角保护移除）。

## 2 决策链（重要）

- **2026-07-29**：`AGENTS.md` 第 71 行明确记录——"`test_host/` 中的级联控制与闭环仿真按实际 `flight_control.cpp` 控制律、参数、单位和轴映射建立，用于宿主机验证固件行为；**它们不是待接管固件输出的另一套生产控制链，不要据此提出架构统一或双链切换**"。
- **2026-08-08**：实机首飞验证通过后，用户决定"警示 + 规划接入"——将 CascadeCtrl 接入固件以统一调参入口（`TandemVec_CtrlParams.h` 恢复名副其实）。
- **执行前置条件**：接入工作开始前，必须**先更新 `AGENTS.md` 第 71 行规则**，注明 2026-08-08 用户决策推翻"不做双链切换"，否则后续会话会持续阻止本计划。这不是"依据 test_host 存在而提出切换"，而是实机调参需求驱动的用户决策——决策依据不同。

## 3 接入目标与建议形态

**目标**：`TandemVec_CtrlParams.h` 成为实机唯一调参入口；消除双参数体系。

**建议形态：只迁移层 1/层 2，保留现有层 3/层 4。**

理由：CascadeCtrl 的 `step()` 内层 3/层 4 是**旧版直连**（`M_cmd[i] = I·α_ref[i]`，`w0 = thr·wMax` 直算），而 `flight_control.cpp` 现有的分配链包含最近 5 个提交的全部成果——直接整体替换会**丢失**：

- 层 1 轴置换（`Mx'=-Ix·alpha_yaw`、`Mz'=-Iz·alpha_roll`，`tools/verify_mix_axes.py` 验证）
- 差速回路增益调度（`Mx ×= (w0_eff/w_hover)²` 限幅 [0.02, 4.0]，根治低油门自激震荡）
- w0 工作点下限（`w0_eff = max(w0, 0.6·w_hover)`，`current_state` 转速同步 floor，防 B 矩阵病态）
- 零油门门控（<5% 油门执行器归中）
- BTRUE 策略 + `s_yaw_gain_sched` 导出给在线辨识（alpha_cmd 修正）

正确形态：`flight_control.cpp` 的姿态/速率环（`execute_attitude_controller` / `execute_yaw_controller` 中 q_error→PositionPID 链）替换为 `AttitudeCtrl` + `RateCtrl` 调用，**分配链（层 1 轴置换 → 层 2 调度 → allocateMoments）保持不动**。CascadeCtrl 的层 3/层 4 或保持现状（仅供 test_host 全链仿真），或后续把现有层 3/4 逻辑反哺进 CascadeCtrl 后再整体使用（见 §8 待决策项）。

## 4 关键差异与风险清单

| # | 差异点 | 现状（flight_control.cpp） | CascadeCtrl | 接入处理 |
|---|--------|---------------------------|-------------|----------|
| 1 | 轴置换 | 层1 显式置换（Mx'←alpha_yaw 等，存档系→模型系） | 层3 直连 `M_cmd[i]=I·α_ref[i]` | 分配链保留现状，不经过 CascadeCtrl 层3 |
| 2 | 差速增益调度 | `Mx ×= (w0_eff/w_hover)²`，限幅 [0.02,4.0] | 无 | 保留在 flight_control.cpp |
| 3 | w0 工作点下限 | `w0_eff=max(w0, 0.6·w_hover)` | `w0=thr·wMax` 直算 | 保留现状 |
| 4 | 零油门门控 | <5% 油门 → 执行器归中 | 无 | 保留现状 |
| 5 | 分配策略 | `allocateMoments(ai, P, BTRUE)` + current_state | 默认 FULL_B（可传参） | 保持 BTRUE 调用 |
| 6 | yaw 纯速率模式 | `yawAnglePID` 永久禁用，yaw 恒为角速度指令（`execute_yaw_controller` 统一映射） | 三轴角度外环（q_err.z→ω_ref.z） | **需扩展 CascadeCtrl**：yaw 通道支持外环旁路/直通 |
| 7 | ATTITUDE_MODE | CH9 切换角度模式/角速率模式（角速率模式旁路外环） | 无此概念 | 外层模式逻辑保留，仅替换环计算 |
| 8 | 角速率滤波 | `roll/pitch/yawSpeedFilter`（α=0.3） | 无 | 外层保留 |
| 9 | 在线辨识联动 | `s_yaw_gain_sched` 导出修正 alpha_cmd（`TandemVec_OnlineID.h` 步骤9） | 无 | 保留现状 |
| 10 | 输出滤波 | `yawDiffOutputFilter`（加强滤波抑震荡） | 无 | 外层保留 |
| 11 | 单位约定 | PositionPID 输入 deg/s、输出 deg/s²，物理边界转 rad/s² | 已删除的 CascadeCtrl 曾在接口内换算 | 历史规划，不再实施 |
| 12 | 四元数分量顺序 | `QuaternionMath.h` double `[w,x,y,z]` | `Quat4f` float `[w,x,y,z]` | 赋值处核对顺序 |

## 5 分阶段实施步骤

- **阶段 0（评审）**：本规划评审；更新 `AGENTS.md` 第 71 行（记录 2026-08-08 决策，推翻"不做双链切换"）。
- **阶段 1（已完成 2026-08-08）**：`TandemVec_CtrlParams.h` / `TandemVec_CascadeCtrl.h` 加"未接入固件"警示注释。
- **阶段 2（宿主机对照基准）**：将实机链参数（state_data.cpp 当前值）与行为固化为 test_host 基准用例，记录 PositionPID 链的输入→输出对照（含 5 轮调参史参数），作为接入后行为等价性判据。
- **阶段 3（代码接入）**：`flight_control.cpp` 层 1/2 替换为 AttitudeCtrl + RateCtrl 调用；处理 §4 风险项 6/7/9（yaw 直通、ATTITUDE_MODE、s_yaw_gain_sched 导出）；`TandemVec_CtrlParams.h` 默认值**抄入实机当前值**作为接入起点（不是设计值 4.0/8.0）。
- **阶段 4（验证）**：`test_host/run_all.sh` 全绿；`tools/verify_*.py` 全套重跑（verify_mix_axes / verify_yaw_gain_schedule / verify_w0_floor / verify_floor_consistency / verify_allocation_sign 等）；地面低功率测试 → 系留 → 自由飞（FCS README 安全流程 5 步）。
- **阶段 5（收尾）**：删除 state_data.cpp 的 PositionPID 姿态对象（高度/位置环可先保留）；更新 README / AGENTS.md 参数说明（单一定义源改为 `TandemVec_CtrlParams.h` + `TandemVec_Config.h`）。

## 6 验证与行为红线

- **红线逐条对照**（`docs/registers/行为保持红线清单.md` 同源原则）：帧 delta≤0.05s / 子步≤0.004s / τm=0.28s；差速公式 ωf=ω0√(1+Δω)；四元数误差 q_e=qCmd⁻¹⊗q 机体系约定；dMax=15°（0.2617994 rad）；dwMax=0.7；SAS 限幅；增益调度公式不变。
- **宿主机**：`test_host/run_all.sh` 全绿（cascade 14 项 + sim 闭环）。
- **实机**：FCS README「实机安全流程」5 步逐项执行。
- **回退**：任一环节失败可回退——`flight_control.cpp` 保留旧链代码（git 历史可恢复），阶段 3 应独立 commit 便于 revert。

## 7 参考文件

- `include/TandemVec_CtrlParams.h` — 接入后目标调参入口（当前已加警示）
- `include/TandemVec_AttitudeCtrl.h` / `TandemVec_RateCtrl.h` — 待接入的层 1/层 2
- `include/TandemVec_CascadeCtrl.h` — 顶层编排（层 3/4 处理见待决策项）
- `src/flight_control.cpp` 1110–1262 行 — 现有分配链（层 1 轴置换 → 层 2 调度 → allocateMoments）
- `src/state_data.cpp` 87–206 行 — 实机限幅/PID 参数（接入起点值来源）
- `AGENTS.md` 第 71 行 — 2026-07-29 旧决定（需更新）

## 8 待决策项

1. **CascadeCtrl 层 3/4 处置**：保持"仅供 test_host 全链仿真"，还是把现有层 3/4 逻辑（轴置换+调度+floor+门控）反哺进 CascadeCtrl 后整体使用？
2. **接入起点参数（历史）**：当时建议采用 2.5/0.25/0.8/0.20 等迁移前值；该方案已废弃，当前参数必须读取 `FlightCtrlParams.h`。
3. **yaw 直通接口形态**：CascadeCtrl 增加 `yaw_rate_direct` 输入标志（外环旁路），还是外部把 ω_ref.z 直接覆写后仍走内环？
4. **高度/位置环**：是否同步迁移（PositionPID 对象）到新参数体系，还是暂时保留 state_data.cpp 定义？
