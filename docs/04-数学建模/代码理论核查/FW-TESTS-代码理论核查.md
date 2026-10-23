# 代码理论核查记录 — fw-tests（固件宿主回归测试）

> 核查日期：2026-10-23 ｜ 执行：fw-tests（注释与理论核对 Agent）｜ 规范：[GOV-004](../../00-项目治理/GOV-004-代码注释与理论对照规范.md)
> 代码基线：工作区（HEAD + 各区域未提交改动；`TandemVec_FCS/test_host/**` 内 40 个已跟踪文件被修改，35 个未跟踪文件属既有新增测试与 `fw_sil/`）
> 理论来源：REF-120（捷联惯导）、REF-121（卡尔曼滤波）、MOD-002（坐标系与符号）、
> CTL-001（控制架构与 SAS）、CTL-002（可控性与失效模式）、THY-002/003（推力/分配）、
> PROTOCOL-001（AnoCom）、COMM-001（数传链路）、`TandemVec_FCS/AGENTS.md` 踩坑记录
> 方法：逐文件读代码 → 对照被测符号与公式 → 补 L0/L1/L2 注释（不改行为）→
> `check-code-comments.py` 复检 → g++/node/py 语法与编译验证 → 汇总矩阵与缺口

---

## 1 范围与覆盖率

本区域由 `tools/check-code-comments.py::AREAS["fw-tests"]` 定义，范围为
`TandemVec_FCS/test_host/**` 内的 `.cpp/.h/.js/.py`（`bin/` 产物与 `*.sh` 不在检查范围）。

| 类别 | 文件数 | 行数 | 说明 |
|------|--------|------|------|
| C++ 回归测试/研究工具 | 48 | 10072 | `test_*.cpp`、`dps310_fifo_policy_test.cpp`、`task_scheduler_interval_test.cpp`、`optimizer_runner.cpp` |
| 测试基础设施头 | 1 | 269 | `mpc_emb.h`（被测原型头，非桩） |
| SIL 入口/链接桩 | 3 | 673 | `fw_sil_main.cpp`、`fw_sil_env_main.cpp`、`fw_sil_stubs.cpp` |
| SIL 桩头 | 5 | 443 | `fw_sil/stub/{Arduino,SPI,Stream,Wire,ubx}.h` |
| 测试桩头 | 2 | 100 | `stub/{Arduino,WS2812Driver}.h` |
| JS 工具链测试 | 2 | 171 | `dps_profile_{parser,runner}_test.js` |
| Python MC 驱动 | 4 | 644 | `run_*_mc.py`（无断言，产出 CSV/JSON） |
| **合计** | **65** | **12372** | 本轮新增注释约 1100 行 |

L0 覆盖与引用：

| 指标 | 核查前 | 核查后 |
|------|--------|--------|
| L0 文件头（`@file/@module/@brief/@details/@theory/@frame/@units`） | 0 / 65（4 个 py 有残缺口） | 65 / 65 |
| `@ref` 引用（章节/式号/页码格式） | 0 处 | 84 处 |
| `@test` 用例索引（文件头汇总用例编号） | 0 处 | 49 处 |
| lint error / warning | 65 / 13 | 0 / 0 |

> 注：`@ref` 页码一律按 VALID 记录；无法确认页码处只写章节（符合 GOV-004 §6.2 第 2 条）。
> 本区域**没有**理论关键文件（`THEORY_GLOBS` 不含 `test_host/`），R07 不适用。

---

## 2 核对矩阵

断言强度定义：

- **强**：与解析解、理论值或不变量对比，容差有量级依据；实现写错会失败。
- **中**：只验证方向、单调性、收敛、门控或边界行为，无绝对参考值。
- **弱**：无断言（只打印/基准/产出数据），失败需人读表或由外部脚本判定。

### 2.1 导航纯函数、静止辅助与传感器数学

| 测试文件 | 被测符号/模块 | 理论性质 | 强度 | 缺口/建议 |
|---|---|---|---|---|
| `test_gnss_dynamic_weight.cpp` | `ins_gnss_dynamic_weight.h`：配置校验、`Icm45686ComputeGnssStd` 类纯函数 | GNSS 量测噪声阵 R 的 floor/cap/pDOP 缩放与 fix 门控（工程整定，非教材式） | 强 | 未覆盖多星座/fix 抖动；R 地板与实机接受率的关系仍靠报告证据 |
| `test_static_aid_profile.cpp` | `ins_static_aid_profile.h` 全部 constexpr 函数 | 静止辅助周期 GCD/相位碰撞、强度三档调度、ZUPT 噪声自适应分段、外部参照门控 | 强 | 只验证纯函数；`navigation_task.cpp` 中实际相位是否错开无回归 |
| `test_static_detector.cpp` | `ins_static_detector.h`：`Icm45686StaticDetectorUpdate` | 迟滞状态机、置信度爬升、NaN 防御与可恢复性 | 中 | 真实振动/机动误判场景未覆盖（伪静止是历史事故点） |
| `test_altitude_reference.cpp` | `ins_altitude_reference.h` 四个换算函数 | NED 向下 → 控制向上、相对气压高度叠加起飞点、GNSS 重锚保持相对高度 | 强 | 未覆盖 GNSS 高度跳变/重锚瞬间的连续性 |
| `test_gnss_epoch_timing.cpp` | `ins_gnss_epoch_timing.h`：消费门控、重锚判据、iTOW→本地时间、micros 回绕 | GNSS 量测时标与年龄（REF-121 §2.2.1） | 强 | 硬件微秒计数器漂移与 PPS 对齐未覆盖 |
| `test_gnss_pps_latency.cpp` | `ins_gnss_pps_latency.h`：`InsGnssEstimatePpsLatency` | PPS 边沿与 PVT 到达差的中位数即链路延时；离散度给 σ | 强 | 硬件 PPS 捕获尚未接入固件，纯算法层证据 |
| `test_gnss_delay_auto.cpp` | `ins_gnss_delay_auto.h` 在线延时估计器 | 机动段曲率使 τ 可观测；野值剔除、失锁拒绝 | 中 | 理想 GNSS 噪声；多径/失锁抖动未覆盖 |
| `test_ins_vibration_adaptive.cpp` | `ins_vibration_adaptive.h`：`InsVibrationEstimator` | 0.5 Hz 高通 + 0.5 s RMS 分离振动与真实加速度 | 中 | 只测 20 Hz 单频；扫频证据在 MC 侧 |
| `test_bias_temp_observer.cpp` | `ins_bias_temp_observer.h` | 线性温漂斜率可辨 + 温度跨度门控 | 强 | 温度滞后/安装热梯度未覆盖 |
| `test_bias_temp_table.cpp` | `ins_bias_temp_table.h` | 两点线性插值解析值 | 强 | 越界外推行为未断言（由调用点保证） |
| `test_bias_temp_pipeline.cpp` | 观测器 + 温度表联合 | 同一线性模型下两条链路斜率一致 | 强 | 表采样窗口与在线窗口不同，容差已放宽到 5e-4 |
| `test_bias_jump_monitor.cpp` | `ins_bias_jump_monitor.h` | 静置不误报 + 缓漂可检出 | 中 | 阶跃幅度谱/持续时间谱未覆盖 |
| `test_rc_curve.cpp` | `RcCurve.h`：`rcNormFromUs`/`rcRateCurve` | Betaflight 三参数曲线：expo 中心压低、super 边缘双曲、rc_rate 线性缩放 | 强 | 实机手感与通道映射未覆盖（属调参范畴） |

### 2.2 EKF 15 状态与辅助测速

| 测试文件 | 被测符号/模块 | 理论性质 | 强度 | 缺口/建议 |
|---|---|---|---|---|
| `test_ekf_15state.cpp` | `ekf_15_state.h`：`Initialize`/`TimeUpdate`/`TimeUpdateTwoSample`/`MeasurementUpdate*Detailed`/协方差接口 | KF 五方程（REF-121 §2.2.1 p.33）、Joseph 形式（§2.2.4 p.42）、速度增量与旋转效应（REF-120 §9.6.1 p.298）、NIS 门控（REF-121 §7.3.1 p.255）、3+3 子集回退与软重锚 | 强 | T8/T9–T11 用 `set_covariance_for_test` 注入协方差，覆盖算法分支而非真实协方差演化；实机噪声谱/多径未覆盖 |
| `test_ekf_aiding_velocity.cpp` | `MeasurementUpdateAidingVelocityDetailed`/`ResetVelocityToExternal`/ZUPT 路径 | aiding 更新豁免零速守卫、30 m/s 残差门、失锁重锚（REF-121 §8.2.3 p.291） | 强 | 实机光流尺度/延迟未覆盖（`fw_sil --flow` 有实现但无断言） |
| `test_vertical_kf.cpp` | `VerticalKF.h` 三状态 KF | 预测/更新、3σ 门限、输入饱和、Joseph 协方差、P 上限 | 强 | 气压计真实噪声谱与更新率耦合未覆盖 |
| `test_ekf_delay_highspeed.cpp` | `MeasurementUpdateDetailed(..., age)` 回放融合 | 高速时延补偿（30 m/s × 50 ms ≈ 1.5 m 偏差量与 GNSS 噪声同量级） | 强 | 匀速直线、无振动；实机需 PPS 标定实际延迟 |
| `test_ekf_delay_mmae.cpp` | 11 候选 age 的 MMAE 原型 | 新息统计量作假设似然（NIS ~ χ²(m)） | 弱 | 研究原型，无断言；匀速段不可观测属理论预期 |
| `test_ekf_highspeed_realistic.cpp` | EKF + 振动自适应 Q + 延时自动估计的仿真台 | 标称传感器噪声下的闭环误差指标 | 弱 | 参数为 1-based 顺序映射，错位会静默改语义（AGENTS 已登记）；无断言，由 MC 驱动消费 |
| `test_ekf_bias_static.cpp` | 零偏状态 + static gyro/gravity/ZUPT 组合 | 静止窗口是零偏唯一强观测段；误检静止会污染零偏 | 弱 | 仅打印，无断言；建议加“真值误差带”门限 |
| `test_gnss_delay_batch.cpp` | 本文件 `fitResidual()` 原型 | 批处理对齐：固定 τ 时对 (p0,v0,b_a) 线性；仅机动段可观测 | 中 | 本体只打印误差，判定在 Python 驱动侧 |
| `test_gnss_delay_batch_vel.cpp` | `fitVelocity()` 原型 | 速度通道信噪比高于位置通道（0.05 m/s vs 1.5 m） | 中 | 同上 |
| `test_gnss_delay_batch_ekf.cpp` | EKF 零偏估计 + 批处理 | 延时与零偏强耦合，需先估零偏 | 中 | 同上 |

### 2.3 控制、分配与执行器

| 测试文件 | 被测符号/模块 | 理论性质 | 强度 | 缺口/建议 |
|---|---|---|---|---|
| `test_tandemvec_allocation.cpp` | `TandemVec_ControlAllocation.h`（`allocateMoments`/`allocateDifferential`）、`TandemVec_Propulsion.h`、`AirframeModel.h` | u=B⁺τ 与对角分配、差速开方分配平方和不变（CTL-001 §7）、推力/反扭（THY-002 §2–3）、往返一致、数据驱动 B 等价 | 强 | 策略切换连续性只在同一输入下比较（无时间维）；τ₀ 交叉项只取少数工作点 |
| `test_flight_control_axis.cpp` | `flight_control.cpp` 姿态环（按同一控制律重建）+ `kFlightCtrlParams` + `computeWrench` | SAS/姿态环（CTL-001 §4）、轴映射（AGENTS 踩坑记录）、四元数右乘误差（MOD-002 §3）、R13 垂直投影、6DOF 悬停 | 强 | **直连轴序 ≠ 实机 mix 层置换**（README 已声明）；无气动/风 |
| `test_position_pid.cpp` | `PositionPID.h` | PID 五特性：积分分离、积分钳位、反算抗饱和、无扰切换、2-DOF 加权 | 强 | 变步长/dt 抖动未覆盖 |
| `test_inertia_decoupling.cpp` | `InertiaDecoupling.h` | ω×(Iω) 与 ω×h 前馈、掩码开关、闭环收益 | 强 | 闭环收益是理想刚体，未含 τm 滞后 |
| `test_airframe_generic.cpp` | `AirframeModel.h`（几何表、数值 Jacobian、`computeWrenchGeneric`） | 换机型只填几何数据；B 由数值差分生成；伪逆往返 | 强 | 只验证四旋翼 demo；本机 B 等价由 allocation 测试覆盖 |
| `test_control_units.cpp` | `ControlUnits.h` + `kFlightCtrlParams` | 速率环量纲迁移（deg/s→deg/s²）前后物理输出等价（AGENTS 记录） | 强 | `kd` 非零时的迁移未覆盖（当前 kd=0） |
| `test_online_id.cpp` | `TandemVec_OnlineID.h`（RLS + 增益调度） | 惯量比可辨识性、零激励不更新、约束边界 | 中 | 激励只有 15° 阶跃；正弦/噪声激励未覆盖 |
| `test_actuator_fault_monitor.cpp` | `ActuatorFaultMonitor.h` | 舵机卡死/单发推力损失的“误差+饱和+持续时间”判据与优先级（CTL-002 §3） | 中 | 真实舵机电流/堵转特征未覆盖；F4 属硬件单点（只能检测+有界下降） |

### 2.4 通信、注册表、灯效、调度、气压计

| 测试文件 | 被测符号/模块 | 理论性质 | 强度 | 缺口/建议 |
|---|---|---|---|---|
| `test_anocom_packing.cpp` | `AnoComProtocol`：buildFrame/sendData/beginGroup/endGroup/高层 sendXxx | 帧格式与双校验（PROTOCOL-001 §3）、长度保护、组模式单次 write（COMM-001） | 强 | 组缓冲恰好 256B 的 flush 边界未全覆盖；`handleAnoCom` 双环频率分档无回归 |
| `test_ano_vars.cpp` | `AnoVars.h` 注册表契约 | 类型→float 序列化、9 字符截断、watch 集合语义（PROTOCOL-001 §4 的 0xF2/0xF3） | 强 | MAVLink 名称截断唯一性由固件侧保证，无宿主测试 |
| `test_led_status.cpp` | `Ws2812AppStatus.h` | 上电自检序列、13 状态配色/节奏、故障覆盖、每拍渲染 | 中 | 真实灯带时序/颜色顺序未覆盖（mock 只记录像素） |
| `task_scheduler_interval_test.cpp` | `task_scheduler_interval.h` | 小数分频（Bresenham 误差累加）、平均间隔=请求值、亚 tick 钳位 | 强 | 真实 tick 抖动与优先级抢占未覆盖 |
| `dps310_fifo_policy_test.cpp` | `Dps310FifoPolicy.h` | FIFO 取数策略：温压配对、读取上限、停采、可恢复错误才 flush | 强 | 真实 SPI 时序与 FIFO 溢出行为未覆盖 |
| `test_mag_parse.cpp` | `MagMath.h` + `tilt_compass.h` | 灵敏度换算、小端补码、芯片系→FRD、北半球 z 分量为正的物理判据 | 强 | 备份磁力计/第二芯片路径未覆盖 |
| `test_mag_cal.cpp` | `MagCalMath.h` | 椭球拟合（硬铁/软铁）、Jacobi 特征分解、质量门与拒绝路径 | 强 | 实机电机电流磁干扰未覆盖 |
| `test_mag_guide.cpp` | `MagCalGuide.h` | 两圈引导的状态机、姿态/转速门、扇区跨 ±π | 中 | 实机姿态源噪声与操作者节奏未覆盖 |

### 2.5 研究型仿真与工具（非回归闸门）

| 文件 | 被测/用途 | 性质 | 强度 | 缺口/建议 |
|---|---|---|---|---|
| `test_comprehensive_sim.cpp` | 单轴闭环增益扫描 + 模型偏差 | 执行器权限主导响应；物理逆解对惯量偏差鲁棒 | 中 | 研究输出为主；结论不外推实机包线 |
| `test_qual_analysis.cpp` | 增益定性分析（Q1–Q5） | 关键增益识别、α_max 瓶颈、积分阈值 | 中 | 同上 |
| `test_robustness.cpp` | 大机动 + MC 参数扰动（R1–R4） | 大角度可收敛、参数扰动有界 | 中 | 同上 |
| `test_advanced_theory.cpp` | Lyapunov/ADRC/辨识（L1–L3、B1–B7） | 二次型单调下降、ESO 扰动估计与抵消、二阶辨识 | 中 | 名义参数理论分析，非实机结论 |
| `test_optimal_gains.cpp` | 二阶系统解析最优增益 | ωn²=Kp_r·Kp_a、2ζωn=Kp_r | 弱 | 无断言（研究输出），不在 run_all 内 |
| `test_integrator_compare.cpp` / `test_integrator_sweep.cpp` | 外环/内环积分对比与 Ki 扫描 | 积分位置对扰动抑制与超调的权衡 | 弱 | 无断言；评分权重是自定工程准则 |
| `test_mpc_emb.cpp` | `mpc_emb.h` 求解器 | 滚动优化 + 框约束投影；单拍耗时预算 | 弱 | 基准型，未接入固件控制链 |
| `optimizer_runner.cpp` | Python 优化器的目标函数 | 单轴闭环代价/峰值/调节时间 | 弱 | 输出格式被 Python 正则依赖；无断言 |
| `dps_profile_parser_test.js` | `tools/dps_profile_parser.js` | 日志窗口聚合与字段提取 | 强 | 需 node，未纳入 `run_all.sh` |
| `dps_profile_runner_test.js` | `tools/dps_profile_runner.js` | label 清洗、参数解析、platformio 配置注入 | 强 | 同上；含“不得出现的宏”负断言 |
| `fw_sil_main.cpp` | 真实固件控制链 + 单刚体 plant | 定高/手动阶跃闭环 | 弱 | 只打印，无断言；README 的 10.00 m 结论未自动判定 |
| `fw_sil_env_main.cpp` | 真实固件 + 6DOF + 传感器/风/故障注入 | 定高（直注/EKF）、手动、故障、光流 | 弱 | 只打印；多 seed 稳定性靠 `run_matrix.sh` |
| `run_delay_closed_loop_mc.py` | 延时补偿 MC 驱动 | 延时估计收敛与改善量 | 弱 | 无断言；`--bin` 默认写死 Windows 临时路径 |
| `run_gnss_delay_large_mc.py` | 延时批处理大规模 MC | τ 估计误差分布 | 弱 | 门限（20 ms 通过率）在报告侧，不在脚本内 |
| `run_vibration_adaptive_mc.py` | 振动自适应 Q 配对 MC | 固定 Q vs 自适应 Q | 弱 | 无断言；需成对跑才有统计意义 |
| `run_vibration_policy_mc.py` | 策略 C 大规模 MC | 不超过 5% 成对退化 | 弱 | 同上；CSV 字段与历史数据兼容 |

### 2.6 测试基础设施（替身，不对应理论）

| 文件 | 作用 | 与真实硬件/上游差异 |
|---|---|---|
| `stub/Arduino.h` | 让依赖 `Arduino.h` 的头文件在宿主机编译 | 无时间基准、无引脚 I/O、无 Serial |
| `stub/WS2812Driver.h` | 记录最近像素与 show 次数 | 无 BSRR/DMA 时序、无颜色顺序换算 |
| `fw_sil/stub/Arduino.h` | SIL 用的更大替身（引脚宏/类型/空实现） | 不模拟外设行为，SIL 自己注入状态 |
| `fw_sil/stub/{SPI,Wire,ubx}.h` | 外设/驱动空实现 | 无时序、无总线错误、恒“在线” |
| `fw_sil/fw_sil_stubs.cpp` | 补齐链接所需符号 | 传感器/日志类构造函数为空，`Crc8::calc` 恒 0 |

---

## 3 覆盖边界（仿真口径 vs 实机）

1. **全部为宿主/离线口径**：没有任何测试接触真实 IMU/GNSS/气压计/舵机/电机，
   传感器输入均为合成或注入；`fw_sil/` 是真实固件源码 + 理想 plant，仍不是
   `pio run` 二进制、不是 QEMU/Renode、不是实机。
2. **执行器与气动**：plant 侧只有一阶电机滞后与推力/反扭，无桨叶气动、无风扰模型
   （`fw_sil_env_main` 的侧风以等效 NED 力注入）；舵机在 SIL 中另加 300°/s 速率限幅与
   0.1% PWM 量化，但与真实舵机的死区/回差/温度漂移不同。
3. **传感器误差模型**：ICM-42688/UBX PVT 用型号标称量级，不是实机标定值；
   无振动谱形状、无磁干扰、无 GNSS 多径、无杆臂安装误差（杆臂只在仿真里显式参数化）。
4. **轴序**：`test_flight_control_axis.cpp` 是**直连轴序**，与实机 `mix` 层的
   存档系→模型系置换不同构；该差异已在 README 与文件头显式声明，专项核对前不引入置换。
5. **时标**：GNSS 延时用编译常量或合成延时，真实链路延迟需 PPS 标定；
   `ins_gnss_pps_latency.h` 的硬件捕获尚未接入。
6. **通信**：只验证打包/校验/组模式与变量注册表；`handleAnoCom` 的双环频率分档
   （COMM-001）、MAVLink 互斥切换、0xF2/0xF3 实机时序均无宿主回归。
7. **参数来源**：控制增益读 `kFlightCtrlParams`（唯一事实源），但测试内仍有
   局部常数（惯量、kT/kQ 取自 `TandemVec_Config.h`）；`models/aircraft-model.json`
   的仿真参数不在本区域验证。

---

## 4 发现清单

| 严重度 | 位置（符号） | 问题 | 影响 | 建议 |
|--------|--------------|------|------|------|
| P1 | `test_ekf_bias_static.cpp`、`test_ekf_delay_mmae.cpp`、`test_ekf_highspeed_realistic.cpp`、`fw_sil_main.cpp`、`fw_sil_env_main.cpp` | print-only / 无断言，回归价值依赖人读表或外部脚本 | 这些链路“跑得过”不等于“没退化” | 为关键指标建立 golden 基线（如定高 10.00 m ±0.05、延时估计误差 ≤20 ms）并加断言 |
| P1 | `test_gnss_delay_batch*.cpp` 与 `run_gnss_delay_large_mc.py`、`run_vibration_*_mc.py` | 判定门限在报告/脚本侧，测试本体只产出数据 | “通过率”结论不可由单一命令复现 | 把门限写进脚本并输出退出码 |
| P1 | `test_ekf_15state.cpp` 的 T8/T9–T11 | 依赖 `set_covariance_for_test` 等仅供宿主的注入接口 | 覆盖算法分支，不等于真实协方差演化下的行为 | 补一条“自然演化 + 注入故障”的端到端用例 |
| P1 | `test_flight_control_axis.cpp` | 直连轴序与实机 mix 置换无对照测试 | 轴映射回归只能靠实机打杆 | 专项核对后增加“置换层”单测（需先统一 README/flight_control/state_data 三处注释） |
| P1 | 构建环境 | 中文仓库路径下 g++ 写二进制失败（MSYS 路径转换） | CI/新机器直接按 README 命令会失败 | 保持 `BIN_DIR`/`-o` 指向 ASCII 目录，并在 README 写明 |
| P2 | `test_host/README.md` 的“断言数”表 | 数字未自动化，改动后易漂移 | 文档与实测不一致 | 由 `run_all.sh` 输出实际 PASS 数并与 README 对账 |
| P2 | `dps_profile_*_test.js` | 需 node，未纳入 `run_all.sh` | 工具链回归靠人工执行 | 在 `run_all.sh` 末尾加可选 node 段（缺 node 时跳过并告警） |
| P2 | `dps310_fifo_policy_test.cpp` 的 `MockFifo` | 只按脚本给样本，不模拟真实 FIFO 满/溢出 | 硬件错误恢复未被证明 | 补“溢出后恢复”用例 |
| P2 | `CTL-001 §9 测试覆盖指针` | 只指向 `simulations/` 的 mjs 测试 | 固件侧测试与规范文档未互链 | 在 CTL-001 §9 增加本区域测试指针（或交叉引用本文件） |

---

## 5 已知失败与环境依赖

1. **EKF 大回归基线失败（既有）**：`TandemVec_FCS/test/ekf_host_regression.cpp` 的
   project 场景在 HEAD 基线即挂 3743 个断言，与本轮注释改动无关，未纳入 `run_all.sh`；
   待单独定位（见 `TandemVec_FCS/AGENTS.md` 2026-08-21 条）。
2. **EKF 头文件注释破坏编译（本轮已修复）**：2026-10-23 22:14 前后
   `lib/navigation-main/src/ekf_15_state.h` 的 Doxygen 块中出现 `subset_*/NIS`，
   `*/` 提前闭合注释导致所有包含它的 EKF 测试编译失败；已由 fw-ekf 改为
   `subset_* 与 NIS`，本轮复验 7 个 EKF 测试全部编译通过。
   **教训**：注释里不要出现 `*/`、`/*` 与行尾续行符。
3. **中文路径**：`g++ -o /c/Users/...` 报 `ld: cannot open output file`，
   改用 `C:/Users/...`（Windows 盘符形式）或 `BIN_DIR` 指向 ASCII 目录即通过。
4. **依赖外部二进制**：四个 `run_*_mc.py` 通过 `subprocess` 调用已编译的测试可执行体，
   `--bin` 默认值写死本机 Windows 临时路径，换机器必须显式传参。
5. **依赖第三方库**：EKF 组需 Eigen（`lib/eigen/src`）与 `lib/units/src/convang*.cpp`，
   并需 `-DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES`；缺宏时链接失败而非编译报错。
6. **SIL 需 MSYS bash**：`fw_sil/build_and_run.sh` 依赖 `/c/...` 形式路径与 `BIN_DIR`，
   在纯 PowerShell 下需手工拼命令。

---

## 6 未决问题

- `run_all.sh` 是否应纳入 `fw_sil/` 两个入口与两个 `.js` 测试？当前它们是“手动/专项”集合。
- 是否为 print-only 研究测试建立统一 golden 数值夹具（放 `test_host/baselines/`）？
- `test_ekf_highspeed_realistic.cpp` 的 1-based 命令行参数映射是否需要改为具名参数
  （错位会静默改变整批 MC 语义，历史上已导致一轮结果撤回）？
- CTL-001 §9 是否补固件侧测试指针，形成“文档 ↔ 测试”双向索引？

---

## 7 门禁与验证证据

### 7.1 注释门禁

```bash
py -3.12 tools/check-code-comments.py --area fw-tests --check
```

| 阶段 | 范围 | error | warning | 平均分 | 退出码 |
|------|------|-------|---------|--------|--------|
| 核查前 | 65 文件 | 65 | 13 | 79.0 | 1 |
| 核查后 | 65 文件 | **0** | **0** | **100.0** | **0** |

规则命中变化：R01 65→0、R04 5→0、R05 3→0、R06 1→0、R09 4→0。

### 7.2 编译/语法验证

编译输出统一落在 ASCII 临时目录（规避中文路径），**只编译不运行**：

- **C++：51 个 `.cpp` 全部编译通过**（48 个测试/研究工具 + 3 个 SIL 文件；
  `mpc_emb.h` 经 `test_mpc_emb.cpp` 间接覆盖）。分两批：
  46 个走 `-Iinclude` 类单文件命令，`test_optimal_gains.cpp`、`test_integrator_compare.cpp`
  单独补编译，`fw_sil_main.cpp`/`fw_sil_env_main.cpp`/`fw_sil_stubs.cpp` 按
  `build_and_run.sh`/`build_and_run_env.sh` 的源码清单编译并链接。
- **改动最大的 10 个**（均已 `[BUILD OK]`）：
  `test_flight_control_axis.cpp`、`test_tandemvec_allocation.cpp`、`test_position_pid.cpp`、
  `test_ekf_15state.cpp`、`test_ekf_aiding_velocity.cpp`、`test_static_aid_profile.cpp`、
  `test_ekf_highspeed_realistic.cpp`、`test_ekf_delay_highspeed.cpp`、
  `test_gnss_delay_batch.cpp`、`test_mag_cal.cpp`。
- **EKF 组（7 个）**：`test_ekf_15state`、`test_ekf_aiding_velocity`、
  `test_ekf_delay_highspeed`、`test_ekf_highspeed_realistic`、`test_ekf_delay_mmae`、
  `test_ekf_bias_static`、`test_gnss_delay_batch_ekf` 全部通过（修好头文件注释后）。
  Lead 侧复验 `test_ekf_15state` 运行 **36/36 PASS**。
- **JS**：`node --check` 通过 2/2。
- **Python**：`py -3.12 -m py_compile` 通过 4/4（`PYTHONPYCACHEPREFIX` 指向临时目录，
  未在仓库生成 `__pycache__`）。

代表性编译命令（仓库根目录，`TandemVec_FCS/` 为工作目录）：

```bash
# 控制/分配（无 Eigen）
g++ -std=c++17 -Iinclude test_host/test_flight_control_axis.cpp -o "$TEMP/fca.exe"
# EKF 15 状态（需宏 + units 源）
g++ -std=c++17 -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES \
    -Iinclude -Isrc -Ilib/navigation-main/src -Ilib/eigen/src -Ilib/units/src \
    test_host/test_ekf_15state.cpp lib/units/src/convang*.cpp -o "$TEMP/ekf15.exe"
# 磁力计标定（需 MagCal 头路径）
g++ -std=c++17 -Iinclude -Ilib/MagCal test_host/test_mag_cal.cpp -o "$TEMP/mc.exe"
# SIL（与 build_and_run.sh 同源码清单，只链接不运行；INC 含 fw_sil/stub 与 lib 各子目录）
g++ -std=c++17 -O2 -DTANDEMVEC_FIRMWARE -Wno-address-of-packed-member -Wno-psabi \
    -Itest_host/fw_sil/stub -Iinclude -Isrc -Itest_host/stub \
    -Ilib/navigation-main/src -Ilib/units/src -Ilib/AnoComProtocol -Ilib/MAVLink \
    src/flight_control.cpp src/state_data.cpp test_host/fw_sil/fw_sil_stubs.cpp \
    test_host/fw_sil/fw_sil_main.cpp \
    lib/DPS310-Pressure-Sensor/src/Dps310.cpp lib/DPS310-Pressure-Sensor/src/DpsClass.cpp \
    -o "$TEMP/fw_sil.exe"
```

### 7.3 行为保持（自查方法与限制）

本轮只新增/改写注释与文档：未修改任何断言、期望值、测试数据、超时、宏开关与编译选项。

- 全部改动通过带锚点的 `edit` 逐处插入/替换，插入内容均为 `/** … */` L0 文件头、
  `@test` 用例索引或 `//` 行内说明；`edit` 的 `old_string` 中若有代码行，
  都在 `new_string` 中原样保留（如 `#include`、`#pragma once`、`int main`、`HardwareSerial Serial;`）。
- 被替换的仅是各文件原有的头注释块（含 5 处被 R04 判为“注释掉的代码”的公式行，
  改为 `@formula` 标注，语义不变）。
- 验证口径：51 个 `.cpp` 全部重新编译通过（§7.2）+ lint 65 文件 0 error（§7.1），
  说明注释未破坏任何编译单元。
- **限制**：`git diff -- TandemVec_FCS/test_host` 同时包含**此前会话**的未提交功能改动
  （例如 `test_ekf_15state.cpp` 的 T7–T11、`test_static_aid_profile.cpp` 的门控用例），
  因此不能单凭整段 diff 判定“本轮只改注释”；上一条逐处核对方法是本轮的自查依据。
  若需机器级证明，应在改动前保存工作区快照做逐文件对比（本轮未留存，记为流程改进项）。
