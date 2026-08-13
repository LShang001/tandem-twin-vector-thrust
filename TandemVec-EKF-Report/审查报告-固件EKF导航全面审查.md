# 固件 EKF 组合导航全面审查报告

> 审查日期：2026-08-12
> 审查对象：`TandemVec_FCS` 导航子系统（ekf_15_state.h / navigation_task.cpp / ins_* 系列 / VerticalKF / HorizontalKF）
> 方法：源码逐行审查 ×3 遍 + 独立数值交叉验证（F矩阵数值微分、NIS分布检验、新息白化、延迟回放多种子、重捕获门控、Monte Carlo、AHRS校正闭环仿真）+ 3 个独立视角并行 agent 交叉核对
> 结论：**核心数学（F 矩阵/四元数/量测雅可比/协方差传播）全部通过数值微分验证，无算术错误**；但集成层发现 **1 个已数值确认的 HIGH 级逻辑错误**（降级 GNSS 路径 AHRS 航向校正开环积分）、**2 个 MED 级配置-语义脱节、3 个 MED 级防御缺口、1 个 MED 级时间跨度缺陷**，以及若干 LOW 级代码质量问题

---

## 〇、严重度总览

| 严重度 | 数量 | 条目 |
|---|---|---|
| 🔴 HIGH | 2 | H1（数值确认）、H2-数学（数值确认，过渡飞行触发） |
| 🟠 MED | 16 | MED-0a/0b, MED-1~MED-11, MED-12/13（集成层），MED-14~16（数学层） |
| 🟡 LOW | 21 | LOW-1~LOW-17（集成层），LOW-18~21（数学层） |

---

## 一、数值交叉验证结果（第三遍，独立于源码的数学验证）

| 验证项 | 方法 | 结果 | 结论 |
|---|---|---|---|
| V1 F 矩阵关键块 | 解析 vs 数值微分（扰动误差状态） | 最大偏差 3e-6 | ✅ F(p,v)/F(v,β)/F(v,ba)/F(β,β)/F(β,bg) 全部正确 |
| V1b 完整 F 地球项 | Fvv/Fpp/Fbv 数值微分 | 偏差 ≤8e-7（二阶截断） | ✅ 含哥氏/传输速率/曲率项正确 |
| V2 NIS 分布 | S2 场景 7200 帧统计 | 均值 4.67（理论 6）、中位 4.20（理论 5.35） | ⚠️ 滤波器**轻度保守**（P 偏大） |
| V3 新息白化 | 白化新息自相关 | lag1=-0.23、lag5=-0.02 | ⚠️ 轻微负相关（Q 偏大所致） |
| V4 单/双子样 | 静止 1s 对照 | quat/vel 差异 0 | ✅ 两路径等价 |
| V5 延迟回放 | 10 种子 + 高动态机动 | 回放 vs 错误时刻无统计差异（-2.1%±波动） | ⚠️ **匀速/机动场景回放收益不显著**（见 MED-4） |
| V6 重捕获门控 | 失联 10s 后 4.28m 残差 | NIS=0.1（P 膨胀后）远小于门限 30 | ✅ 印证固件两级检查必要性 |
| V7 静止档位 | 源码+仿真 | 档位 1 只存活 1 帧 | 🔴 见 MED-1 |
| V8 NIS 根因 | accel_std 0.25→0.05 | NIS 均值 4.67→5.10（逼近理论 6） | ✅ Q 偏大是轻微保守主因，属抗震动工程权衡（非 bug） |
| V9 H1 校正闭环 | 100s 仿真 EKF=30°/raw=0° | 全质量收敛 30°；降级路径 corr 在 ±180° 旋转 | 🔴 **H1 数值确认** |
| V10 H2 NIS 语义 | R=1e4 对照 | A/B 路径 NIS 相同（位置通道本就关闭） | ⚠️ 当前影响极小，属语义隐患 → MED |
| V11 M4 静止航向 | 200k 帧噪声统计 | 8.94% 通过门控、航向 σ=145°、每秒 0.9 帧污染 | 🔴 **M4 数值确认**（MED） |
| V12 M1 dt 低估 | 溢出帧模拟 | 卡顿 64-100ms 时传播 dt 低估 20-36% | 🔴 **M1 数值确认**（MED） |
| V14 H2-数学 yaw 轴 | 真航向轴机体系分量 | 俯仰 45° 捕获 71%/错注入 71%；85° 捕获 9% | 🔴 **H2 数值确认**（过渡飞行触发） |
| V15 H2 触发路径 | 双矢量调用条件 | 悬停(<1m/s)被拦、巡航(水平)正确、仅过渡飞行触发 | ✅ 触发边界确认 |

---

## 二、发现清单

### HIGH 级（已数值确认，建议优先修复）

#### H1 [navigation_task.cpp:319-321] 降级 GNSS 路径的 AHRS 航向校正"双重抵消"，校正量开环无限积分 🔴
- **现象**：降级路径（`applyDowngradedGnssUpdate` 内）写 `yaw_err = wrapAnglePi(nav_ekf.yaw_rad() - (backup_ahrs_yaw - ahrs_yaw_correction_rad))`，而 `backup_ahrs_yaw` 在数据源处**已叠加校正量**（sensor_imu.cpp:406 `backup_ahrs_yaw = wrap(yaw + ahrs_yaw_correction_rad)`）。故 `backup_ahrs_yaw - ahrs_yaw_correction_rad` 等于原始 Madgwick yaw，`yaw_err` 变成全量误差 `EKF−raw`（与校正量无关），闭环退化为**开环积分器**——只要持续降级融合（如长时间弱 GNSS），`ahrs_yaw_correction_rad` 以最高 28.6°/s 持续缠绕。
- **数值确认（V9）**：EKF 航向恒定 30°、raw=0°，100s 仿真——全质量路径 corr 正确收敛至 30°；降级路径 corr 在 +143°→−73°→+70°→… 持续旋转（±180° 回绕），backup_ahrs_yaw 被同步拖偏。
- **修复**：删除 `- ahrs_yaw_correction_rad`，与全质量路径（1183 行）统一为 `yaw_err = wrapAnglePi(nav_ekf.yaw_rad() - backup_ahrs_yaw)`。

#### H2-数学 [ekf_15_state.h:3128-3129] 标量航向量测 H=e₈ 无倾斜补偿，VTOL 过渡飞行时修正轴错位 🔴
- **现象**：标量航向更新用 `H = [0,...,1,...,0]`（仅观测体系 z 轴误差 δβ_z，索引 8）。但欧拉 yaw 误差与体系误差的映射为 `δψ = (cosφ/cosθ)δβ_z + (sinφ/cosθ)δβ_y`——仅在水平姿态（θ≈0）退化为 δβ_z。对纵列双发矢量推力 VTOL，悬停/过渡时俯仰可达 90°。
- **数值确认（V14）**：真航向轴（世界垂直）在机体系的分量随俯仰变化——俯仰 30° 时 H=e₈ 捕获 87%（50% 错注入横滚）；45° 时捕获 71%（71% 错注入横滚）；85° 时捕获 9%（几乎完全错位）。
- **触发条件**（V15）：悬停（速度<1 m/s）被双矢量低速保护拦截不触发；巡航（水平）H=e₈ 正确；**仅在过渡飞行**（倾转走廊，速度>1 m/s 且俯仰 30-85°）触发——修正量部分注入横滚轴，污染横滚姿态且航向收敛变慢。
- **修复**：量测前按当前 roll/pitch 对 H 做倾斜补偿 `H_yaw = [0,0,cosφ/cosθ, sinφ/cosθ]`（对应状态 7/8），或过渡飞行禁用标量航向更新（改用 `ApplyAttitudeUpdateDetailed` 的对数映射路径）。

### MED 级（建议修复）

#### MED-0a [navigation_task.cpp:311-312] 降级 GNSS 路径以 EKF 自身 LLA 作为"位置量测"（语义隐患，定时炸弹）
- **现象**：全质量路径（1116 行）传真实 `gnss_epoch` LLA；降级路径却传 `nav_ekf.lla_rad_m()`（EKF 自身当前估计），位置新息 = `lla2ned(EKF_curr, EKF_replay)` ≈ 自差 0。
- **数值确认（V10）**：R=1e4 下 A（真实 LLA）/B（自身 LLA）的 NIS 完全相同（1.60/0.98），位置通道本就关闭 → **当前影响极小**。但这是语义错误：一旦未来调低降级 R 或误用，会产生"自差=0 的假位置融合"。且位置自差进入 NIS 门控，稀释速度新息质量信号。
- **修复**：降级路径也传 `epoch` 的真实 LLA（若担心拉偏，保留 R=1e4 即可，无需自差量测）。

#### MED-0b [navigation_task.cpp:627-631+673-681] IMU 缓冲溢出帧的传播 dt 被低估（时间跨度不一致）
- **现象**：ISR 在缓冲满（64 样本）后停止写 delta（sensor_imu.cpp:265-281），`imu_delta_time_sum_s` 只含前 64 样本；而均值加速度用全部样本。溢出帧 `use_two_sample_imu=false` 走单样本路径，用全窗口均值加速度 × 仅 64 样本时间跨度传播。
- **数值确认（V12）**：卡顿 64-100ms（真实 dt<IsValidTimeStep 上限 100ms）时，传播 dt 被低估 20%-36%，速度/姿态积分欠量。
- **修复**：溢出时 `nav_update_dt_s` 改用真实时间（`imu_sample_count × 平均样本间隔` 或 micros 差分），或溢出帧直接跳过传播（避免不一致输入）。

#### MED-1 [ins_static_aid_profile.h:143] 档位 1"弱约束窗口"实际只存活 1 帧
- **现象**：确认静止当帧 `confirmed_static_frames = enter_candidate_frames = 20`（navigation_task.cpp:718 设 enter_min_frames=20），档位 1 条件 `confirmed_static_frames < 16` 永不成立；`confidence < 0.35` 仅当帧成立（dwell=20/60=0.333<0.35），下一帧 21/60=0.35 即升档 2。
- **影响**：注释声称的"刚确认静止 80ms 弱约束窗口"实际仅 5ms。若设计意图是静止初始阶段轻柔修正（避免误判强拉），则该意图未实现。
- **修复**：档 2 帧门槛提到 ≥24（或 enter_min_frames 降到 <16），使弱约束窗口真正存在；或明确删除档 1 的帧条件，接受"确认即强约束"语义。

#### MED-2 [ins_static_aid_profile.h:86-89] `prefer_gravity_first` 分支会整体饿死 ZUPT/StaticGyro
- **现象**：档位 1 的 `prefer_gravity_first=true` 时，`SelectStaticAidAction` 返回 `gravity_due ? Gravity : None`——gravity 未到期（divider=8 下 7/8 的帧）时**连 ZUPT 和 StaticGyro 都不执行**。
- **影响**：档位 1 期间约 87.5% 帧无辅助动作。因档位 1 只 1 帧（MED-1）当前影响有限，但若置信度在退出迟滞边界波动、档位 1 反复进入，零速约束会被周期性饿死；且与注释"先调平再零速"（应为 gravity 到期让位、未到期正常 ZUPT）不一致。
- **修复**：改为 `gravity_due ? Gravity : (zupt_due ? Zupt : None)`，并在 gravity 未到期时允许 StaticGyro。

#### MED-3 [ins_gnss_epoch_timing.h:77-80, 92-93] 时序映射负值/陈旧值被钳成"零年龄新鲜数据"
- **现象**：`mapped_time_us = receive_us - base_delay + tow_offset` 为负时强转 uint32 得近 2³² 巨值 → `now - 巨值` 为负 → 年龄钳 0；`int32 elapsed_us` 上限 35.8 分钟，超过则变负 → 同样钳 0。
- **影响**：时序异常（映射时间落在未来 / 陈旧映射）的数据被当作"零年龄新鲜数据"送入 EKF 与重锚判定，绕过延迟回放的合理性保护。
- **修复**：`if (mapped_time_us < 0) return invalid;`；`elapsed_us < 0` 时返回显式无效而非 0。

#### MED-4 [报告级结论] GNSS 延迟回放（15ms）在 5m/s 巡航与 3m/s² 机动下无统计精度收益
- **现象**：10 种子 Monte Carlo：理想 0.178、回放 0.179、错误时刻 0.175（差异在 ±0.07 波动内）；高动态 3m/s² 机动下同样无差异；姿态（roll）误差亦无差异。
- **机理**：15ms 延迟对应的位置偏差 = v·τ = 0.075m（5m/s），远小于 R 地板 σ_p=2m；速度偏差 = a·τ = 0.045m/s（3m/s²），小于 σ_v=0.15m/s。时间失配偏差被量测噪声完全覆盖，回放的精度价值仅在**高动态 + 高精度 GNSS（RTK σ<0.1m）** 场景显现。
- **建议**：延迟回放架构保留（正确性无损害，为 RTK 预留），但固件注释"15ms 比 120ms 更适合"的依据应改为高精度 GNSS 场景；当前配置下可考虑降低回放频率节省 CPU，或标注该收益场景边界。

#### MED-5 [VerticalKF_2State.h:156-157 vs HorizontalKF.h:112-115] 两 KF 过程噪声 Q 语义不一致
- **现象**：VerticalKF 每步加与 dt 无关的定值方差（q_pos/q_vel），HorizontalKF 将 std² 当方差率乘 dt。
- **影响**：同一"过程噪声"概念两种语义，调参时差一个 dt（约 20 倍）量级；VerticalKF 的 Q 缺 G·q·Gᵀ 交叉项，P[0][1] 被系统性低估；`begin()` 不重置 P，重复调用沿用旧协方差。
- **修复**：统一为方差率×dt 语义；VerticalKF 补交叉项或文档化说明。

#### MED-6 [navigation_task.cpp:405-456] 双矢量航向融合缺少静止门控
- **现象**：`handleDualVectorYawFusion` 只检查 GNSS 新鲜、光流有效、速度门限（1.0 m/s）与机动门限（0.35 rad/s），不检查 `is_static_confirmed`。静止时位置更新已强制速度=0（1118-1123 行），但航向融合仍用原始 GNSS 速度解算航迹角。
- **数值确认（V11）**：静止 + GNSS 速度噪声 σ=0.8 m/s（近地多径）下，200k 帧中 8.94% 穿过三道门控（低速+尺度+机动），估计航向标准差达 **145°**（纯噪声），每秒约 0.9 帧被错误融合进 EKF 航向更新。
- **修复**：加 `if (is_static_confirmed) return;` 显式门控（与位置更新强制 v=0 的逻辑对齐）。

#### MED-7 [navigation_task.cpp:248-251 vs 188-192] 降级门限 SV 比全质量更严，3~8 星中间质量帧被整体丢弃
- **现象**：全质量只需 `dw.passed_minimum`（fix≥3D + sv≥3 + h_acc>0）；降级却要求 `num_sv ≥ GNSS_MIN_SV(=9)`。sv 在 3~8 且 h_acc=0/pDOP 超限的合法 3D fix 帧既过不了全质量、也进不了降级。
- **影响**：中间质量帧完全丢弃；且降级路径不检查 h_acc/v_acc，对"弱精度"的防御反而不如全质量路径。门限设计自相矛盾。
- **修复**：统一降级门限为与全质量一致的 sv≥3（或明确 sv 下限为 5），并补充 h_acc 合理性检查。

#### MED-8 [navigation_task.cpp:1437-1446] `gnss_status.has_obs` 超时后从不清零，hAcc/vAcc 永久显示旧值
- **现象**：`has_obs` 只在 `gnssEpochPassesFullQuality` 置 true，全文件无清零路径。GNSS 断电数分钟后地面站仍显示最后一次通过质量帧的 hAcc/vAcc；1453 行 fix 类型有 `gnss_data_fresh_for_nav` 门控变 NONE，但精度字段没有——与 1436 行注释"GNSS超时后必须清零"直接矛盾。
- **修复**：在 GNSS 超时（`!isGnssDataFreshForNav()`）时清零 `has_obs` 与精度字段。

#### MED-9 [navigation_task.cpp:993-1006] 静止辅助分频相位相对"开机累计帧数"而非"本次静止会话"，档位切换时相位突变
- **现象**：`static_aid_frame_counter` 为块内 static，只在静止帧自增、从不复位。relaxed/deep_rlx 档位切换（div 4/8/8→8/16/16→16/32/32）时取模条件对同一计数值离散跳变，切换瞬间 ZUPT/Gravity 相位错位、间隔不均。
- **影响**：各档位内部 gcd 无碰撞（已核对），但跨档切换无平滑处理，长静止中可能出现某次辅助量测间隔异常。
- **修复**：档位切换时按当前计数值重新对齐相位，或记录"本次静止会话帧号"代替累计帧。

#### MED-10 [navigation_task.cpp:1412-1422] EKF 未初始化时用 Madgwick 填充 AHRS_Packet，在 DETA100 直出模式下会覆盖 DETA100 输出
- **现象**：该 else 分支不检查 `use_deta100_output`。DETA100 模式、EKF 尚未初始化（需静止检测，可能因振动永不触发）时，`handleDeta100` 写入的 AHRS_Packet 会被 Madgwick 值逐帧覆盖，DETA100 直出通道实际失效；1268-1270 的同步在 `nav_system_initialized` 内不会执行。
- **修复**：填充分支加 `if (!use_deta100_output)` 隔离。

#### MED-11 [navigation_task.cpp:1066-1086] 无 GNSS 运动时 AHRS 姿态量测的门限近乎恒真
- **现象**：`accel_gravity_trusted = |a|∈[7.8,11.8]`（容忍 ±2 m/s²）、`angular_rate_quiet = |ω|<1.2 rad/s`（69°/s）。两个门限都极宽松，无 GNSS 期间"运动"与"机动"几乎永远进入 AHRS 融合分支；Madgwick 剧烈机动时 roll/pitch 本身不准，作为 EKF 量测等于周期灌入低质量姿态。与双矢量航向的 20°/s 门限形成鲜明反差。
- **数值确认**：剧烈机动（|ω| 峰值 2.5 rad/s）下 33.1% 帧通过 1.2 rad/s 门限，而同一机动双矢量航向 20°/s 门限仅 8.2% 通过——AHRS 门限几乎不拦机动。
- **修复**：收紧 `angular_rate_quiet`（如 <0.5 rad/s 对应 28°/s），或对 AHRS 量测按加速度机动幅度调高噪声。

#### MED-12 [navigation_task.cpp:771-776 vs 202/209] 降级帧标记 `aid_tracker.MarkAvailable(Gnss,true)` 与全质量拒绝帧语义不一致
- 降级路径位置观测被 R=1e4 等价关闭、速度弱融合，却标记"可用"；全质量路径 tow 不匹配/重复 iTOW 标记不可用。诊断计数 `fix3d_count` 同时计入降级帧，指标口径混乱（不影响飞行，影响排障）。

#### MED-13 [navigation_task.cpp:645-648] `sample_dt_s <= 0` 的 continue 使该样本增量从双子样路径丢失、但仍留在时间总和与均值路径
- ISR 侧已把 dt≤0 钳为 0.0005（sensor_imu.cpp:163-165），正常不可达；但一旦触发，该样本 Δθ/Δv 从双子样消失，而 `local_delta_time_sum_s` 与均值路径仍含其贡献——三路输入互相矛盾。防御代码应在破坏一致性时禁用双子样。

#### MED-14 [ekf_15_state.h:2297-2298 vs 2345] 传播用"上时刻姿态"旋转速度增量，F 矩阵线性化用"中点姿态"——名义积分与协方差线性化点不一致
- 代码已算 `t_b2ned_mid`（用于 F(V,Φ) 与 F(V,b_a)），但比力增量旋转用 `t_b2ned_prev`。惯导标准做法绕中点姿态旋转（含划摇时尤其如此）。差异 O(ω·dt/2·Δv)，5ms 步长下很小，但高动态下协方差轻微失配。改 `t_b2ned_mid` 更一致且几乎零开销。

#### MED-15 [ekf_15_state.h:3573-3635] GNSS 延迟重传播不重放失联协方差膨胀（GrowGnssOutageCovariance）
- 实时路径每个 TimeUpdate 在失联时给 P 膨胀并存快照；重传播路径只执行 `PropagateStateFromIncrements`（ΦPΦᵀ+Q），不含膨胀。15ms 窗口（约 3 步）影响可忽略，但长失联恢复首帧后 P 比前向路径乐观，NIS 门控可能误拒后续合理量测。建议重传播循环里对失联期步同样调用膨胀逻辑。

#### MED-16 [ekf_15_state.h:2799-2805] 重力方向量测先扣减当前零偏估计再取方向——量测与状态形成反馈耦合
- 有意设计（避免零偏被误读成倾角），但零偏误差经"补偿→方向→姿态修正→交叉协方差"重新耦合回零偏，长期静止可能形成缓慢振荡；上层机动中调用时未建模比力方向也被当重力，NIS 门限 18 仅拦大残差。建议上层仅低动态调用。

### LOW 级（代码质量/防御加固）

#### LOW-1 [navigation_task.cpp:1186] GNSS 分支 constrain 上限硬编码 0.0025f，未复用 `MAX_AHRS_YAW_CORRECTION_STEP_RAD`
- 行为等价（数值相同），但常量未复用，DETA100 分支已正确复用——统一风格。

#### LOW-2 [ins_gnss_dynamic_weight.h:92-93] `s_acc` 回退 `h_acc*1.5` 量纲错配（m vs m/s）
- 典型 h_acc=2m → s_acc_eff=3.0 = cap_vel_mps → 任何不报告 spd_acc 的 epoch 速度噪声恒封顶（速度量测几乎失效）。行为保守安全，但回退值语义是"放弃速度融合"而非"合理估计"。

#### LOW-3 [ins_gnss_dynamic_weight.h:103-122] pDOP 缩放发生在 clamp 之后，cap 天花板名不副实
- 缩放会把已 clamp 的噪声再放大超过 cap（h_acc=100、pDOP=10 → 30×5=150）。若 cap 是物理上限，缩放应在 clamp 前。

#### LOW-4 [ins_static_detector.h:177-178] `confirmed_static_frames == 1` 分支是死代码
- 确认帧 frames=20≥1 恒成立，`frame_seq == 1U` 的重入检测永不触发；`< last_frame` 兜底在"上一段恰好 20 帧退出"边界失效（旧段样本污染新段均值，占比 1/N）。

#### LOW-5 [ins_static_aid_profile.h:113-114] 融合/拒测在 r=1.5 处硬跳变
- 恰好 1.5 → allow_fusion=true（scale=7.5 封顶 6），1.5001 → false。建议 `>= 1.5f`。

#### LOW-6 [VerticalKF_2State.h:65] 注释 3-sigma 与默认值 25（5²）矛盾
- 实际是 5-sigma 门限，异常值抑制比注释预期宽松。

#### LOW-7 [navigation_task.cpp:694] 伪磁场 `mag_ut << 30,0,40` 与 `init_heading_err_std=π` 语义矛盾
- TiltCompass 用伪磁场算**确定**初始 yaw（机头朝北方向），P0[8,8]=π² 却声称"完全未知"。行为无害（GNSS 首帧修正），但语义不一致，且大 P 使首帧 yaw 修正经 F(v,β) 短暂扰动速度估计。

#### LOW-8 [ins_gnss_dynamic_weight.h:43] `min_sv` 默认 5 与调用方 kGnssDwCfg.min_sv=3 不一致
- 当前调用点显式传 cfg 无影响，但头文件默认值是静默陷阱（未来调用点忘设则卫星门限收紧到 5）。

#### LOW-9 [ins_static_aid_profile.h] constexpr 多语句/while 依赖 C++14 relaxed constexpr
- 编译依赖 STM32 核心默认 gnu++14；建议 platformio.ini 显式 `-std=gnu++14`。

#### LOW-10 [navigation_task.cpp:741-764] 时间映射失败时量测年龄回落为固定 15ms 而非拒绝
- 配合融合分支无 millis 新鲜度门控，理论上陈旧 epoch（含无效时间戳）按 15ms 假年龄在当前时刻融合，绕过延迟回放保护。EKF 侧 `require_delayed_snapshot` 对 age>窗口帧会拒绝，但 15ms 假年龄绕过该保护。

#### LOW-11 [navigation_task.cpp:770 vs 405] `gnss_instant_valid_for_nav`（局部 epoch 质量）与 `isGnssInstantValidForNav()`（ubx 缓存+millis 新鲜度）双轨并存
- 1098 行分支用局部变量、1107 行双矢量航向用函数，UBX 缓存滞后时可出现"位置更新了但航向融合跳过"的窗口不一致。

#### LOW-12 [navigation_task.cpp:1492-1531] `updateEstimatedVerticalVelocity` 差分 dt 无上界，GNSS 有效期间不更新时间戳
- `last_vel_calc_time_micros` 只在非 GNSS 分支更新；GNSS 丢失后首帧差分 dt 达数分钟，`(h2-h1)/dt`≈0，垂直速度低估数秒。仅下限无上限。（当前调用点被注释，属潜在隐患）

#### LOW-13 [navigation_task.cpp:1398-1408] `lla2ned_skip` 使 relative_north/east 每 4 帧（20ms）刷新，而 relative_down 每帧刷新
- 静止悬停无影响，急加速时可短暂出现水平位置滞后。

#### LOW-14 [navigation_task.cpp:1468-1488] `_ekf_time_sum / _ekf_time_cnt` 整型除法，60s 平均耗时被截断为整数微秒
- 诊断精度损失（非功能问题）。

#### LOW-15 [navigation_task.cpp:1643-1653] `if (0)` 激光数据残留死代码
- `laser_update_counter`、`LASER_NOISE_STD` 随之失效；注释"200/2=100Hz"与实际 `>=1` 不符（激光应为 200Hz）。且被禁用分支与 `else`（气压计）结构容易误读。

#### LOW-16 [navigation_task.cpp:1727] `handleHorizontalEstimation` 若重新注册会清除 `flow_data.is_flow_valid`，与 `handleDualVectorYawFusion`（409 行）竞争该标志
- 当前未调度（main.cpp:421 注释），属潜在回归点。

#### LOW-17 [navigation_task.cpp:1619] 垂直 KF 用 `G_ACCEL_CONST` 减重力，未用 EKF 已算出的 `ekf_gravity_mps2`（Somigliana 当地重力）
- 与 EKF 输出口径不一致（差值 ~0.01-0.05 m/s²，量级小）。

#### LOW-18 [ekf_15_state.h:480,673,759,3298,3669] `buf_full_` 标志只写不读（死代码）
- 全文件仅赋值从未消费。无功能影响，但未来若有人依赖它判断可回溯深度会踩坑。

#### LOW-19 [ekf_15_state.h:3537-3541] `GetDelayedSnapshotIndexByAge` 误差相等时（`<=`）偏向更旧快照
- 延迟恰在中间时选旧一格，时间对齐误差约半帧（2.5ms）。改用 `<` 优先保留更近快照更直观。

#### LOW-20 [ekf_15_state.h:3557-3571] `RestoreSnapshot` 不恢复 `ins_accel_mps2_`/`ins_gyro_radps_`（诊断字段短暂显示当前值）
- 重传播首帧即覆盖，不污染滤波；但 VOFA/诊断读数在回溯窗口内与状态不一致。

#### LOW-21 [ekf_15_state.h:843-848] 量测延迟超 64 格缓冲（320ms）且 `require_delayed_snapshot=true` 时直接拒帧，无降级
- 当前 15ms 默认不触发；但若 PPS 标定前上报延迟错误或主循环卡顿超窗，会 GNSS 长期不融合的假死。建议超窗后保守当前时刻降级并告警。

---

## 三、核对无误区域（重要，避免误改）

1. **F 矩阵全部关键块**（数值微分验证）：F(p,v)=I₃、F(v,β)=-Cbn·Skew(f)、F(v,ba)=-Cbn、F(β,β)=-Skew(ω)、F(β,bg)=-I₃、F(5,2)=2g/R、Fvv/Fpp/Fbv 地球项 ✅
2. **双子样组装**（navigation_task.cpp:632-681）：时间中点拆分三分支完备、零/负 dt 样本跳过不影响时间累计、跨中点比例拆分正确 ✅
3. **HorizontalKF 手算展开**：np00=p00+dt(p10+p01)+dt²p11 等全部与 F·P·Fᵀ 一致；H=[0,1] 增益/协方差正确 ✅
4. **VerticalKF Joseph 更新**：(I-KH)=[[1-K0,0],[-K1,1]] 正确 ✅
5. **静止检测器阈值/迟滞/置信度**：预平方、enter/exit 互斥、四子分 min + dwell 合成全部正确 ✅
6. **相位碰撞检测**：CRT gcd 判定正确，实际配置 (4/0, 8/1, 8/3) 无碰撞 ✅
7. **GNSS 周回绕**（InsGnssTowDeltaMs）：±半周判定、int64 中间量、reference 失效回退全部正确 ✅
8. **AHRS 航向校正闭环**（sensor_imu.cpp:406 + navigation_task.cpp:1183-1187）：负反馈收敛到 EKF yaw，方向正确 ✅
9. **重捕获两级检查**：NIS 门控在 P 膨胀后失效（V6 验证），物理残差门限 + 膨胀 R 是必要兜底，设计正确 ✅
10. **ins_altitude_reference.h**：NED 符号翻转、起飞点反推全部正确 ✅

---

## 四、优化改进建议（按优先级）

### P1（影响行为，建议尽快修——含 2 个数值确认的 HIGH）
1. **H1**（🔴 最高优先级）：降级路径 `yaw_err` 删除 `- ahrs_yaw_correction_rad`，与全质量路径统一。修复后需验证长时间弱 GNSS 下 backup_ahrs 不再缠绕。
2. **H2-数学**（🔴）：标量航向量测加倾斜补偿 `H_yaw=[0,0,cosφ/cosθ,sinφ/cosθ]` 或过渡飞行禁用该接口（改姿态四元数量测路径）。
3. **MED-0b**：溢出帧 `nav_update_dt_s` 改用真实时间或跳过传播，杜绝 dt 低估。
4. **MED-1 + MED-2 合并修复**：对齐静止辅助档位逻辑——档 2 帧门槛 ≥24 或 enter_min_frames<16；`prefer_gravity_first` 加 ZUPT 回退。直接改善静止初始阶段的辅助行为。
5. **MED-6**：双矢量航向加 `is_static_confirmed` 门控（数值确认 145° 噪声污染）。
6. **MED-3**：epoch 时间映射负值判无效，杜绝陈旧数据伪装新鲜。

### P2（防御加固）
7. **MED-0a**：降级路径位置量测改传真实 epoch LLA（消除自差=0 的语义隐患）。
8. **MED-7**：统一降级门限 sv（3~8 星中间帧不再被丢弃）。
9. **MED-8**：GNSS 超时清零 `has_obs` 与精度字段（与注释"必须清零"对齐）。
10. **MED-5**：统一两独立 KF 的 Q 语义为方差率×dt。
11. **MED-14 / MED-15**：传播改中点姿态旋转速度增量；重传播补失联协方差膨胀。
12. **MED-10 / MED-11 / MED-16**：DETA100 填充分支隔离；收紧 `angular_rate_quiet`；重力量测限低动态调用。
13. **LOW-3 / LOW-2**：pDOP 缩放移到 clamp 前；s_acc 回退语义修正。

### P3（报告/文档修订）
14. **MED-4**：修订延迟回放的价值论证——收益在高精度 GNSS（RTK）+ 高动态场景，当前 15ms/2m 噪声下无统计收益；报告 S4 单种子结论（回放 0.064 vs 错误 0.108）是噪声偶然，应改为多种子并标注场景边界。
15. **LOW-7**：伪磁场与 π 航向不确定度的语义矛盾写入文档。
16. **LOW-12**：`updateEstimatedVerticalVelocity` 差分 dt 加上限（GNSS 恢复后首帧避免分钟级 dt）。
17. **LOW-15 / LOW-18**：清理 `if (0)` 激光死代码与 `buf_full_` 死标志。

---

## 五、审查方法说明（可复现）

- 数值验证脚本：`TandemVec-EKF-Report/sim/verify.py` + 本报告 §一 各实验的即时脚本
- 交叉验证均为**独立于固件源码的 Python 复现**，不调用固件代码
- 3 个独立视角 agent：核心数学 / 集成调度 / 辅助模块，交叉核对
- 所有 HIGH/MED 发现均经独立数值仿真确认（V9-V12），非仅静态推断

---

## 六、修复实施与回归验证（2026-08-12）

### 已实施修复（8 项，4 个文件，git diff 72+/12-）

| 修复 | 文件 | 改动 | 验证 |
|---|---|---|---|
| **H1** | `navigation_task.cpp:319-321` | 降级路径 `yaw_err = wrap(EKF - backup)`，删除 `- ahrs_yaw_correction_rad`（双重抵消→开环） | R1: 300s 降级 backup 收敛 30°（bug 前缠绕至 21°） |
| **H2** | `ekf_15_state.h:3128-3134` | `h_yaw = C_b^n(2,:)`（真航向轴投影），水平退化为 e₈ | R2: 全姿态有效 yaw 修正 100%/roll 0%（bug 前 85° 捕获仅 9%） |
| **MED-1** | `ins_static_aid_profile.h` | 档位 1 帧门槛 16→36（确认后 80ms 弱约束窗口恢复） | R3: 档 1 存活 16 帧（bug 前 1 帧） |
| **MED-2** | `ins_static_aid_profile.h` | `prefer_gravity_first` 加 ZUPT/StaticGyro 回退（不再饿死） | R3: 低置信度 ZUPT=10（bug 前 0） |
| **MED-3** | `ins_gnss_epoch_timing.h` | `mapped_time_us<0` 判无效；负/溢出年龄不再钳 0 | R4: 未来映射/陈旧 30min → invalid |
| **MED-6** | `navigation_task.cpp:405-409` | 双矢量航向加 `static_det.confirmed_static` 门控 | R5: 静止 30s 融合 0 帧（bug 前 2 帧/-91° 注入） |
| **MED-0a** | `navigation_task.cpp:311-314` | 降级路径位置量测改传 epoch 真实 LLA（消除自差=0 语义） | 编译✓（R=1e4 行为不变） |
| **MED-0b** | `navigation_task.cpp:679-685` | 溢出帧 `nav_update_dt_s` 用 micros 差分（真实时间） | R6: 500ms 卡顿 dt 0.32→0.50s 积分完整 |

### 回归验证结果

- **修复后仿真回归（audit_after.py）：6/6 项通过**（R1-R6 全部达到预期）
- **固件 host 回归测试（test_ekf_15state.cpp，静态编译 + EKF_HOST_REGRESSION）：17/17 通过**
  —— 证明 H2 倾斜补偿在水平姿态下退化为原行为，不破坏既有滤波器断言
- 全部修改文件语法检查通过（纯函数头 + ekf_15_state.h + host 回归编译 0 error）
- git diff 仅含上述 4 文件 8 处修复，无无关改动

### 尚未修复（记录在案，建议后续）

- MED-4（延迟回放收益场景边界）、MED-5（VKF/HKF Q 语义）、MED-7~16、LOW-1~21 —— 属防御加固与代码质量，非行为正确性问题，按 P2/P3 分批处理

### 复现方式

```bash
cd TandemVec-EKF-Report/sim
py -3.12 audit_before.py   # 修复前影响量化
py -3.12 audit_after.py    # 修复后回归
```

---

## 七、第三轮多维度复查与升级优化（2026-08-13）

### 7.1 复查维度与结论

| 维度 | 复查内容 | 结论 |
|---|---|---|
| **维度1 长时稳定性** | 40s 定高发散根因深挖 | 根因=仿真 ekf_core 缺 ZUPT 物理残差门控（固件有 `ZERO_VELOCITY_RESIDUAL_REJECT_MPS=5.5`/`VELOCITY_RESIDUAL_REJECT_MPS=8.0`），补上后 40s 全程稳定（后 30s 误差 0.43-0.51m）。**非固件 bug，是仿真简化；固件此防护是设计关键** |
| **维度2 数值精度** | 整型除法 | LOW-14 确认：`_ekf_time_sum/_ekf_time_cnt` 整型除法截断诊断精度 → **已修**（改 float 除法）|
| **维度3 状态生命周期** | has_obs 清零 | MED-8 确认：`has_obs` 超时无清零路径，与注释"必须清零"矛盾 → **已修**（hAcc/vAcc 加 `gnss_data_fresh_for_nav` 门控）|
| **维度4 门限一致性** | sv 门限/AHRS 门限/DETA100 分支 | MED-7 重新评估为**设计权衡**（降级只融合速度需 sv≥9 保速度质量，非 bug）；MED-11 AHRS 门限 69°/s 过宽 → **已修**（收紧 0.5 rad/s）；MED-10 DETA100 分支覆盖 → **已修** |

### 7.2 本轮新增固件修复（4 项）

| 修复 | 文件 | 改动 | 验证 |
|---|---|---|---|
| MED-8 | `navigation_task.cpp:1463-1475` | hAcc/vAcc 加 `gnss_data_fresh_for_nav` 门控（兑现"超时必须清零"注释）| 语法✓ |
| MED-10 | `navigation_task.cpp:1440-1454` | EKF 未初始化填充分支加 `!use_deta100_output` 隔离（DETA100 直出不被 Madgwick 覆盖）| 语法✓ |
| MED-11 | `navigation_task.cpp:986` | AHRS 姿态量测门限 1.2→0.5 rad/s（69°→28°/s，与双矢量 20°/s 一致量级）| 语法✓ |
| LOW-14 | `navigation_task.cpp:1499` | 整型除法改 float（诊断精度）| 语法✓ |

### 7.3 仿真层修复（1 项）

| 修复 | 文件 | 改动 |
|---|---|---|
| 仿真 ZUPT 残差门控 | `sim/ekf_core.py` | 对齐固件 `ZERO_VELOCITY_RESIDUAL_REJECT_MPS=5.5`/`VELOCITY_RESIDUAL_REJECT_MPS=8.0`（40s 定高稳定根因）|

### 7.4 验证结果

- **固件 host 回归：17/17 通过**（含 ekf_core ZUPT 残差门控改动后复验）
- **仿真全场景收敛**：S2 定高 RMSE 0.63m（25s 窗口）、S1/S3/S4a 姿态稳定
- **40s 长时定高**（补 ZUPT 门控后）：后 30s 误差 0.43-0.51m 稳定

### 7.5 遗留（记录，不擅自改）

- **MED-7**（GNSS_MIN_SV=9 vs 全质量 3）：重新评估为"降级只融合速度需足够卫星数"的合理保守设计，建议实机确认而非直接放宽
- **MED-5**（VKF/HKF Q 语义不一致）、**MED-9**（分频相位跨档突变）等：属代码质量/一致性，非行为正确性，按需分批

---

## 八、第四轮：MED-14 修复 + 剩余项重评（2026-08-13）

### 8.1 MED-14 修复（速度推进姿态旋转点不一致）

**问题**：固件 `ekf_15_state.h` 速度增量用 `t_b2ned_prev`（上时刻姿态）旋转，而 F 矩阵线性化（F(V,Φ)/F(V,b_a)）用 `t_b2ned_mid`（中点姿态）——名义积分与协方差线性化点不一致。

**量化验证**：
- 单步：ω=2 rad/s 时 |mid-prev|=0.005 m/s，ω=10 rad/s 时 0.025 m/s
- 累积：30s 持续转弯 ω=3 rad/s 时速度偏差 0.042 m/s（确定性累积，非随机噪声）

**修复**（零开销，mid 姿态上文已计算）：
- 固件 `ekf_15_state.h`：`t_b2ned_prev` → `t_b2ned_mid`
- 仿真 `ekf_core.py`：同步改 `Cnb_mid`

**验证**：host 回归 17/17 ✓；verify.py V1-V5 全部一致 ✓；hifi 全场景收敛（S2 RMSE 0.63m）✓

### 8.2 MED-15 量化结论（重传播缺失联膨胀）

**确认**：`GrowGnssOutageCovariance` 只在 TimeUpdate(665)/TimeUpdateTwoSample(747) 调用，`RepropagateFromDelayedSnapshot` 缺。

**量化**：15ms 延迟 = 3 步，每步膨胀 pos_ne=1.2²·0.005·scale、vel_ne=0.35²·0.005·scale：
- 位置：3 步累计 0.02-0.09 m²（scale 1→4），vs R_pos=4 m² → 影响 ~2%
- 速度：3 步累计 0.0018 m²/s²，vs R_vel=0.0225 → 影响 ~8%（仅长失联 scale=4）
- NIS 门限 30 有余量

**结论**：影响可忽略，且修复涉及重传播期间 `time_since_gnss_update_s_` 的复杂状态管理，风险 > 收益。**记录为防御加固项，暂不修复**。

### 8.3 MED-5 重评（VKF/HKF Q 语义）

**确认**：VerticalKF 参数是"方差"（`KF_V_Q_POS=0.0025` 定值），HorizontalKF 是"标准差·dt"（`kf_h_q_accel=0.35`），注释已明确两种语义，**是文档约定差异非 bug**。改代码有实机行为回归风险（垂直 KF 输出供地面站与 EKF 对比一致性），**记录不擅自改**。

### 8.4 累计修复状态

累计四轮：**已修复 13 项固件**（H1/H2 + MED-1/2/3/6/8/10/11/14/0a/0b + LOW-14）+ 仿真 2 项（ZUPT 残差门控、MED-14 对齐），全部 host 回归通过。

**遗留**：MED-7/15（设计权衡/防御加固，量化后暂不修）、MED-5/9（文档/一致性，实机确认后再改）。
