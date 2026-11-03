# AGENTS.md — TandemVec_FCS 飞控固件

> 基于 PlatformIO 的 STM32H743 **纵列双发矢量推力 VTOL 飞控固件**（从共轴双桨 VTVL 移植重构）。安全关键嵌入式控制软件，不是普通应用代码。

## 坐标系与执行器（2026-08-02 钉死；权威 = mix 层 `flight_control.cpp` 1202–1252）

- **机体系 FRD**：`x_b` 前（前电机 +x 拉力式、尾电机 −x 推进式）、`y_b` 右、`z_b` 下；★ **电机推力轴 = z_b**（悬停机头朝天时 z_b 竖直指地）。旧"x_b=推力轴"错误（2026-08-09 修正）。
- **执行器→力矩（悬停构型语义）**：上摆 δf（前电机最高点）→滚转主控；下摆 δt（尾电机）→俯仰主控；差速 Δω→偏航主控。旧"前摆=偏航、差速反扭滚转"是巡航读法，已废止。
- **悬停构型**：机头朝天（x_b ∥ NED −z）——差速（绕 x_b）=世界航向；z_b 悬停时水平。
- **RATE_MODE**：三轴经 `RcCurve.h`（rc_rate/expo/super），满杆 roll/pitch 333°/s、yaw 222°/s（限幅 600°/s）；**ATTITUDE_MODE**：q_err.x→上摆、q_err.y→下摆、q_err.z→差速。
- **NED z ≠ 机体 z_b**：推力垂直补偿用 R13（x_b 投影）；激光斜距用 R33（沿 −z_b）。推导见 `docs/04-数学建模/MOD-002` §1.2。
- **AnoVars**（0xF2/0xF3/MAVLink/DBG vars）：加变量 = `src/ano_vars.cpp` kAnoVars 加一行宏；MAVLink 名 9 字符截断须唯一。
- **编码**：批处理内容必须 ASCII（cmd 启动代码页解析）；中文输出走 Python（PYTHONUTF8=1）。

## 飞行器构型与控制分配

- 前电机(CW)最高点=上摆 δf=滚转主控；尾电机(CCW)最低点=下摆 δt=俯仰主控；差速 Δω=偏航主控。
- 物理逆解：α(deg/s²)→`ControlUnits::dps2ToRadps2`→I×α(rad/s²)→M_cmd→`allocateMoments(BTRUE)`→δf/δt/Δω。
- 执行器映射：δf→PA0(TVC_ROLL)、δt→PA1(TVC_PITCH)、Δω→PA2/PA3。
- 齿轮 30T/40T=1.333:1（PWM 映射已含）。

## 当前约定（含已修复遗留）

- 机体系 = 原始 VTVL 实飞存档版约定；★ 判据：机头竖直朝天静止解算 roll=pitch=0。
- 推力垂直投影用 **R33**=1−2(qx²+qy²)；**勿改 R13**（"x_b 竖直"错误映射时期产物）。
- 目标姿态合成 = `Rz(Heading)⊗Rxy(rollTarget,pitchTarget)`；勿加悬停基态补偿。
- **调试开关宏陷阱**：`#define XXX_TEST 0` 时 `#ifdef` 仍为真——调试开关一律用 `#if`。
- **IMU 轴映射**（`sensor_imu.cpp` 步骤5，与存档逐字一致，勿改）：`bX=-sZ`、`bY=+sX`、`bZ=-sY`；判据：机头朝天静止 `acc≈(0,0,-1)g` 且 roll=pitch≈0。改前先问：是否与存档一致？
- `Quaternion` 分量顺序 **{w,x,y,z}**（`QuaternionMath.h:49`）。
- 符号/轴排查顺序：**先查轴映射，再查控制律符号**。
- **mix 层轴置换**（`flight_control.cpp` 层1）：`x'=-z_b, y'=+y_b, z'=+x_b`；`Mx'=-Ix·alpha_yaw`、`My'=+Iy·alpha_pitch`、`Mz'=-Iz·alpha_roll`；勿改 `computeEffectMatrix`。
- 通道↔执行器：roll→上摆、pitch→下摆、yaw→差速；TVC 轴可高带宽、差速轴受电机 τm=0.28s 限带必须保守。
- 摇杆语义：ATTITUDE_MODE=roll/pitch 目标姿态角、yaw 目标角速度；RATE_MODE=三杆目标角速度。
- `TVC_Control_Geometric.h`/`TVC_Control_3rdOrder_Poly.h` 已弃用；分配由 `TandemVec_ControlAllocation.h`。
- **时基与 IMU 抽象（2026-11-01）**：TIM8 6.4kHz；IMU 任务周期与 Madgwick 由 `actualAccelOdrHz()` 反推（2k→0.5ms、3.2k→0.3125ms）；默认 42688P，45686 用 `-DFCS_IMU_MODEL_ICM45686`；LPF 陀螺 200/加计 100Hz；EKF `q_accel_scale=2.0`；hAcc 门控宏默认 0。
- **导航宏一览（除注明外均默认关，宏=0/1 双状态编译通过）**：`BFS_NAV_TRIAD_INIT`（TRIAD 初始化，`include/tilt_compass_triad.h`）、`BFS_NAVIGATION_PARAM_ALIGN`（参数辨识精对准，`include/ins_param_align.h`）、`BFS_PRELAUNCH_ALIGN_STABILITY_GATE`（预对准窗口，`include/ins_prelaunch_align.h`）、`BFS_NAVIGATION_IMU_HEALTH`（卡死监测）、`BFS_NAVIGATION_IMU_TEMP_COMP`（温补）、`BFS_NAVIGATION_GNSS_REANCHOR`（硬重锚）、`BFS_NAVIGATION_GNSS_HACC_GATE`（hAcc 门控）、**`BFS_AHRS_GNSS_FRESH_GATE`（默认 1，GNSS 新鲜时不做 AHRS/重力姿态辅助）**、**`BFS_ATT_AID_MODE_DEFAULT`（默认 5=严格 + 倾斜 σ 超 3° 后 AHRS 有界兜底：无外部参考时先纯陀螺，σ 超限后用 AHRS 保证不发散；1=纯严格 /0=AHRS 旧行为 /2=严格+AHRS 回退 /3=控制加速度指令补偿（研究臂，先验非观测，已被严格门控拦截）/4=强振动回退，运行期 `g_att_aid_mode`、ANO `att_mode`、SIL `--att-aid-mode`）**、`BFS_GRAVITY_AID_VEL_FF`（默认 0，2026-11-02 的“EKF 速度差分扣比力”自参照前馈，MC 证伪后仅作对照）、`BFS_MOTION_GRAVITY_GATE_RAD`（默认 15°，运动段方向门）、**`BFS_TILT_DG_WARN_DEG`/`BFS_TILT_DG_HARD_DEG`（默认 3°/10°，无参考漂移预算两级阈值，ANO 变量 `tilt_dg`/`tilt_hard`）、**`BFS_NAVIGATION_EMBEDDED_FLOAT_COVARIANCE`（默认 0，飞行 env=1：协方差传播中间量改 float，Phi·P·Phiᵀ 与 Simpson Qd 公式不变；上板 cov_avg 5.20→0.97 ms、Navigation avg 5.66→1.52 ms）**、**`BFS_GUIDED_ACCEL_TRIM`（默认 0，GUIDED 比力慢积分：0.1 Hz LPF、±0.5 死区、0.3 增益、±2.0 限幅、泄漏、进 GUIDED 预热 1 s）**、**`BFS_GUIDED_ACCEL_TRIM_VEL`（默认 0，速度差分输入，噪声下已证伪，仅对照）**、**`BFS_GUIDED_THRUST_ADAPT`（默认 0，推力增益在线辨识，乘性前馈补偿）**、**`BFS_THRUST_CURVE_QUADRATIC`（默认 0，仅 ESC 呈 F∝ω² 平方特性时启用；现役线性 ESC 不启用）**。上板验证与验收标准见 `docs/04-数学建模/代码理论核查/上板标定与验证清单-2026-11-01.md`；AHRS/重力融合的 A/B 见 `代码理论核查/AHRS姿态辅助与无GNSS重力融合-2026-11-01.md`。

## 核心参数

- **控制增益/限幅/滤波唯一入口**：`include/FlightCtrlParams.h` `kFlightCtrlParams`（static constexpr，固件与宿主测试共用）；速率环 kp 单位 s⁻¹、ki s⁻²；旧参数 0.28/0.20/0.10 不得导入。
- 单一定义源：`include/TandemVec_Config.h`（kT/kQ/wMax/I/a/b/dMax/m/g/ServoConfig）。
- 差速增益调度/工作点下限/零油门门控：`flight_control.cpp` 层2（改后跑 `tools/verify_*.py`）。
- 控制链遥测：`state_data.h` §4.0b `gnc_tel`。
- CascadeCtrl 半成品架构已废弃删除（2026-08-08）。
- 舵机行程/方向/中位：`TandemVec_Config.h` §ServoConfig。

## 执行原则

- 先读上下文再动手（`git status --short` + 相关源码 + `platformio.ini` + 本文件）；先诊断后行动。
- 小步、聚焦、可验证；不覆盖用户未提交改动（`main.cpp`/`.vscode`/`lib`/当前打开文件）。
- 新增/修改的注释、文档、提交说明用中文；命令/路径/标识符原样。全部思考过程用中文。
- 同一问题连续改 3 次未解决 → 停下，重新按工程式分析根因/约束/影响面/验证方案。

## 硬件与串口

- 电路权威参考：`docs/电路拓扑参考.md`（52 引脚/型号/连接器）；改引脚前必读。
- 串口/引脚/传感器驱动：改前读 `.agents/docs/hardware-reference.md`。
- **Serial6(USART6) 波特率** = `state_data.h` `SERIAL6_BAUDRATE`（当前 921600；2M 在 VCP 丢帧 ~15%）。
- **DBG 调试**：Serial6 收 `"DBG\n"` 进调试（`handleDebugConsole`），`exit` 退出；命令表与工具链见 `docs/registers/固件调试工具链-2026-11-01.md`。
- **磁标定（DBG `magcal`）**：`start`/`fit`/`save`（写内部 Flash bank2 末扇区 0x081E0000）；★ 禁止 EEPROM 仿真（同用末扇区）；引导校准 `magcal guided`。

- **DAPLink 烧录（2026-11-03）**：adapter speed 需降到 **20–100 kHz** 才稳定 `Verified OK`；高速下偶发 checksum mismatch（diff 为孤立位翻转）不是固件坏。OpenOCD `program` 后必须 `reset run`，否则板子停住。命令见 `docs/registers/踩坑记录-AGENTS.md` 2026-11-03 条。
- **GNSS（NEO-F10N，protocol 40.00）**：F10 接口说明只收录 **CFG-VALSET/VALGET/VALDEL + CFG-MSGOUT**（PVT UART1 `0x20910007` p.143、EOE UART1 `0x20910160` p.142），**没有 CFG-MSG/CFG-PRT 章节**；固件保留 CFG-MSG 是实测兼容层（F10 仍 ACK）。CFG-MSG 8 字节 payload 的 rate 必须同时写 UART1/UART2。**配置 ACK 不等于有输出**，验收看 `ubxstat` 的 `pvt/eoe` 是否增长。详见 `docs/04-数学建模/代码理论核查/F10N-GNSS配置与调试-2026-11-03.md`。
- **新增 `health` 命令**：一屏看模式/IMU 健康/气压/GNSS pvt-eoe-fix-sv/ EKF rpy-vel-pos/ Serial5 上行 rx-bad-age-txskip，以及 loop CPU/max_pass/window（不重置 tasks 统计）。

## 硬性代码边界

- `DETA100_module.h` 只能在 `main.cpp` 包含（内含解析实现与静态状态）。
- `NavDataSource::INTERNAL/DETA100` 由 `setup()` 锁定；`handleDeta100` 仅在 DETA100 时注册且必须在状态估计之前。
- `QuaternionMath.h`/`GeoDisplacement.h` 工具函数保持 `inline`；`Vector3` 用 `VECTOR3_TYPE_GUARD`。
- MAVLink 与 AnoCom 共用 Serial6，按互斥处理。
- `lib/MAVLink` 是 vendor tree，不编辑。

## 安全关键修改规则

未经用户明确要求，不改：硬件引脚/串口/波特率/PWM/舵机中位限幅；任务频率/定时器/调度/控制周期；PID 增益/混控/TVC 限幅；传感器坐标方向/FRD-NED 约定；解锁/故障保护/超时/降级/输出限幅。

改了控制/估计/混控/解锁/协议 → 最终回复必须说明：影响行为、改变的模式/故障路径、已完成验证与上板风险。不得为编译通过删除安全检查/降级/超时/限幅。

## 构建与验证

```powershell
pio run                    # 编译固件
pio test -e test           # 宿主回归
bash test_host/run_all.sh  # 宿主全量（40+ 段，以脚本实际为准）
bash test_host/fw_sil/run_acceptance.sh          # SIL 验收：真实固件+EKF，3 seeds
bash test_host/fw_sil/build_and_run_failsafe.sh  # SIL 失效保护/模式重入
```

- 宿主构建用 `tools/with-ascii-bin.sh`（subst ASCII 盘符、`BIN_DIR`=仓库 `tmp/`，不写系统 %TEMP%）。
- EKF 测试重力 `g=9.79`（`TandemVec_Config.h`），勿改 9.81。
- 上传只在用户明确要求时：`pio run -t upload`；串口监视加超时。
- 找不到 `pio` 必须明确报告，不得声称构建通过。
- 验证选择：文档改动 `git diff --check`；代码整理 `pio run`；控制/导航/通信/安全改动 `pio run` + `pio test -e test`；无法运行必须说明原因。
- **算法测试两阶段**：先理想模型（零噪声/零偏、WGS84 重力 `ins_test::GravityWgs84()`、解析真值）验证正确性，再真实噪声/零偏/振动/失锁评估工程效果；理想阶段失败修算法，真实阶段不达标先归因。
- **不声称未执行的构建/测试/上板验证通过。**
- **宿主 UBSan**：MinGW 没带 libubsan，用 `-fsanitize=undefined -fsanitize-undefined-trap-on-error`（命中 UB 直接 trap，不需要运行时库）。2026-11-03 ekf15 在 UBSan 下 116/0、无陷阱；命令模板见 `docs/registers/固件调试工具链-2026-11-01.md`。
- **性能 A/B 环境（默认飞行 env 不变）**：`[env:TandemVec_FCS_PHASE]` 开 `BFS_TASK_PHASE_STAGGER`（只错开 ≥5ms 任务首次相位，治 200Hz 实跑约 155Hz 的 flag 合并）；`[env:TandemVec_FCS_DTCM]` 开 `BFS_NAV_EKF_DTCM`（`nav_ekf` 落 DTCM 0x20000000）。A/B 只看 `tasks` 的 avg/max/迟到 + `health`，不以单次 CPU 平均下结论；详见 `docs/04-数学建模/代码理论核查/飞控实时性能基线-2026-11-03.md`。

## 仓库卫生

- 改前后 `git status --short`；提交前 `git diff --check`。
- `.pio/`、`compile_commands.json`、本地 VS Code 索引/调试产物不提交。
- 上下文文件不改成参数流水账。
- COM 口独占：`pio device monitor` 加 timeout；PowerShell 串口脚本 `try/finally` + `ReadTimeout=500ms`。
- 运行时抓包文件用完清理；串口拒绝访问先 `taskkill /f /im pio.exe` 或杀 PowerShell。

## 实机安全注意事项

建议任何实机测试前，必须提醒断开电机动力电源（除非请求明确不涉及实时输出）。飞控逻辑改动分阶段：编译检查 → 传感器初始化/遥测 → 断动力查执行机构方向与限位 → 系留低功率 → 自由飞。
