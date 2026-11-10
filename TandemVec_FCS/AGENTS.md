# AGENTS.md — TandemVec_FCS 飞控固件

> 基于 PlatformIO 的 STM32H743 **UniGNC 固件**：一套 GNC 内核（姿态环 + 15 状态组合导航 + 200 Hz 调度）+ 可插拔机型分配层。同一块板可烧纵列双发、共轴双桨 VTVL、单涵道四燃气舵。安全关键嵌入式控制软件，不是普通应用代码。

## 路径约定（本文所有路径都按此解析）

- `docs/...` = **固件侧** `TandemVec_FCS/docs/`（电路拓扑、PROTOCOL-001、mavlink-ref、08-08/08-10 的 memory 等）。
- `../docs/...` = **项目级** 仓库根 `docs/`（GOV/MOD/CTL/VER/DEC 系列、11 月的 memory、registers）。
- `.agents/docs/hardware-reference.md` = 固件侧硬件参考。
- 两边都有 `memory/` 子目录，**写路径前先确认这份文档在哪边**（查项目级总索引
  [`../docs/README.md`](../docs/README.md)）。项目级文档导航一律以 `../docs/README.md` 为准；
  固件侧 `docs/` 只有 8 篇独立文档 + 6 篇 memory，见其 [`docs/`](../docs/) 目录。

## 当前实机机型与构建

- **同一块 WeAct MiniSTM32H743 板可烧三种机型，env 选错等于烧错飞机**：

  | 机型 | 构建 env | `health` 输出 |
  |---|---|---|
  | 纵列双发矢量推力 | `[env:TandemVec_FCS]` | `veh=TandemVec` |
  | 共轴双桨 VTVL | `[env:CoaxialVTVL]`（`-DBFS_COAXIAL_VTVL`） | `veh=CoaxialVTVL` |
  | 单涵道 + X 型四燃气舵 | `[env:SingleEdf4Vane]`（`-DBFS_SINGLE_EDF_4VANE`） | `veh=SingleEdf4Vane` |

- **烧录前先确认飞哪一型**；上板第一件事 `health` 核对 `veh=`。

- **IMU 型号也是构建期二选一（2026-11-09 起）**：`FCS_IMU_MODEL_ICM45686` 选后端，不带则默认 ICM-42688-P。两个后端**互相独立、可随意切换**——改的都是 `lib/ICM45686/` 与 `lib/ICM42688/` 各自的驱动，上层 `imu_factory.cpp` 按宏分派，共用同一套 SPI 引脚与 `ImuDevice` 抽象。

  **env 是「四维正交」的（2026-11-10 重整）：别背组合名，只记四个维度 + 命名规约 `‹机型›[_ICM45686][_MAVLink][_Flight]`。速查表在 `platformio.ini` 文件头。**

  | 维度 | 取值 | 对应宏 |
  |---|---|---|
  | 机型（必选） | 纵列 / 共轴 / 单涵道 | （无宏=纵列）/ `-DBFS_COAXIAL_VTVL` / `-DBFS_SINGLE_EDF_4VANE` |
  | IMU | 42688（缺省）/ 45686 | — / `-DFCS_IMU_MODEL_ICM45686` |
  | 默认遥测协议 | AnoCom（缺省，配本项目 GCS）/ MAVLink | — / `-DBFS_DEFAULT_PROTO_MAVLINK` |
  | 用途 | 调试（缺省，带 DBG 控制台）/ 实飞 | — / `-UBFS_TASK_PROFILE -UBFS_NAV_PROFILE -UBFS_PROFILE_TO_ANOCOM_SERIAL` |

  ★ **上层全共用**：板级/CPU-FPU/编译优化/调度时基/IMU 采样架构/导航与 EKF 宏/看门狗全部收在 `[env:unignc]`；三个机型 env 各自只差一个机型宏。**新增机型或组合 = 两行**，不许复制基座里的宏（照 `platformio.ini` 末尾「新增组合」抄）。
  ★ **采样率**：42688 = 2000 Hz（硬件 FIFO）；45686 = 3200 Hz（FIFO 单包 + ISR 每 2 拍取一包）。
  ★ `default_envs` 指向当前实机（现为 `CoaxialVTVL_ICM45686`）；**换板/换机型必须同步改这一行**，否则 `pio run -t upload` 会刷错机型。
  ★ 编译产物按 env 各自独立（`.pio/build/<env>/`），换 env 不需要 clean。

  ★ **45686 = 3.2 kHz FIFO 单包，ISR 每 2 拍取一包**（2026-11-10 定稿），不是 200 Hz 轮询。ODR 取 3200，一个 FIFO 包即一次满量程样本，2 kHz ISR 每隔一拍取一包——这就是「3.2k 采样率」的落地方式。
  ★ **绝不给 45686 构建加 `-UBFS_IMU_ISR_RING`**：该宏关掉 ISR ring、退回 200 Hz 轮询寄存器，是用户 2026-11-10 明确拒绝的方案。判断有没有被误加：看 `pio run -e <env> -v` 的 `-U` 列表，或启动日志 `[Init] IMU: ICM-45686 ... 3200Hz` 后 `isr_f≈3200`。详见 `../docs/memory/2026-11-09-ICM45686换型适配.md`。
- **单涵道实飞用 `[env:SingleEdf4Vane_Flight]`**：不带 `BFS_DBG_USB_SERIAL8`、`-U BFS_TASK_PROFILE`/`BFS_NAV_PROFILE`/`BFS_PROFILE_TO_ANOCOM_SERIAL`；保留 2kHz ISR ring、AnoCom USB 参数链路、0xF4/0xF5 遥测、全部联锁限幅。烧前六项自查见 CTL-008。
- **0xF4/0xF5 诊断帧只在调试 env 编译**：这两个帧包在 `#ifdef BFS_ANOCOM_DIAG` 里（`communication.cpp:945–1028`）。调试 env（`[env:SingleEdf4Vane]` 等）带 `-DBFS_ANOCOM_DIAG`，实飞 env（`[env:SingleEdf4Vane_Flight]`）不带。所以烧实飞固件时 GCS 看不到 0xF4/0xF5 是**正常降载**，不是链路故障——实飞靠 0xF1/0x0A/0x21 等标准帧。
- **共轴 SIL 必须 `--coax-plant-model rigorous`**：L1 集总 plant 的俯仰/姿态约定有误，会误报 max_pitch≈23°，不要用 L1 结果判共轴。
- **共轴控制链 2026-11-10 已统一到通用层**：与纵列/单涵道同一套 `execute_attitude_controller` + `execute_yaw_controller`（输出 α[rad/s²]）→ `CoaxialAllocator`（α→τ→摆角 `asin(τ/(l·T))` / 差速 `τ/k_yaw`）→ `mapCoaxialVtvOutputs`。生产路径**不再**引用 `CoaxialVtvController` / `CoaxialVtvParams`（文件与宿主测试保留作历史对照）。共轴参数走通用 `att_*`/`rate_*` 表（deg/s² 域：kp 353.4、ki 113.1 是旧 rad 域 6.168/1.974 ×57.3）；`rate_*.out_max` 由 `coax` 物理量在 `applyFlightCtrlParams()` 里自动换算（10° 舵角 ≈246.7 deg/s²、180 µs ≈0.1289 deg/s²），**不要手改 out_max**——要收权限就改 `coax.max_corr_deg` / `coax.max_yaw_us`，要调响应就改 `rate_*.kp/ki`。
- **手动模式不做姿态切电机**：手动 RATE/ATTITUDE 可能翻跟头或被风扰超 45°，不因姿态角切电机；AUTO/GUIDED 保留 `coax.att_limit_deg`（默认 90°，可调 10–180°）。禁止把 45° 硬切电机加回手动路径。
- **`BFS_NAV_EKF_VERBOSE` 默认关**：Navigation 60s 一次的多段 Serial8 打印是 300s 数传/P1 长停顿的根因，需要时才显式打开。
- **P1 是采样架构阶段名，不是 env 名**：共轴没有叫 P1/P1_UI 的环境，机型 env 是 `[env:CoaxialVTVL]` 及其 `_ICM45686` / `_MAVLink` 变体，全部跑 P1 架构（2kHz ISR ring）。纯轮询回退环境 2026-11-10 已删除——它与主 env 的语义曾恰好互换（主 env 带 -U 实际跑轮询、_UI 实际跑 ring），保留只会烧错。临时 profiling 用编译宏。
- **高频采样/时序测量禁用 DBG**：只用 0xF4（`tvc record`）或逻辑分析仪；MAVLink 模式可用 `DBG\n` 切回 AnoCom，不需物理复位。
- **单涵道机型要点**（完整移植说明见 `../docs/05-控制与分配/CTL-008-单涵道四燃气舵机型移植-2026-11-05.md`）：
  四舵 PA0–PA3 + 推进器 PC8 + 推进使能 PE6；速率环 α 域，kp=57.3 s⁻¹ 是源机型舵角域 0.25 的等价换算；限幅由 `sedf.ctrl_vane_deg` 换算（20° → 4583.66 deg/s²），**不要直接改 `rate_*.out_max`**（下次 apply 会覆盖）；矩阵与 yaw 轴符号属机型知识，收在 `SingleEdf4VaneAllocator.h`。

> 三机型历次实飞/实测完整数据（IMU 2kHz 基线、单涵道 120s 采样、验收读数）见 `../docs/memory/2026-11-04-IMU-2kHz攻关.md`、`../docs/memory/2026-11-05-单涵道四燃气舵首飞.md`、`../docs/04-数学建模/代码理论核查/IMU采样调试经验-2026-11-04.md`。


## 坐标系与执行器（2026-11-04 审查定稿；权威 = `src/flight_control.cpp` `mix_and_output_commands()` + `GOV-004` §7.2）

> ★ **全仓有两套轴名**，是同一机械布局的纯旋转置换 `x' = −z_b, y' = +y_b, z' = +x_b`（det=+1，`tools/verify_frame_map.py` 已校验）。写任何轴/通道注释必须标注「存档系」或「模型系」，**不得只写 x/y/z**。下表是唯一对照口径。

| 项 | 固件存档系（本文件 / 固件源码 / `ControlContracts.h` / `CoaxialAllocator.h` / `FlightCtrlParams.h`） | 仿真模型系（`MOD-002` / `MOD-001` / `CTL-001 §3` / 论文 / `TandemVec_Propulsion.h`） |
|---|---|---|
| 推力轴 = 机身纵轴 | **z_b**（悬停竖直指地；推力沿 −z_b 朝上） | **x′**（朝机头；悬停竖直朝天） |
| 其余两轴 | x_b / y_b 为横向轴（悬停水平）；滚转绕 x_b、俯仰绕 y_b | y′ = 右 / 尾摆轴，z′ = 上摆轴 |
| 悬停姿态 | **roll=pitch=0，q=Rz(Heading)**，R33=1（判据：机头朝天静止 acc≈(0,0,−1)g） | q_hover = 绕 +y′ 转 90°，x′ ∥ NED −z，R13=±1 |
| 通道→执行器 | 上摆 δf→滚转（Mx，绕 x_b）、下摆 δt→俯仰（My）、差速 Δω→偏航（Mz，**绕 z_b = 推力轴 = 世界航向**） | 前摆 δf→Mz 偏航、尾摆 δt→My 俯仰、差速 Δω→Mx 滚转 |
| 倾角补偿 | **\|R33\| = \|1−2(qx²+qy²)\|**：推力倾斜补偿与光流/激光斜距反投影都用 | **R13 = 2(xz+wy)**（仅仿真模型） |
| 四元数分量 | `{w,x,y,z}`（`QuaternionMath.h:49`） | `{x,y,z,w}`（THREE） |

- **机体系（存档系）**：`z_b` = 推力轴 = 机身纵轴；`x_b`/`y_b` 为两条横向轴（轴序校验见 `FW-CTRL` P2-1）。
- **执行器→力矩（悬停构型语义）**：上摆 δf（前电机最高点）→滚转主控；下摆 δt（尾电机最低点）→俯仰主控；差速 Δω→偏航主控。
- **悬停构型**：机身竖立、机头朝天；推力轴 z_b 竖直指地（与 NED +z 平行），差速绕 z_b 即世界航向；x_b/y_b 水平。
- **RATE_MODE**：三轴经 `RcCurve.h`（rc_rate/expo/super），满杆 roll/pitch 333°/s、yaw 222°/s（限幅 600°/s）；**ATTITUDE_MODE**：q_err.x→上摆、q_err.y→下摆、q_err.z→差速（航向由 `execute_yaw_controller` 统一处理）。
- **倾角补偿只用 R33**：固件 `compute_thrust_control()`、`sensor_peripheral.cpp` 两处斜距反投影全部走 `|R33|`（钳 0.5 / 0.1）。**`MOD-002 §1.2` 的 R13 只适用于仿真模型系，不得作为固件依据**。
- **AnoVars**（0xF2/0xF3/MAVLink/DBG vars）：加变量 = `src/ano_vars.cpp` kAnoVars 加一行宏；MAVLink 名 9 字符截断须唯一。
- **编码**：批处理内容必须 ASCII（cmd 启动代码页解析）；中文输出走 Python（PYTHONUTF8=1）。

## 飞行器构型与控制分配

- 前电机(CW)最高点=上摆 δf=滚转主控；尾电机(CCW)最低点=下摆 δt=俯仰主控；差速 Δω=偏航主控。
- 物理逆解：α(deg/s²)→`ControlUnits::dps2ToRadps2`→I×α(rad/s²)→M_cmd→`allocateMoments(BTRUE)`→δf/δt/Δω。
- 执行器映射：δf→PA0(TVC_ROLL)、δt→PA1(TVC_PITCH)、Δω→PA2/PA3。
- 齿轮 30T/40T=1.333:1（PWM 映射已含）。
- **单涵道机型（`BFS_SINGLE_EDF_4VANE`）**：执行器换成 4 片燃气舵 + 1 路推进器油门（无差速通道）。
  ★ **飞控路径与其他机型统一**：速率环输出三轴角加速度 α [deg/s²]，
  分配器 `include/SingleEdf4VaneAllocator.h` 做 α → τ=I·α → `δ = Hᵀ·τ/(4k)` → 联合限幅
  （行正交编译期断言），限幅由上节的机械摆角换算而来。
  直通路径（`alloc_in.direct_vane_enabled`）只用于**未解锁/手动 TVC 的地面标定**，不是飞行闭环路径；
  矩阵已按存档系重构——纵列"偏航"（绕 z_b=推力轴）对应源机型的"滚转"，物理上是世界航向。
  四舵 PA0–PA3 只做语义重映射，PC8 为推进器油门，PE6 为推进使能。

## 当前约定

- 机体系 = 固件存档系（`z_b`=推力轴=机身纵轴，悬停竖直指地）；★ 判据：机身竖立、机头朝天静止时解算 roll=pitch=0，即 `q=Rz(Heading)`、R33=1。
- 推力垂直投影（含光流/激光斜距反投影）用 **R33**=1−2(qx²+qy²)；仿真模型系的 R13 不适用固件，不得作为固件依据。
- 目标姿态合成 = `Rz(Heading)⊗Rxy(rollTarget,pitchTarget)`；勿加悬停基态补偿。
- **调试开关宏陷阱**：`#define XXX_TEST 0` 时 `#ifdef` 仍为真——调试开关一律用 `#if`。
- **IMU 轴映射**（`sensor_imu.cpp` 步骤5，与存档逐字一致，勿改）：`bX=-sZ`、`bY=+sX`、`bZ=-sY`；判据：机头朝天静止 `acc≈(0,0,-1)g` 且 roll=pitch≈0。改前先问：是否与存档一致？
- `Quaternion` 分量顺序 **{w,x,y,z}**（`QuaternionMath.h:49`）。
- 符号/轴排查顺序：**先查轴映射，再查控制律符号**。
- **mix 层轴置换**（`flight_control.cpp` 层1）：`x'=-z_b, y'=+y_b, z'=+x_b`；`Mx'=-Ix·alpha_yaw`、`My'=+Iy·alpha_pitch`、`Mz'=-Iz·alpha_roll`；勿改 `computeEffectMatrix`。
- 通道↔执行器：roll→上摆、pitch→下摆、yaw→差速；TVC 轴可高带宽、差速轴受电机 τm=0.28s 限带必须保守。
- 摇杆语义：ATTITUDE_MODE=roll/pitch 目标姿态角、yaw 目标角速度；RATE_MODE=三杆目标角速度。
- 力矩分配由 `TandemVec_ControlAllocation.h` 负责。
- **时基与 IMU 抽象（2026-11-04 P1）**：TIM8 6.4kHz；P1 用 2kHz ISR 单包入 SPSC ring，Navigation 200Hz 排空；备份 Madgwick/欧拉约200Hz，IMU 增量/EKF 每样本；默认 42688P，45686 用 `-DFCS_IMU_MODEL_ICM45686`；LPF 陀螺 200/加计 100Hz；EKF `q_accel_scale=2.0`；hAcc 门控宏默认 0。
- **导航宏（18 个，除注明外默认关）**：完整表格（宏名 / 默认值 / 作用 / 开启前提）见 `../docs/04-数学建模/代码理论核查/上板标定与验证清单-2026-11-01.md` §7——改导航宏或判 EKF 行为前先查那张表，别只凭记忆里的默认值；上板标定流程见同文档 §1–§6、§8。

## 核心参数

- **无GNSS/首次定位（2026-11-04）**：无GNSS允许解锁并退化到姿态模式（无定点）；
  带fake origin解锁期间**不做首次GNSS硬重锚**，避免位置/速度突变；等上锁后
  再锚定绝对位置。SIL 构建旁路首次解锁门。
- **EKF位置语义（2026-11-04）**：内部状态是绝对LLA；`rel_ned` 是
  `lla2ned(EKF_LLA, origin)` 的输出。首次GNSS且未解锁时重锚并令origin=当前点、
  相对位置归0；已解锁后保留真实起飞origin。不要把相对位置当EKF输入。
- **静止相对位置保持（2026-11-04）**：EKF 新增零位移伪量测
  `MeasurementUpdatePositionDetailed`；仅 `is_static_confirmed && throttle<3%
  && fake_origin` 时用静止锚点做相对位置更新，运动/恢复绝对参考立即退出。
  室内80s实测 pos_sig0.44、conv1；这是相对定位，不替代GNSS。
- **参数无Flash持久化（2026-11-04审计）**：`param save 0xAB` 只回校验帧，不写Flash；
  重启回 `kFlightCtrlParamsDefaults`。调参后用 GCS `param dump` 保存快照，
  下次上电用 `param load` 重放并回读确认。需要掉电保存时必须单独设计独立扇区+CRC+
  掉电测试，不能直接复用磁标定末扇区。
- **控制增益/限幅/滤波唯一入口**：`include/FlightCtrlParams.h` `kFlightCtrlParams`
  （static constexpr，固件与宿主测试共用）；旧参数 0.28/0.20/0.10 不得导入。
  **三机型速率环统一为 α 域**：kp 单位 s⁻¹、ki 单位 s⁻²（输出 deg/s²）；
  单涵道的限幅由机械摆角 `sedf.ctrl_vane_deg` 经 `applyFlightCtrlParams()` 内部换算，
  用户不需要也不应该直接改 `rate_*.out_max`（下次 apply 会覆盖）。
- 单一定义源：`include/TandemVec_Config.h`（kT/kQ/wMax/I/a/b/dMax/m/g/ServoConfig）。
- 差速增益调度/工作点下限/零油门门控：`flight_control.cpp` 层2（改后跑 `tools/verify_*.py`）。
- 控制链遥测：`state_data.h` §4.0b `gnc_tel`。
- 舵机行程/方向/中位：`TandemVec_Config.h` §ServoConfig。

## 执行原则

- 先读上下文再动手（`git status --short` + 相关源码 + `platformio.ini` + 本文件）；先诊断后行动。
- **器件手册/驱动先查资产索引**：`../docs/08-参考资料/器件手册与驱动资产索引.md` + `TandemVec_FCS/lib/<器件>/doc/`；项目已有 ICM-42688-P（DS-000347 v1.2）、ICM-45686（DS-000577）、DPS310（V1.1）官方手册及核对报告，**禁止未查就上网重复下载**（2026-11-03 教训）。
- 小步、聚焦、可验证；不覆盖用户未提交改动（`main.cpp`/`.vscode`/`lib`/当前打开文件）。
- 新增/修改的注释、文档、提交说明用中文；命令/路径/标识符原样。全部思考过程用中文。
- 同一问题连续改 3 次未解决 → 停下，重新按工程式分析根因/约束/影响面/验证方案。

## 硬件与串口

- 电路权威参考：`docs/电路拓扑参考.md`（52 引脚/型号/连接器）；改引脚前必读。
- **WS2812 灯效（PD15 / TIM4_CH4）正本 = `docs/memory/2026-08-08-WS2812驱动调试.md`**：
  H743 的 DMAMUX1 **没有 TIM4_CH4 请求**（只有 CH1-3 + UP），驱动走 TIM4_UP DMA 写 `CCR4`。
  ★ **灯不亮第一步读 `wsstat` 的 `driver` 字段**：`1`=TIM4_DMA（正常）、`0`=bitbang（残留）。
  `wsstatic` 会把驱动强制切 BITBANG 且不自动恢复——测过一次就必须 `wsmode 1` 复原，
  否则后面所有点灯测试都跑在 bitbang 上，看起来像「灯坏了」（2026-11-10 实际踩到，白查一轮）。
  复位后默认即 DMA；`wsmode 1` + `ws <r> <g> <b>` 能直接点亮。
- 串口/引脚/传感器驱动：改前读 `.agents/docs/hardware-reference.md`。
- **遥控通道语义（★ 共轴 CH8 与原版固件相反，遥控器必须按此配置开关方向）**：

  | 通道 | 功能 | 判据 | 与原版 `VTVL_ElectricDualRotor_FCS` |
  |---|---|---|---|
  | CH5 | 解锁 | 7 点中值 `> 1500` 解锁、`<= 1500` 上锁 | 等价 |
  | CH6 | 点火允许 | 中值 `> 1500` | 等价 |
  | CH7 | 飞行模式 | 五档（`math_utils.h` `getControlMode`） | 等价 |
  | CH8 | **手动 TVC** | **`> 1750` = 手动**；否则 = 飞控自稳 | ★ **相反**：原版 `raw < 1200` 才是手动 |
  | CH9 | 姿态/速率模式 | `< 1500` = ATTITUDE，`>= 1500` = RATE | 等价 |

  ★ **CH8 为什么反着来**：2026-08-10 用户明确要求「手动 TVC 开关反逻辑」——默认低位 = 飞控自稳介入，
  只有主动拨到高位才是手动演示/标定，**避免忘记打回自动导致意外手动旁路**。据此实现为 `> 1750`。
  ★ **判据**：DBG `vanes` 打印的 `manual_tvc` 字段必须是 0（自稳）/ 1（手动）；`ch5-8` 行显示 CH8 实际值。
  手动 TVC 打开时**未解锁也出舵**（地面标定必需），但电机始终停——与原版一致。
  ★ **CRSF 未配置通道返回 999**（实测 `ch5-8=999/999/999/999`），不是满低位 988；无效值判据用
  990–1010 区间（`flight_control.cpp` 的 `rc_switch_invalid`）。`999 > 1750` 为假 ⇒ 无信号时自动落到自稳分支，安全。
  ★ 对照记录：`../docs/07-验证与确认/VER-004-共轴机型原始固件对照-控制与安全.md` §2.1 与 D3／A-9。
- **Serial6(USART6) 波特率** = `state_data.h` `SERIAL6_BAUDRATE`（当前 921600；2M 在 VCP 丢帧 ~15%）。
- **DBG 调试**：Serial6 收 `"DBG\n"` 进调试（`handleDebugConsole`），`exit` 退出；命令表与工具链见 `../docs/registers/固件调试工具链-2026-11-01.md`。
- **站点磁偏（2026-11-07）**：磁偏默认取长沙 IGRF-14 常数 −4.114°（`include/mag_reference.h` 的 `kDefaultMagReference`）。异地飞行前用 DBG `magdec <deg>` 现场修正（改 `bfs::g_mag_reference` 运行时副本的 `declination_rad`，场强/倾角仍用长沙表）；**未执行 `magdec` 时行为与改前逐位一致**。判据：跨纬度转场仍用长沙值，航向会带等量的系统偏差。
- **IMU 原始值（2026-11-07）**：`DBG imuraw [n]` 打印未校准/未滤波六轴（原始轴序 RUB，g/dps）与当前处理值（FRD）并排；判据是"差异只在频段高端 → 滤波/校准所为；raw 本身就抖 → 传感器/供电/机械问题，调滤波器无效"。对应 AnoVars `imu_raw_*`；另 `sat_cnt_f/t/w` 是分配层饱和**次数**（`gnc_tel.alloc_sat[]` 只是当拍 bool）。
- **磁标定（DBG `magcal`）**：`start`/`fit`/`save`（写内部 Flash bank2 末扇区 0x081E0000）；★ 禁止 EEPROM 仿真（同用末扇区）；引导校准 `magcal guided`。

- **DAPLink 烧录（2026-11-03）**：adapter speed 需降到 **20–100 kHz** 才稳定 `Verified OK`；高速下偶发 checksum mismatch（diff 为孤立位翻转）不是固件坏。OpenOCD `program` 后必须 `reset run`，否则板子停住。命令见 `../docs/registers/踩坑记录-AGENTS.md` 2026-11-03 条。
- **GNSS（NEO-F10N，protocol 40.00）**：F10 接口说明只收录 **CFG-VALSET/VALGET/VALDEL + CFG-MSGOUT**（PVT UART1 `0x20910007` p.143、EOE UART1 `0x20910160` p.142），**没有 CFG-MSG/CFG-PRT 章节**；固件保留 CFG-MSG 是实测兼容层（F10 仍 ACK）。CFG-MSG 8 字节 payload 的 rate 必须同时写 UART1/UART2。**配置 ACK 不等于有输出**，验收看 `ubxstat` 的 `pvt/eoe` 是否增长。详见 `../docs/04-数学建模/代码理论核查/F10N-GNSS配置与调试-2026-11-03.md`。
- **调度统计口径（2026-11-03）**：`tasks` 新增 `丢执行` 列（flag 未消费又被 ISR 置位次数）并打印 `TIM8 clk/psc/arr`；当前 240 MHz/PSC1/ARR37500/6.4 kHz。**`tasks`/`health` 走 DBG，打印会显著加剧丢拍，高频采样速率必须用非 DBG 的 0xF4 `imu_rate_hz`（`tvc record`）核对。** IMU 2 kHz 已由 P1（2kHz ISR+SPSC ring+Navigation 排空）解决，默认实测 mean≈2000、std≈70、ring_drop=0；详见 `../docs/04-数学建模/代码理论核查/IMU采样调试经验-2026-11-04.md`。
- **新增 `health` 命令**：一屏看模式/IMU 健康/气压/GNSS pvt-eoe-fix-sv/ EKF rpy-vel-pos/ Serial5 上行 rx-bad-age-txskip，以及 loop CPU/max_pass/window（不重置 tasks 统计）。

## 内存占用（2026-11-08 实测，SingleEdf4Vane）

- **RAM 335720 / 524288（64.0%）**，Flash 395408 / 2097152（18.9%）。
- RAM 大户（`arm-none-eabi-nm --size-sort`）：`nav_ekf` 84.3 KB（15 状态 EKF）、
  **5 个 Arduino Serial 实例各 20.7 KB 共 103 KB**、`flashLog` 25.0 KB。
- ★ **可优化但未做**：`SerialLP1`（LPUART1）当前**零引用**却占 20.7 KB；
  禁用可把 RAM 从 64% 降到约 60%。不做是因为要改 Arduino 框架的 UART 实例配置，
  为 4% 余量动框架不划算——等真需要更多 RAM 时再评估。

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
bash test_host/run_all.sh  # 宿主回归全量（74 段，以脚本实际为准；test_host/ 下共 119 个 .cpp）
bash test_host/fw_sil/run_acceptance.sh          # SIL 验收：真实固件+EKF，3 seeds
bash test_host/fw_sil/build_and_run_failsafe.sh  # SIL 失效保护/模式重入
```

- 宿主构建用 `tools/with-ascii-bin.sh`（subst ASCII 盘符、`BIN_DIR`=仓库 `tmp/`，不写系统 %TEMP%）。
- **重力口径**：导航/估计/控制的真值统一用 WGS-84 正常重力——固件侧 `nav_ekf.gravity_mps2()` / `ekf_gravity_mps2`（Somigliana，随经纬高），宿主测试侧 `ins_test::GravityWgs84()`（31.2304°N、5 m ≈9.794 m/s²）。`TandemVec_Config.h` 的 `g=9.79` 只作 1g 量程换算（`G_TO_MS2`）与 EKF 未初始化时的回退值，**不作导航真值**；用 9.79 构造比力会注入 ~0.004 m/s²，1 h 纯惯导可伪造出 24 km 级假漂移。
- 上传只在用户明确要求时：`pio run -t upload`；串口监视加超时。
  **两种烧录方式**（2026-11-07 实测都可用）：
  1. `pio run -e <env> -t upload` —— 走 `upload_protocol = cmsis-dap`，
     速率由根 `platformio.ini` 的 `[env] upload_speed = 100000` 统一设（100 kHz）。
  2. **OpenOCD 直烧**（目标机没有项目仓库时最省事，只需 bin + OpenOCD）：
     ```bash
     openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
       -c "adapter speed 100" \
       -c "program firmware.bin 0x08000000 verify reset exit"
     ```
     输出 `Verified OK` 即写入成功。
  ★ **`Verified OK` 只证明字节写进了 Flash，不证明固件是新的**。
    烧完必须读回验证：`py -3.12 tools/verify_board.py COM6 <期望机型>`
    （8 项判据，只读），其中「参数表数量 = `ANO_PARAMS_COUNT`」是版本锚点。
- 找不到 `pio` 必须明确报告，不得声称构建通过。
- 验证选择：文档改动 `git diff --check`；代码整理 `pio run`；控制/导航/通信/安全改动 `pio run` + `bash test_host/run_all.sh`（可按改动面只跑其中相关段）；无法运行必须说明原因。
- **不要用 `pio test -e test` 做回归**：`[env:test]` 是 STM32 板载自检固件环境（用 `pio run -e test`），PlatformIO 的 `test/` 只认嵌入式 Unity 单测；本项目所有回归都是宿主测试（g++ 直编，见 `test_host/run_all.sh`）。历史上 `test/ekf_host_regression.cpp`（4798 行宿主测试，自带 main）误放此处，导致 `pio test` 把固件源码 + 宿主 main + Unity 框架一起链接而报 `ld` 错误；2026-11-07 已彻底移除。
- **旧 `ekf_host_regression.cpp` 已删除，不要恢复**（2026-11-07）：它 4798 行 / 248 KB，能编译（需补链接 `lib/units/src/convang*.cpp`）但运行时必崩——默认栈下 `0xC0000409` 栈溢出；加 `-Wl,--stack,67108864` 后 `assert(report.is_open())` 失败（`.pio/ekf_host_regression_report.txt` 在 ASCII 盘符与中文路径下都打不开）；3 次尝试均失败。功能已被 `test_host/` 里拆分后的 `test_ekf_*.cpp` 系列覆盖，无需保留。
- **算法测试两阶段**：先理想模型（零噪声/零偏、WGS84 重力 `ins_test::GravityWgs84()`、解析真值）验证正确性，再真实噪声/零偏/振动/失锁评估工程效果；理想阶段失败修算法，真实阶段不达标先归因。
- **不声称未执行的构建/测试/上板验证通过。**
- **宿主 UBSan**：MinGW 没带 libubsan，用 `-fsanitize=undefined -fsanitize-undefined-trap-on-error`（命中 UB 直接 trap，不需要运行时库）。2026-11-03 ekf15 在 UBSan 下 116/0、无陷阱；命令模板见 `../docs/registers/固件调试工具链-2026-11-01.md`。
- **性能 A/B 宏（常驻 env 已于 2026-11-10 清理）**：`BFS_TASK_PHASE_STAGGER`（错开 ≥5ms 任务首次相位，治 200Hz 实跑约 155Hz 的 flag 合并）与 `BFS_NAV_EKF_DTCM`（`nav_ekf` 落 DTCM 0x20000000）两个宏仍保留在代码里，但**不再有常驻 env**——当时的 A/B 一直没做完（2026-11-03 挂起），且 P1 架构已改变调度负载，旧基线不再可比。要做时按 `platformio.ini` §新增组合 加两行临时 env；A/B 只看 `tasks` 的 avg/max/迟到 + `health`，不以单次 CPU 平均下结论。详见 `../docs/04-数学建模/代码理论核查/飞控实时性能基线-2026-11-03.md`。

## IMU 数据链与延迟排查（2026-11-10 实飞结论）

**采样率只有一个真源**：`src/imu_odr_config.h` 的 `bfs_imu_odr::kOdrHz`。
ODR 配置、排空 `kTargetRateHz`、批量帧 `s_fifo_frame_dt_s`、ISR 抽点频率**全部由它派生**，
禁止再在任何地方手写 1600/2000/3200。换档位只改那里，或编译期覆盖 `-DBFS_IMU_ODR_HZ=800`；
不在型号支持列表内会触发 `static_assert` 编译失败，不会烧出跑错档位的固件。
后端的 ODR 映射已改**严格匹配、禁止吸附**——不支持就直接 `begin()` 失败而不是跑到相邻档，
因为静默吸附会让"请求值"与"实际生效值"不同，正是 2026-11-10 延迟事故的成因之一。
ISR 抽点频率 == ODR：若两者不等，`health` 的 `isr_empty` 会立刻暴露（对齐后应为 0）。

**排查判据（`health` 直接看）**：
- `ring_size` 应长期为 0；持续非零＝排空欠量，控制用的就是旧数据
- `drain_f` 必须等于 `isr_f`；不等＝有帧没被消费
- `ring_drop` 必须为 0
- `rate` 是实际帧率，与配置值可能差几个百分点（芯片 RC 时钟偏差；实测 45686 配 1600 实跑 1683）

**已排空的误判方向（不要重走）**：EKF 姿态延迟、Madgwick 对照、EKF 零偏补偿、滤波器时间常数、
惯量估计、PID 增益——2026-11-10 全部验证排除。真正根因是 IMU ring 积压导致控制吃 73ms 前的数据。
完整过程与修复见 `docs/memory/2026-11-10-共轴控制延迟根因与调度优化.md`。

**Flash 黑匣子**：扇区擦除会整体卡死主循环 14~28ms（所有任务间隔 max 同步飙高），飞行构型禁用页写。
禁用时保留任务占位、函数内直接 `return`——删任务会改变调度序列，实测反而让 Navigation 恶化到 22~25ms。

**偏航换算**：`diff = (I_yaw/k_yaw)·α_rad` 只用比值。改 `yaw_moment_nm_per_us` 必须同倍率改
`rate_yaw` 的 kp/ki/out_max/int_limit 四项，否则差速输出瞬间变化数百倍。

## 仓库卫生

- 改前后 `git status --short`；提交前 `git diff --check`。
- **动仓库前先 `git fetch` 比对 `origin/main`**（revo 与 magicbook 两会话共用本仓库）。
  远端领先时用 `git show --stat` 逐个看过再 `merge --ff-only`；禁止 `push --force`
  与 `reset --hard origin/main`。完整规范见根 `AGENTS.md` §多会话并发与变更联动规范。
- 改机型默认值/参数数量时，按根 `AGENTS.md` 的联动清单同步宿主测试、GCS 参数表与文档，
  并跑全量 `test_host/run_all.sh`（2026-11-06 曾因只跑单文件漏改 P10 断言）。
- `.pio/`、`compile_commands.json`、本地 VS Code 索引/调试产物不提交。
- 上下文文件不改成参数流水账。
- COM 口独占：`pio device monitor` 加 timeout；PowerShell 串口脚本 `try/finally` + `ReadTimeout=500ms`。
- 运行时抓包文件用完清理；串口拒绝访问先 `taskkill /f /im pio.exe` 或杀 PowerShell。

## 实机安全注意事项

建议任何实机测试前，必须提醒断开电机动力电源（除非请求明确不涉及实时输出）。飞控逻辑改动分阶段：编译检查 → 传感器初始化/遥测 → 断动力查执行机构方向与限位 → 系留低功率 → 自由飞。

- **数传链路验收（2026-11-04）**：0xF4从100Hz降至10Hz、0x0A/0x21/0xF1从100Hz降至50Hz后，120s记录行率39.8Hz、gap p99=41.6ms、max47.7ms、无>100ms；参数（当时 148 项）load 双确认 148/148。验收脚本 `tools/verify_telemetry.py`（rate≥35、p99≤60ms、max≤100ms、ring_drop=0）。
