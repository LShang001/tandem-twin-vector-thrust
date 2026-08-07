# AGENTS.md — TandemVec_FCS 飞控固件

> 本仓库是基于 PlatformIO 的 STM32H743 **纵列双发矢量推力 VTOL 飞控固件**（从原共轴双桨 VTVL 移植重构）。处理本项目时，必须把它视为安全关键的嵌入式控制软件，而不是普通应用代码。

## 坐标系统一约定（2026-08-02 钉死，全仓引用）

- **机体系 FRD**：`x_b` = 机身纵轴 = **电机推力轴**（前电机 +x 端拉力式、尾电机 −x 端推进式）；`y_b` 右；`z_b` 下（垂直于纵轴）。
- **执行器–力矩映射（机械布局，不随姿态变）**：前摆 δ_f（绕 z_b）→ Mz 偏航通道；尾摆 δ_t（绕 y_b）→ My 俯仰通道；差速 Δω → Mx 滚转通道。
- **VTOL 悬停构型**：机头朝天（x_b 竖直 ∥ NED −z）——差速（绕 x_b）= **世界航向**；z_b 悬停时水平。
- **RATE_MODE 摇杆映射（四轴式，悬停构型）**：yaw 摇杆 → 差速（绕 x_b，保守幅值 MAX_MANUAL_yawRATE=35°/s）；roll 摇杆 → 前摆（绕 z_b 倾斜，MAX_MANUAL_rollRATE）；pitch 摇杆 → 尾摆（绕 y_b）。ATTITUDE_MODE 姿态环同为四轴语义（q_err.x→差速、q_err.y→尾摆、q_err.z→前摆）。
- ⚠️ **NED z（世界竖直）≠ 机体 z_b**：推力垂直补偿用 R13（x_b 投影）；激光斜距用 R33（激光沿 −z_b）。
- 详细推导见 `docs/04-数学建模/MOD-002` §1.2。

## 飞行器构型与控制分配

- **纵列双发**：前电机(CW)绕z_b摆动(δ_f, **偏航主控**)，尾电机(CCW)绕y_b摆动(δ_t, 俯仰主控)
- **差速反扭**：Δω绕推力轴(x_b)产生Mx——**水平巡航=滚转；垂直悬停（x_b竖直）=世界航向**
- **物理逆解**：α→I×α→M_cmd→allocateMoments(BTRUE)→δ_f/δ_t/Δω
- **执行器映射**：前摆δ_f→PA0(TVC_ROLL), 尾摆δ_t→PA1(TVC_PITCH), Δω→PA2/PA3(前后电机差速)
- **齿轮传动**：舵机30T/摆座40T = 1.333:1，PWM映射已含齿轮比

## 当前约定（含已修复遗留）

- **机体系 = 原始 VTVL 实飞存档版约定**（`D:\PIO_Projects\VTVL_ElectricDualRotor_FCS_原始飞行存档版`）：`x_b`=前、`y_b`=右、**`z_b`=下=推力轴**（机头朝天时 z_b 指向地面）。★ 判据：**机头竖直朝天静止解算 roll=pitch=0**（悬停基态），天然避开欧拉奇异点。板子安装与存档一致 → 控制响应逻辑必须与存档一致。
- 推力垂直投影用 **R33** = `1-2(qx²+qy²)`（z_b 在 NED 垂直方向投影；悬停 roll=pitch=0 → R33=1 无需补偿），与存档一致。**勿改用 R13**（那是"x_b 竖直"错误映射时期的产物）。
- 目标姿态合成 = 标准 `Rz(Heading) ⊗ Rxy(rollTarget, pitchTarget)`（存档原版），**勿加悬停基态补偿**（`q_hover ⊗ Rx(-Heading)` 是错误映射时期的产物）。
- 命名残留：`servo_deg_roll`/`TVC_ROLL_SERVO_PIN` 实际是前摆，待重命名（不影响功能）。
- 调试开关宏陷阱：`#define XXX_TEST 0` 时 `#ifdef XXX_TEST` 仍为真——调试开关判断一律用 `#if`（2026-08-07 `GYRO_DIRECT_TEST` 曾因此常驻生效，姿态环被整体旁路、打杆无响应）。
- **IMU 轴映射（`sensor_imu.cpp` 步骤5，与存档逐字一致，勿改）**：`bX=-sZ`、`bY=+sX`、`bZ=-sY`（传感器 RUB=右/上/后）。手性 `bX×bY=-sY=bZ` ✓。判据：机头朝天静止 `acc≈(0, 0, -1)g` 且 **roll=pitch≈0**。⚠️ 2026-08-07 曾误改为"x_b 竖直"（`bX=+sY`），导致 pitch≈+89° 落进万向锁、roll 与 Heading 退化耦合（实测 roll=-110/Heading=-121，二者和≈-231），打杆与目标姿态关系错乱；连带 Madgwick 输入（步骤6）、`q_body_from_FLU`（步骤8 = `{0,1,0,0}` 绕 X 转 180°）、`cos_tilt`、目标姿态合成、mix 轴置换、PID 换位共 6 处全部被污染。**改轴映射前先问：是否与存档一致？**
- `Quaternion` 结构体分量顺序是 **`{w, x, y, z}`**（`QuaternionMath.h:49`），不是 `(x,y,z,w)`；写常量四元数前先核对（2026-08-07 曾写错导致姿态基准错位）。
- 符号/轴排查顺序：**先查轴映射，再查控制律符号**。轴错会让"通道张冠李戴"（绕竖直轴转→前摆动作）伪装成"符号反"，在错轴上翻符号只会掩盖问题（2026-08-07 曾为此翻了 3 轮直通符号）。
- **mix 层轴置换**（`flight_control.cpp` 层1）：控制律在**存档系**（roll 绕 x_b、pitch 绕 y_b、yaw 绕 z_b=推力轴），分配器 `allocateMoments` 吃**模型系**（x'=推力轴朝机头、y'=尾摆、z'=前摆），`x'=-z_b, y'=+y_b, z'=+x_b`（det=+1）。故 `Mx'=-Ix·alpha_yaw`（差速）、`My'=+Iy·alpha_pitch`（尾摆）、`Mz'=-Iz·alpha_roll`（前摆）。符号以"实机已验证的陀螺直通行为"为锚点反解（`tools/verify_mix_axes.py`），勿凭几何直觉、勿改 `computeEffectMatrix`（2026-08-07）。
- **通道↔执行器对应（存档系）**：roll 通道→**前摆舵机**、pitch 通道→**尾摆舵机**、yaw 通道→**电机差速**。PID 参数须随之匹配：TVC 轴可高带宽（Kp_r=0.25/Kp_a=2.5，ζ≈1.20），差速轴受电机 τm=0.28s 限带必须保守（Kp_r=0.10/Kp_a=0.8，ζ≈1.34）。
- **摇杆语义（与存档一致）**：ATTITUDE_MODE = roll/pitch 杆控**目标姿态角**、yaw 杆控**航向角速度**（不做航向姿态回中，`execute_yaw_controller` 两种模式统一处理）；RATE_MODE = 三杆全控目标角速度。
- `include/TVC_Control_Geometric.h` / `TVC_Control_3rdOrder_Poly.h` — 原版 TVC 几何模型，**已弃用**。控制分配现由 `TandemVec_ControlAllocation.h` 处理。

## 核心参数

- **控制增益/限幅/滤波 ★ 实机调参唯一入口**：`include/FlightCtrlParams.h` `kFlightCtrlParams`（2026-08-08 C路径重构：12 个 PID 增益+输出/积分限幅+阈值+微分滤波、7 个控制滤波器 alpha，全部集中于此结构体；static constexpr，**固件与宿主机测试共用同一事实源**——`state_data.cpp` PID 构造、`main.cpp` setup、`flight_control.cpp` initPositionHold 与 `test_host/test_flight_control_axis.cpp` 均读它）。数值域为 PositionPID 实际语义（deg 域）。
- 单一定义源：`include/TandemVec_Config.h`（kT/kQ/wMax/I/a/b/dMax/m/g/ServoConfig）
- 差速增益调度/工作点下限/零油门门控：`src/flight_control.cpp` 层2（mix 函数，硬编码公式，改后需跑 `tools/verify_*.py`）
- 控制链遥测：`src/state_data.h` §4.0b `gnc_tel`（每层中间量：error_deg/omega_ref_dps/alpha_ref/M_cmd/w0_eff/yaw_gain_sched/执行器指令+饱和），CAN/AnoCom/Serial8 均读它
- ⚠️ CascadeCtrl 半成品架构已于 2026-08-08 **废弃删除**（`TandemVec_AttitudeCtrl/RateCtrl/CascadeCtrl/CtrlParams.h` 已删；浮点四元数工具抽为 `include/Quat4f.h` 供 test_host 使用；`test_tandemvec_cascade/sim.cpp` 两测试已随架构一并删除，run_all.sh 仅保留 allocation 测试）
- 舵机行程/方向/中位：`include/TandemVec_Config.h` §ServoConfig `kDefaultServoConfig`（`half_travel_deg` 默认45°待标定、`dir_pitch/dir_roll` 已实机核查、`zero_*_pct` 待标定；mix 函数消费）

## 执行总原则

- 先读上下文再动手：修改前至少检查 `git status --short`，并阅读与任务直接相关的源码、`platformio.ini` 和本文件。
- 先诊断，后行动：不要看到表面症状就立刻改代码，先识别根因。
- 小步、聚焦、可验证：最小必要修改，不做大范围格式化、机械清理、顺手重构或无关依赖升级。
- 不要覆盖用户改动：尤其不要覆盖 `src/main.cpp`、`.vscode/`、`lib/` 或当前打开文件中的未提交内容。
- 中文规则：本项目新增或修改的注释、README、说明文档、提交说明必须使用中文；命令、路径、文件名、API 名、协议名、库名和代码标识符按原文保留。保留已有中文技术注释，除非注释明确错误且当前任务要求更新。
- **语言规则（最高优先级）**：所有思考过程（thinking）、内部推理、分析、决策、自我对话，必须使用中文。唯一允许保留英文的部分：代码片段、命令行、函数名、变量名、技术专有名词、路径、文件名、API 名、协议名、库名。

## 诊断与失败处理

- 同一问题连续修改 3 次仍未解决时，必须停下，重新按工程式分析根因、约束、影响面和验证方案。
- 方案失败后优先追问"为什么失败、根因是什么"，不要无依据地连续换方案；应回溯假设、边界条件、环境差异和理解偏差。

## 调试工具链（2026-08-08 实战沉淀，全部验证过）

- **固件内调试模式（首选诊断手段）**：Serial6 收到 `"DBG\n"` 进入调试模式（`handleDebugConsole`），命令 `help/ws/wsmode/wsfault/gpio/tim4/ver/exit`。设计要点：①与地面站协议互斥共存（`DBG\n` 不会与 AnoCom 帧头 0xAB 冲突）；②调试命令直接读**寄存器**（MODER/AFR/ARR/CCR4/NDTR/DMAMUX）而非猜参数；③退出 `exit` 恢复数传。**任何新外设调试都优先加一个寄存器转储命令，比反复烧录试参数快一个数量级。**
- **OpenOCD 定位卡死**：`openocd -s <scripts> -f cfg -c "reset run; sleep 2000; halt"` 读 PC，再用 `arm-none-eabi-addr2line -e firmware.elf -f -p <PC>` 映射回源码函数——几分钟定位"卡在哪个函数"，不用猜。
- **★ xPack OpenOCD 的 stdout 输出会被吞**（`reg`/`mdw`/`mrw` 的结果进不了 stdout，脚本里 echo 也部分丢失）：遇到输出异常，**放弃 OpenOCD 读寄存器，改用固件内调试命令打印**（如 `gpio`/`tim4`）——固件自己读寄存器从串口输出，100% 可靠。
- **DAPLink 通信紊乱恢复**：OpenOCD 反复连接后出现 `CMSIS-DAP command mismatch` / `init failed` → **物理拔插 DAPLink USB 线**即可恢复（软件复位无效）。
- **python 串口脚本**（比 pio device monitor 灵活）：`python -c "import serial; ser=serial.Serial('COM10',2000000); ser.write(b'DBG\n'); print(ser.read(4096).decode('utf-8','replace'))"`——可编程、可发二进制、可扫波特率。注意 Serial6 波特率以 `SERIAL6_BAUDRATE` 宏为准（当前 2M）。
- **跨平台编译验证**：库要支持多平台时，在 /tmp 建独立 PlatformIO 项目（仅库 + 最小 main.cpp），用目标板 env（如 genericSTM32F407）编译——**无硬件也能验证编译层**，不污染主项目。
- **分层验证外设不亮**：①GPIO 配置（MODER/AFR）→ ②静态电平输出 → ③协议时序（寄存器/示波器）→ ④供电接线。每层有明确通过判据，避免在错误层反复试。

## 项目背景

- 固件主入口：`src/main.cpp`，只保留 `setup()`、`loop()` 和必须位于单编译单元的初始化/调度胶水逻辑。
- 本地库位于 `lib/`；其中很多是随项目带入的第三方依赖，不要随意格式化、重构或替换。

## 硬件与串口

- 电路权威参考：`docs/电路拓扑参考.md`（2026-06-30 经 EDA 实测修正），包含全部 52 个 MCU 引脚映射、传感器型号、连接器定义和未使用硬件资源清单。修改任何引脚相关代码前必须先对照该文档。
- 串口分配总览、已验证硬件事实、未使用硬件资源表：**修改串口/引脚/传感器驱动代码或排查通信问题前**，读 `.agents/docs/hardware-reference.md`。
- **Serial6 (USART6) 波特率 = `state_data.h` 的 `SERIAL6_BAUDRATE` 宏**（当前 2,000,000，2026-08-07 与 2.4G 数传对齐）——换数传模块时只改宏；抓串口前先查宏值，波特率不匹配会全乱码（来源：2026-08-08 WS2812 调试）。
- **调试串口**：Serial6 收到 `"DBG\n"` 进入调试模式（`handleDebugConsole`），`exit` 退出；命令 `help/ws <r g b>/wsoff/wsseq/wsstat/wsmode <0|1>/wsfault <0|1>/gpio/tim4/ver`。地面站帧头 0xAB 与 "DBG\n" 不冲突。详细命令表见 `docs/memory/2026-08-08-WS2812驱动调试.md`。

## WS2812 / 踩坑记录（2026-08-08）

- **★ W25N01GV NAND Flash (SPI3) 已点亮**（2026-08-08）：驱动 `lib/W25N01GV/`（命令层 + 日志层：RAM 环形缓冲/后台批量写/游标持久化/坏块跳过）。JEDEC ID 0xEF/0xAA/0x21 验证通过，写读回+掉电恢复+续写全通。调试命令 `flash id|erase|stat|dump|test`。
- **★ SPI 硬件 NSS 与手动 CS 冲突**：`SPIClass(mosi,miso,sclk,ssel)` 传 ssel 引脚 → `SPI_NSS_HARD_OUTPUT` + `pinmap_pinout` 覆盖 GPIO 配置 → 读 ID 全 0x6E。**SSEL 必须传 NC（软件 NSS）**，CS 由驱动 `digitalWrite` 手动控制（来源：2026-08-08 W25N01GV 点亮调试）。
- **★ WS2812 (PD15) 已点亮，通用库化完成**：驱动 = `lib/WS2812Driver/`（跨平台库：stm32duino BSRR bitbang + STM32H7 TIM_UP DMA + 通用 fallback）。应用状态机在 `include/Ws2812AppStatus.h`。状态：待机蓝呼吸/解锁绿闪/校准黄闪/故障红闪。
- **★ STM_GPIO_PIN(pn) 返回位掩码（1<<pin），不是位编号**（PinNamesTypes.h）：写 `1<<STM_GPIO_PIN(pn)` 是 UB，掩码/GPIO_Pin 变垃圾值 → HAL_GPIO_Init 配错引脚、AFR 不写 → 灯不亮。**直接用返回值**（来源：2026-08-08 库化重构定位出）。
- **DMA1_Stream6_IRQHandler 是 weak→Default_Handler**（startup），项目无强定义；`HAL_DMA_Start_IT` 不使能 NVIC → 中断/回调永不触发。**用 HAL_DMA_Start + PollForTransfer（轮询）**最稳（来源：2026-08-08）。
- **H743 TIM4 独缺 CH4 的 DMAMUX 请求**（只有 CH1/2/3/UP）：PD15=TIM4_CH4 上 `HAL_TIM_PWM_Start_DMA(CH4)` 硬件不触发。绕道用 TIM4_UP 的 DMA 写 `&TIM4->CCR4`，详见 `docs/memory/2026-08-08-WS2812驱动调试.md` §1。
- **Adafruit NeoPixel 库在 stm32duino 上高位引脚（PD8-15/PE8-15）不可用**：库用 `volatile uint8_t*` 写 BSRR，只能操作 GPIO 0-7（来源：2026-08-08，PD15 灯不亮定位出）。本项目 WS2812 必须自实现驱动。
- **DWT CYCCNT 时序等待在 -O3 + 关中断下会死锁**（飞控整个卡死）：Cortex-M7 需 `dwt_access(true)` 解锁 LAR（`SrcWrapper/src/stm32/dwt.c`），且 -O3 内联下计数行为不稳。**改用纯 NOP 循环延时最可靠**。
- **H7 RCC 寄存器与 F4 不同**：`RCC_CFGR_PPRE1` 在 H743 不存在，用 `RCC_D2CFGR_D2PPRE1`；APB1 timer clock = APB1×2（APB1 分频≠1 时）；DMAMUX1 无独立时钟使能位，与 DMA1 共享 AHB1 时钟（`__HAL_RCC_DMA1_CLK_ENABLE` 即可）。
- **PWM+DMA 两个必踩**：① `ARR = clk/800000-1`（不是 /800，漏零则 16 位 ARR 溢出截断，PWM 频率全错）；② DMA 传输完成回调里必须关外设请求使能位（TIM_UDE 等），否则残留请求卡死 DMA 状态机、下帧启动 HAL_BUSY → 灯闪。
- **重构教训**：所有模式共用的资源（引脚掩码）必须在**构造函数**初始化，不能放在某个模式的 begin 里（DMA 模式跳过 bitbang begin → 掩码恒 0）。

## 模块化架构

- 除明确的头文件工具函数外，模块间状态共享应通过 `state_data.h` 暴露的全局状态完成。
- EKF 内部始终保留 WGS84 绝对 LLA；`origin_*` 表示解锁时的起飞点；`relative_*` 与 `INS_GNSS_Packet.location_*` 为相对起飞点的 NED 位置；控制和遥测高度使用 `-relative_down`（向上为正）。DPS310 对外发布相对起飞点高度，进入 EKF 前必须通过 `origin_alt_m + baro_altitude` 恢复为与 GNSS 一致的绝对高度观测，不得直接把相对值传给 `MeasurementUpdateBaroAltitudeDetailed()`。
- `test_host/` 中的级联控制与闭环仿真按实际 `flight_control.cpp` 控制律、参数、单位和轴映射建立，用于宿主机验证固件行为；它们不是待接管固件输出的另一套生产控制链，不要据此提出架构统一或双链切换（来源：2026-07-29；**2026-08-08 用户最终确认废弃 CascadeCtrl 接入路线**，相关头文件与测试已删除，实机体系为准）。
- `include/` 下的头文件（ins_static_detector / ins_gnss_dynamic_weight / ins_gnss_epoch_timing / ins_static_aid_profile）**不依赖 Arduino / STM32 HAL**，可用 `g++` 在宿主机直接编译回归；修改后应宿主机单独验证，不必每次都走 `pio run` 完整编译。

## 硬性代码边界

- `DETA100_module.h` 仍包含解析函数实现和内部静态解析状态，只能在 `main.cpp` 中包含，避免多编译单元重复定义。
- 其他模块通过 `state_data.h` 访问 DETA100 类型；`state_data.h` 已包含 `deta100_types.h`。
- **`NavDataSource` 与任务条件注册**：`state_data.h` 中的 `NavDataSource::INTERNAL / DETA100` 枚举由 `setup()` 上电检测阶段锁定（在 Serial4 上检测 DETA100 协议帧，窗口期结束后不可更改）。`handleDeta100` 任务仅在 `nav_data_source == NavDataSource::DETA100` 时才注册，且必须在状态估计层（`handleNavigationSystem` 等）之前注册，确保同周期内下游读到最新数据。不得在不理解此机制的情况下单独启停 DETA100 相关功能或调整注册顺序。
- `QuaternionMath.h` 的工具函数按 header-only 方式使用 `inline`，可被需要四元数/向量运算的模块包含。
- `GeoDisplacement.h` 的工具函数应保持 `inline`，避免后续多模块包含时产生链接重复定义。
- `Vector3` 类型使用 `VECTOR3_TYPE_GUARD` 防止与 `QuaternionMath.h` 重复定义。
- `MAVLink` 和 `AnoCom` 共用 `Serial6`。除非代码被明确改造成可仲裁共存，否则应按互斥使用处理。
- `lib/MAVLink` 是本仓库跟踪的 vendor tree，不是子模块；除非问题明确位于协议库内部，否则不要编辑该目录。

## 安全关键修改规则

未经用户明确要求或批准，不要改动以下内容：

- 硬件引脚映射、串口分配、波特率、PWM 范围、舵机中心和限幅。
- 任务频率、定时器配置、调度周期和控制环周期。
- PID 参数、控制增益、混控比例、TVC 摆角限制。
- 传感器坐标方向、FRD/NED 约定、安装方向补偿。
- 解锁逻辑、故障保护阈值、超时处理、模式降级逻辑、输出限幅。

如果任务确实要求修改控制逻辑、状态估计逻辑、混控逻辑、解锁逻辑、串口协议或遥测协议，必须在最终回复中清楚说明：

- 影响的飞控行为。
- 可能改变的输入/输出、模式切换或故障保护路径。
- 已完成的验证和仍需上板验证的风险点。

不得为了让编译通过而移除安全检查、模式降级、超时处理或输出限幅。

## 构建与验证

在仓库根目录使用 PowerShell：

```powershell
pio run                    # 编译固件
pio test -e test           # 宿主机回归测试
bash test_host/run_all.sh  # 宿主机 g++ 平台无关回归（16 套件含 EKF 15 状态）
```

- `run_all.sh` 需要 `$env:BIN_DIR = "$env:TEMP\tv_fcs_bin"`：**必须覆盖**，MSYS bash 的路径转换层对中文路径不可靠（g++ 链接阶段写中文路径报 Invalid argument）。EKF 测试（`test_ekf_15state.cpp`，需 `-DEKF_HOST_REGRESSION` + units convang 源）已封装在脚本内。
- 单模块仿真（不依赖 bash）：`g++ -std=c++17 -Iinclude -Itest_host/stub test_host/test_tandemvec_sim.cpp -o test_host/bin/ts && ./test_host/bin/ts`
- EKF 测试的重力输入与 `TandemVec_Config.h` 统一（g=9.79，唯一事实源），勿改回 9.81。
- 只有在用户明确要求时才执行上传：`pio run -t upload`。串口监视：`pio device monitor`。
- 如果 PATH 中找不到 `pio`，必须明确报告这一点，不能声称构建通过。除非用户要求修复环境，否则不要安装或修改全局工具链。本机历史上可用的 PlatformIO 路径为 `C:\Users\12631\.platformio\penv\Scripts\pio.exe`，但仍应以当前 `Get-Command pio` 和实际命令输出为准。

## 验证选择

- 文档或注释修改：至少检查 `git diff --check`，并确认内容与 `platformio.ini`、实际源码结构不冲突。
- 非行为性代码整理：运行 `pio run`；如果影响测试入口或公共工具函数，再运行 `pio test -e test`。
- 控制、导航、通信、安全相关修改：优先运行 `pio run` 和 `pio test -e test`；无法运行时必须说明原因。
- 不要声称"通过"没有实际执行的构建、测试或上板验证。

## 仓库卫生

- 修改前后检查 `git status --short`；提交前检查 `git diff --check`。
- `.pio/`、`compile_commands.json`、本地 VS Code 索引和调试产物是生成产物，不应提交，除非用户明确要求更新相关生成文件。
- 根目录文档应与 `platformio.ini` 和实际固件结构保持同步。
- 不要把 `README.md`、`AGENTS.md` 或其他上下文文件改成参数流水账；只记录稳定、可复用、能指导后续代理工作的规则。
- **COM 口是独占资源**：`pio device monitor` 必须加超时（`timeout 8 pio device monitor -p COM10 -b 921600`），避免无限阻塞；不要用 Ctrl+C 硬杀。PowerShell 串口脚本必须 `try/finally` 关闭端口、读写超时设短（`ReadTimeout = 500ms`）、用完立即删除自身。
- 串口数据抓取文件（`ano_raw.bin`、`*.ps1`、`/tmp/*.txt`）是运行时产物，分析完毕后立即清理，不堆积在仓库目录、不提交 git。`analyze_ano.js` 是项目级分析工具，应保留在仓库根目录，不受上述清理规则约束。
- 残留进程清理：如果串口突然全部拒绝访问，通常是上次会话的 `pio device monitor` 或 PowerShell 进程未退出，用 `taskkill /f /im pio.exe` 或 `Get-Process powershell | Stop-Process -Force` 清理。

## 实机安全注意事项

在建议任何实机测试前，除非请求明确不涉及实时输出，必须提醒用户断开电机动力电源，或用其他方式让推进系统处于安全状态。

涉及飞控逻辑修改时，优先采用分阶段验证：

1. 编译检查。
2. 传感器初始化与串口遥测检查。
3. 在推进动力断开的条件下检查执行机构方向和限位。
4. 进行系留或其他约束条件下的低功率测试。
5. 只有在以上步骤均验证通过后，才考虑自由飞行测试。
