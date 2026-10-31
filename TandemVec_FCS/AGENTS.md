# AGENTS.md — TandemVec_FCS 飞控固件

> 本仓库是基于 PlatformIO 的 STM32H743 **纵列双发矢量推力 VTOL 飞控固件**（从原共轴双桨 VTVL 移植重构）。处理本项目时，必须把它视为安全关键的嵌入式控制软件，而不是普通应用代码。

## 坐标系统一约定（2026-08-02 钉死，全仓引用）

- **机体系 FRD**：`x_b` = 机身纵轴（前电机 +x 端拉力式、尾电机 −x 端推进式）；`y_b` 右；`z_b` 下（垂直于纵轴）。**★ 电机推力轴 = z_b**（悬停构型机头朝天：z_b 竖直指地、x_b 水平——2026-08-07 轴置换后实码语义，见 mix 层 1250 行；旧"x_b=推力轴"描述错误，2026-08-09 修正）。
- **执行器–力矩映射（2026-08-07 轴置换后，悬停构型语义；★权威 = mix 层 `flight_control.cpp` 1202-1252 行）**：**上摆 δ_f（前电机，机头朝天时在最高点）→ 滚转主控**（alpha_roll 体轴 x → Mz'）；**下摆 δ_t（尾电机）→ 俯仰主控**（alpha_pitch 体轴 y → My'）；**差速 Δω → 偏航主控**（alpha_yaw 体轴 z → Mx'）。旧描述"前摆=偏航、差速反扭滚转"为**巡航读法**（物理公式 Mz=a·Tf·sδf 在巡航系即偏航力矩、悬停系即侧倾力矩），已废止——写任何轴/执行器注释前先查 mix 层实码，勿凭概念名或旧注释。
- **VTOL 悬停构型**：机头朝天（x_b 竖直 ∥ NED −z）——差速（绕 x_b）= **世界航向**；z_b 悬停时水平。
- **RATE_MODE 摇杆映射（四轴式，悬停构型，★2026-08-11 FPV 摇杆曲线接入）**：三轴均经 `RcCurve.h` 曲线（归一化→死区→expo→rc_rate→super→限幅），参数 `rc_rate/rc_expo[3]/rc_super[3]`（FlightCtrlParams.h，在线可调）；默认 rc_rate=0.5/expo=0.25/super 0.7·0.7·0.55 → 满杆 roll/pitch 333°/s、yaw 222°/s（限幅 MAX_MANUAL_*RATE=600°/s，2026-08-11 由 80 解耦提高——旧值 35/50/80 全部过时）。roll 摇杆 → 上摆（前摆）；pitch 摇杆 → 下摆（尾摆）。ATTITUDE_MODE 姿态环（2026-08-07 轴置换后，★以 `flight_control.cpp` 1016-1023 行为准）：**q_err.x→上摆（roll 外环）、q_err.y→下摆（pitch 外环）、q_err.z→差速（yaw hold 外环）**——旧"q_err.x→差速"为置换前语义。
- ⚠️ **NED z（世界竖直）≠ 机体 z_b**：推力垂直补偿用 R13（x_b 投影）；激光斜距用 R33（激光沿 −z_b）。
- 详细推导见 `docs/04-数学建模/MOD-002` §1.2。
- ★ 通用变量上报（AnoVars：0xF2 值帧/0xF3 清单/MAVLink NAMED_VALUE_FLOAT/DBG vars 命令）与 MAVLink 双向（QGC 参数读写/命令/STATUSTEXT）见项目根 `AGENTS.md` 踩坑记录；加变量 = `src/ano_vars.cpp` kAnoVars 加一行宏。MAVLink 名称只有 9 个可见字符，新增变量须检查截断后唯一。
- ★ 编码统一（2026-08-11）：批处理内容必须 ASCII（cmd 用启动代码页解析、chcp 不救文件解析），中文输出走 Python（PYTHONUTF8=1）；详见根 `docs/registers/踩坑记录-AGENTS.md`。

## 飞行器构型与控制分配

- **纵列双发（悬停构型语义，2026-08-07 轴置换）**：前电机(CW)最高点=**上摆 δ_f，滚转主控**；尾电机(CCW)最低点=**下摆 δ_t，俯仰主控**
- **差速偏航**：Δω（alpha_yaw 绕推力轴 x_b）→ Mx'——**悬停/巡航统一 = 偏航主控**（旧"水平巡航=滚转"读法已废止，同公式在不同姿态系下的不同解读）
- **物理逆解**：速率 PID 在角度域输出 α(deg/s²) → `ControlUnits::dps2ToRadps2` → I×α(rad/s²) → M_cmd → allocateMoments(BTRUE) → δ_f/δ_t/Δω
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
- **通道↔执行器对应（2026-08-07 轴置换后）**：roll 通道→**上摆（前摆 δ_f）**、pitch 通道→**下摆（尾摆 δ_t）**、yaw 通道→**电机差速 Δω**。PID 参数原则：TVC 轴可高带宽、差速轴受电机 τm=0.28s 限带必须保守（**当前数值以 `include/FlightCtrlParams.h` kFlightCtrlParams 为唯一事实源，勿在此处维护数值**）。
- **摇杆语义（与存档一致）**：ATTITUDE_MODE = roll/pitch 杆控**目标姿态角**、yaw 杆控**航向角速度**（不做航向姿态回中，`execute_yaw_controller` 两种模式统一处理）；RATE_MODE = 三杆全控目标角速度。
- `include/TVC_Control_Geometric.h` / `TVC_Control_3rdOrder_Poly.h` — 原版 TVC 几何模型，**已弃用**。控制分配现由 `TandemVec_ControlAllocation.h` 处理。
- **时基与 IMU 抽象（2026-11-01）**：TIM8 6.4kHz；IMU 任务周期与 Madgwick 由 `IMU->actualAccelOdrHz()` 反推（2k→0.5ms、3.2k→0.3125ms）；IMU 经 `lib/ImuAbstraction` 抽象，默认 42688P，换 45686 用 `-DFCS_IMU_MODEL_ICM45686`；默认 LPF 陀螺 200Hz/加计 100Hz（降延迟，代价是噪声略升）；EKF `q_accel_scale=2.0`；hAcc 门控宏 `BFS_NAVIGATION_GNSS_HACC_GATE` 默认 0。细节见根 `AGENTS.md`。

## 核心参数

- **控制增益/限幅/滤波 ★ 实机调参唯一入口**：`include/FlightCtrlParams.h` `kFlightCtrlParams`（2026-08-08 C路径重构 + 2026-08-11 扩到 128 参数：12 环 PID 增益/限幅/阈值/滤波 + 滤波数组 + inertia_comp_mask + FPV 摇杆曲线 7 参数（rc_rate/rc_expo[3]/rc_super[3]），全部集中于此结构体；static constexpr，**固件与宿主机测试共用同一事实源**——`state_data.cpp` PID 构造、`main.cpp` setup、`flight_control.cpp` initPositionHold 与 `test_host/test_flight_control_axis.cpp` 均读它）。速率环输入 `deg/s`、输出 `deg/s²`：`kp` 为 `s⁻¹`，`ki` 为 `s⁻²`，输出/I项限幅为 `deg/s²`，积分分离阈值为 `deg/s`；旧参数 `0.28/0.20/0.10` 不得导入。
- 单一定义源：`include/TandemVec_Config.h`（kT/kQ/wMax/I/a/b/dMax/m/g/ServoConfig）
- 差速增益调度/工作点下限/零油门门控：`src/flight_control.cpp` 层2（mix 函数，硬编码公式，改后需跑 `tools/verify_*.py`）
- 控制链遥测：`src/state_data.h` §4.0b `gnc_tel`（每层中间量：error_deg/omega_ref_dps/alpha_ref_dps2/alpha_ref_radps2/M_cmd/w0_eff/yaw_gain_sched/执行器指令+饱和）。0x21/CAN/旧 `alpha_ref_*` 保持物理域 `rad/s²`；`adps2_x/y/z` 为角度域 `deg/s²`。
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
- **python 串口脚本**（比 pio device monitor 灵活）：`python -c "import serial; ser=serial.Serial('COM10',921600); ser.write(b'DBG\n'); print(ser.read(4096).decode('utf-8','replace'))"`——可编程、可发二进制、可扫波特率。注意 Serial6 波特率以 `SERIAL6_BAUDRATE` 宏为准（当前 921600，2026-08-10 COMM-001 由 2M 降频——VCP 丢帧 ~15%）。
- **跨平台编译验证**：库要支持多平台时，在 /tmp 建独立 PlatformIO 项目（仅库 + 最小 main.cpp），用目标板 env（如 genericSTM32F407）编译——**无硬件也能验证编译层**，不污染主项目。
- **分层验证外设不亮**：①GPIO 配置（MODER/AFR）→ ②静态电平输出 → ③协议时序（寄存器/示波器）→ ④供电接线。每层有明确通过判据，避免在错误层反复试。

## 项目背景

- 固件主入口：`src/main.cpp`，只保留 `setup()`、`loop()` 和必须位于单编译单元的初始化/调度胶水逻辑。
- 本地库位于 `lib/`；其中很多是随项目带入的第三方依赖，不要随意格式化、重构或替换。

## 硬件与串口

- 电路权威参考：`docs/电路拓扑参考.md`（2026-06-30 经 EDA 实测修正），包含全部 52 个 MCU 引脚映射、传感器型号、连接器定义和未使用硬件资源清单。修改任何引脚相关代码前必须先对照该文档。
- 串口分配总览、已验证硬件事实、未使用硬件资源表：**修改串口/引脚/传感器驱动代码或排查通信问题前**，读 `.agents/docs/hardware-reference.md`。
- **Serial6 (USART6) 波特率 = `state_data.h` 的 `SERIAL6_BAUDRATE` 宏**（当前 921,600，★2026-08-10 COMM-001 由 2M 降频：VCP 下 2M 丢帧 ~15%，921600 位时间翻倍、带宽余量仍 87%；2.4G 数传需重新对齐）——换数传模块时只改宏；抓串口前先查宏值，波特率不匹配会全乱码（来源：2026-08-08 WS2812 调试；2026-08-10 COMM-001 优化）。
- **调试串口**：Serial6 收到 `"DBG\n"` 进入调试模式（`handleDebugConsole`），`exit` 退出；命令 `help/ws <r g b>/wsoff/wsseq/wsstat/wsmode <0|1>/wsfault <0|1>/gpio/tim4/ver`。地面站帧头 0xAB 与 "DBG\n" 不冲突。详细命令表见 `docs/memory/2026-08-08-WS2812驱动调试.md`。
- **磁力计椭球标定（DBG `magcal`，2026-08-15）**：`magcal start`（绕三轴旋转采样本 ≥30）→ `magcal fit`（9 参数椭球拟合，质量门：正定/覆盖度/残差≤15%，通过自动应用）→ `magcal save`（写入**内部 Flash bank2 末扇区 0x081E0000**，magic+CRC，上电自动加载）。★ 该扇区为标定专用：**固件内禁止引入 EEPROM.h/stm32duino EEPROM 仿真**（同用末扇区，冲突），黑匣子 W25N01GV 全盘归 flashLog 环形覆盖、不可借页。标定在**芯片系**进行（与轴映射解耦），应用顺序 raw→标定→轴映射→FRD；解锁态（`raw_rc_values[4]>1500`）拒绝 start/save。数学层 `lib/MagCal/MagCalMath.h` 纯平台无关（宿主机 `test_mag_cal.cpp` 回归：合成椭球恢复精度 ~1uT@0.2uT 噪声）。
- **大疆式两圈引导校准（DBG `magcal guided`，2026-08-15）**：阶段1 水平旋转 → 阶段2 机头朝下旋转，各 ≥315°（磁方位 45°×8 扇区覆盖 ≥7）自动切换；判定输入 FRD 加速度/陀螺 + 姿态门（|acc|∈[0.85g,1.15g]、主导轴≥0.7|acc|、主导转速 0.3-6 rad/s、次轴<0.45×主导），两圈完成自动椭球拟合+应用。状态机 `lib/MagCal/MagCalGuide.h` 纯平台无关（宿主机 `test_mag_guide.cpp`）；灯效复用 CALIBRATING 黄快闪（`handleStatusLedTask` 的 is_calibrating 已并入 guideActive）；`magcal stop` 中止、`magcal fit`/`magcal start` 与引导互斥。

## WS2812 / 硬件踩坑记录（2026-08-08，通用教训保留，细节见 docs/memory/2026-08-08-WS2812驱动调试.md）

- **★ W25N01GV NAND Flash (SPI3) 已点亮**：驱动 `lib/W25N01GV/`（RAM 环形缓冲/后台批量写/游标持久化/坏块跳过），调试命令 `flash id|erase|stat|dump|test`。
- **★ SPI 硬件 NSS 与手动 CS 冲突**：`SPIClass(mosi,miso,sclk,ssel)` 传 ssel 引脚 → 硬件 NSS 覆盖 GPIO 配置 → 读 ID 全 0x6E。**SSEL 必须传 NC（软件 NSS）**，CS 手动 `digitalWrite`。
- **★ WS2812 (PD15) 已点亮并库化**：`lib/WS2812Driver/`（BSRR bitbang + TIM_UP DMA + fallback），应用状态机 `include/Ws2812AppStatus.h`（待机蓝/解锁绿/校准黄/故障红）。
- **★ 非对齐 float/int 访问 HardFault（ARM 通用）**：`uint8_t` 数组强转 `(const float*)` 读，地址未 4 字节对齐 → HardFault → **复位循环**（现象"复位后正常"，OpenOCD 抓假象，极难定位）。修复用 `memcpy` 逐元素拷贝。
- **★ 变长帧在混合帧流里靠定界符搜索不可靠**：payload 里的 0x00 误命中 `\0` 定界 → CRC 错 → 时有时无丢帧。**修复：固定长度 + pad**（CRC 固定位置）。通用教训：协议帧优先定长或帧内长度字段。
- **PWM+DMA 两个必踩**：① `ARR = clk/800000-1`（漏零则 16 位 ARR 溢出，频率全错）；② DMA 完成回调里必须关外设请求使能位（TIM_UDE 等），否则残留请求卡死 DMA 状态机、下帧 HAL_BUSY。
- **DMA1_Stream6_IRQHandler 是 weak→Default_Handler**（startup 无强定义），`HAL_DMA_Start_IT` 不使能 NVIC → 中断永不触发。**用 HAL_DMA_Start + PollForTransfer**。
- **调试命令可靠性**：`DBG\n` 检测必须在串口协议解析（receiveData）**之前**；调试模式下 Flash 写页与 CAN 互斥（`s_flashWriting`），否则 MCP2515 超时 reset 卡死 SPI2。
- **重构教训**：所有模式共用的资源（引脚掩码）必须在**构造函数**初始化，不能放在某个模式的 begin 里。

## 模块化架构

- 除明确的头文件工具函数外，模块间状态共享应通过 `state_data.h` 暴露的全局状态完成。
- EKF 内部始终保留 WGS84 绝对 LLA；`origin_*` 表示解锁时的起飞点；`relative_*` 与 `INS_GNSS_Packet.location_*` 为相对起飞点的 NED 位置；控制和遥测高度使用 `-relative_down`（向上为正）。DPS310 对外发布相对起飞点高度，进入 EKF 前必须通过 `origin_alt_m + baro_altitude` 恢复为与 GNSS 一致的绝对高度观测，不得直接把相对值传给 `MeasurementUpdateBaroAltitudeDetailed()`。
- **真实固件源码 SIL（2026-10-22）**：`test_host/fw_sil/` 不是“按源码重写”，而是直接链接 `src/flight_control.cpp` + `src/state_data.cpp`，仅桩掉 Arduino/STM32 外设、传感器和 MAVLink。运行：`bash ../tools/with-ascii-bin.sh --sub sil -- bash test_host/fw_sil/build_and_run.sh`（定高 10 m，构建产物在仓库 tmp/）或加 `--manual`（roll/pitch 阶跃）。已验证定高到 10.00 m、roll +5.86°/pitch -5.85°。`fw_sil_env_main.cpp` 追加 6DOF、IMU/GNSS/baro/风/地面，默认直注；`--ekf` 已编入真实 `ekf_15_state.h`，seed1 已验证 plant 9.16 m / EKF 10.00 m 定高闭环；SIL 加舵机 300°/s 速率限幅 + 0.1% PWM 量化后，无光流失锁水平漂移从 309 m 降到 151 m；5 seeds EKF 定高 plant 9.15–9.30 m / EKF 9.93–10.04 m；无光流失锁水平漂移 151–287 m；磁航向 `--mag` 默认关闭：`MeasurementUpdateYaw` 10 Hz 在多 seed 下会 NaN/发散；`--ekf` 本身 3 seed 稳定在 9.17–9.23 m。`--flow` 走真实 `MeasurementUpdateAidingVelocityDetailed`（机体系×0.95尺度→EKF姿态转NED）：无延迟 3 seeds 漂移 115/268/457 m，不稳定；0.1 s 延迟更差。仍是理想光流，不能当实机验证。它仍不是 `pio run` 二进制、不是 QEMU/Renode、不是实机。
- **GNSS/INS 延时补偿/高速精度（2026-10-22）**：新增宿主回归 `test_host/test_ekf_delay_highspeed.cpp`（理想 IMU，速度/延迟/GNSS 噪声扫描）与 `test_host/test_ekf_highspeed_realistic.cpp`（ICM-42688 标称噪声 + UBX PVT 1.5 m/0.05 m/s + 10 Hz + 50 ms）。30 m/s、50 ms 下年龄补偿位置 RMSE 1.458→0.164 m（−88.7%）；15 ms 仍 −61.7%。报告 `TandemVec-EKF-Report/卫惯组合导航-延时补偿-高速精度-2026-10-22.md`；`test_host/run_nav_acceptance.sh` 验收延时/杆臂/振动（0.30/0.20/1.10/5.00 m 门限），当前 PASS。`accel_std=0.10` 在无振动标称仿真中速度 RMSE 0.065→0.046 m/s，但 0.25 是实机振动保守值，**不得未采振动数据直接改默认参数**；0/0.1/0.3/0.5/1/2 g 20 Hz 单频振动 × 匀速/纵向/3D 机动矩阵已跑（3 seeds）：2 g pos3d 约 3.4 m；新增 40/80/120 Hz 多频 + 80 Hz 陀螺振动后，1 g 多频 pos3d 约 0.26–0.30 m，单频 1 g 约 1.7 m；20–200 Hz / 0.5 g 单频扫频显示 20 Hz 最差（0.878 m）、150 Hz 次差（0.314 m）、100/200 Hz 仅 0.132 m；10 s GNSS 失锁 + 0/1 g 振动 pos3d 约 0.10–0.17 / 0.86–0.96 m；新增航向转弯+杆臂：1 m 天线杆臂未补偿 pos3d 0.923 m、补偿后 0.098 m，未补偿杆臂可让延时收益变负——实机必须量天线到 IMU 安装向量；振动谱形状/频率与采样率关系/陀螺耦合/失锁/杆臂组合必须实测，不能只用一个 g 值描述；宿主自适应 Q 候选（高通 RMS <0.1→0.05，>0.5→0.25）在 0 g 提速、振动时退化为 0.25。GNSS 实际延迟需 PPS 标定，15 ms 只是编译常量；新增纯平台无关 `include/ins_gnss_pps_latency.h`（PPS 边沿+PVT 到达中位数估计，25/50 ms 回归 8/8），但硬件捕获尚未接入；无 PPS 批处理对齐原型 `test_host/test_gnss_delay_batch.cpp`：修复 A 矩阵三轴行覆盖和 t≈0 速度差分退化后，联合 `p0/v0/b_a` 在纵向/侧向+爬升 3D 机动 50 seeds × 20/50/80 ms 下平均绝对误差 1.3–1.5 ms、P95 5 ms；仍只在机动段可观测，匀速/悬停不可用。已加默认关闭的固件钩子 `include/ins_gnss_delay_auto.h` + `navigation_task.cpp` 的 `BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE`（默认 0，只记录，不改 age）；模块回归 `test_gnss_delay_auto.cpp` 在机动窗口估计 45 ms/真实 50 ms（5 ms 误差，8/8 通过），并含失锁拒绝/野值剔除；10/30/50 m/s × GNSS 0.5/1.5/3.0 m 的 3D 机动 MC（20 seeds/组）平均绝对误差约 1.8–2.0 ms、P95 5 ms；大规模拉偏 2×2000 例（5–60 m/s、5–120 ms、GNSS 0.3–5 m、三轴 ±0.5 m/s²、20% 3–10 s 失锁）合计 3998/4000≤20 ms（99.95%）、平均绝对 2.92 ms、P95 8.5 ms、最大 23 ms——失效集中在高延时+高噪声+高速度+大零偏+3D 机动角落；模块级野值注入 0–10%（50 seeds/组）全部有效、平均绝对 5.7–6.5 ms；新增 5 ms 网格抛物线亚网格插值后 2×2000 例通过 3999/4000、平均绝对 2.59 ms、P95 8.2–8.4 ms、最大 21.2 ms（插值前 2.92 ms/23.0 ms）；再扩到 4×2500 例（10k，含失锁/高噪声/三轴零偏）通过 9997/10000、平均绝对 2.54 ms、P95 8.29 ms、最大 22.38 ms，失败集中在高延时+高速度+大零偏+高噪声；固件参考轨迹已改为纯 IMU 航位推算 `ins_inertial_reference.h`；固件 GNSS 推送时间戳改为投递时刻，闭环估计 49.4–52.2 ms/真实 50 ms、位置改善 86–97%；`BFS_NAVIGATION_GNSS_DELAY_AUTO_ESTIMATE` 与 `APPLY` 均已默认改为 **1**（估计有效、sharpness≥0.003、估计新鲜≤5 s、与15 ms先验差≤100 ms、且 `!gnc_tel.vib_high` 才替换 age；否则回退 15 ms）；`pio run` 默认与开关=1 均 SUCCESS。新增在线振动估计 `include/ins_vibration_adaptive.h`（0.5 Hz 高通+0.5 s RMS，建议 Q=0.05+1.5·RMSg，clamp 0.05–0.5），默认只遥测、不写 EKF，`BFS_NAVIGATION_VIBRATION_ADAPTIVE=0`；宿主 3 seeds：0.5–2 g 单频/多频位置误差降 4–7%，无振动不退化，失锁持平；`vib` 测试已入 run_all，开关=1 构建 SUCCESS；但多维 MC（500 安全包线 + 300 含延时/失锁）显示有延时+失锁时最坏可从 0.27 m 恶化到 179 m，最终改为安全版 Policy C（Q 仅允许 0.15–0.25，延时 >50 ms/失锁/恢复/hAcc>2 m 回退 0.25）；10,000 例（4 shard×2500，含传感器误差/延时/失锁/振动）无 >5% 退化、最坏 1.0062×，但平均收益接近零。默认开关已改为 **1**，采用该保守策略。
- **EKF 在线零偏可靠性（2026-10-23；★数据更正）**：测试脚本参数 1-based 映射错位一位（`argv[21]=auto_delay`、`argv[22]=accel_scale`），旧 500/1000/2000/3000/5000 例 MC、40 km 发散、失锁硬重锚 A/B 全部撤回。正确参数：1000 例动态加计 0.0643/P95 0.2185、陀螺 0.0069 dps、位置均值 0.4993 m；另 500 例动态开加计门控加计 0.0637、位置均值 0.4818 m，与关闭一致；1000 例组合故障加计 0.1846/P95 0.4199、陀螺 0.0438 dps、位置均值 0.486 m、0 例 >1000 m；300 对同分布门控 A/B：加计均值均 0.1786、位置均值均 0.479 m、47 好/84 差，门控无收益，保持撤销；100 例阶跃加计 0.1505、位置 0.509 m；100 例温漂加计 0.1623、位置 0.457 m；200 对失锁冻结末端加计均 0.0310、位置均 42.433 m、41 好/4 差；200 对恢复硬重锚数值爆炸（~3e23 m），禁止；200 对软速度重锚（仅抬速度协方差）加计/位置完全一致、0 好/0 差，无收益；300 对加计 Q 0.010/0.020 对照均值 0.1769/0.1768、位置均值均 0.457 m、160 好/83 差，保持 0.010。已加 IMU 温度、温漂观测器、温度表采点（未补偿）；宽门控当前撤销，需重评。
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
bash test_host/run_all.sh  # 宿主机 g++ 平台无关回归（40+ 段：EKF/控制分配/调度/失败链路/IMU 驱动 mock 等——以脚本实际为准）
```

- 宿主机构建输出路径：用仓库根 `tools/with-ascii-bin.sh`（自动 subst 到 ASCII 盘符并清理；BIN_DIR 指到仓库 `tmp/`，不写系统 `%TEMP%`）。中文路径下 MinGW ld 会报 Invalid argument；`run_all.sh` 的 EKF 测试（`-DEKF_HOST_REGRESSION` + units convang 源）已封装在脚本内。
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
