# Agent 工作说明

本文件是本仓库的项目级 `AGENTS.md`。Codex 会在会话启动时读取全局与项目级 `AGENTS.md`，并按从上层目录到当前目录的顺序合并指令；距离被编辑文件更近的 `AGENTS.md` 优先级更高。本文件适用于仓库根目录及其所有子目录，除非子目录内另有更具体的 `AGENTS.md`。

本仓库是基于 PlatformIO 的 STM32H743 **纵列双发矢量推力 VTOL 飞控固件**项目（从原共轴双桨 VTVL 移植重构）。处理本项目时，必须把它视为安全关键的嵌入式控制软件，而不是普通应用代码。

## 飞行器构型与控制分配

- **纵列双发**：前电机(CW)绕z_b摆动(δ_f, **偏航主控**)，尾电机(CCW)绕y_b摆动(δ_t, 俯仰主控)
- **差速反扭**：Δω绕推力轴(x_b)产生Mx——**水平巡航=滚转；垂直悬停（x_b竖直）=世界航向**
- **物理逆解**：α→I×α→M_cmd→allocateMoments(BTRUE)→δ_f/δ_t/Δω
- **执行器映射**：前摆δ_f→PA0(TVC_ROLL), 尾摆δ_t→PA1(TVC_PITCH), Δω→PA2/PA3(前后电机差速)
- **齿轮传动**：舵机30T/摆座40T = 1.333:1，PWM映射已含齿轮比

## 已知待办（轴约定遗留）

- **AUTO_POSITION/GUIDED 目标姿态构造**（`constructTiltTargetQuaternion` + `q_yaw_base = eulerToQuaternion(0,0,Heading)`）仍基于"悬停基准 q=[1,0,0,0、推力沿 -z_b]"的多旋翼式假设（航向基准绕机体 z）。本构型悬停基准是**机头朝天（q=绕 y 90°）**：航向基准应为绕机体 x（差速轴），合成公式需结构性改造（q_tilt 与航向的合成不是简单换轴）。影响：AUTO_POSITION/GUIDED 在悬停基准下的目标姿态可能偏差 90°。**修复前不要使用 AUTO_POSITION/GUIDED 悬停模式**；需专用数值验证（四元数合成 + 仿真闭环）后修正。
- **cos_tilt 已修复**（2026-08-02）：推力沿 +x_b → 垂直投影用 R13 = 2(qx·qz+qy·qw)，原 R33 在悬停时≈0 导致油门×2（审查 HIGH #1）。sensor_peripheral 的 R33 是激光斜距补偿（激光沿 -z_b），**保持正确**。
- **命名残留**：`servo_deg_roll`/`TVC_ROLL_SERVO_PIN` 实际是前摆（偏航通道），待重命名（不影响功能）。

## 核心参数

- 单一定义源：`include/TandemVec_Config.h`（kT/kQ/wMax/I/a/b/dMax）和 `src/state_data.cpp`（限幅/质量/推力）
- PID增益：`src/state_data.cpp` §4.1 + `src/main.cpp` §4（限幅配置）
- 舵机行程：`SERVO_HALF_TRAVEL_DEG`（`flight_control.cpp` mix函数，默认45°，待标定）

## 执行总原则

- 先读上下文再动手：修改前至少检查 `git status --short`，并阅读与任务直接相关的源码、`platformio.ini` 和本文件。
- 先诊断，后行动：不要看到表面症状就立刻改代码。先阅读相关代码、配置、日志、依赖关系和调用链，识别可能的根因。
- 小步、聚焦、可验证：优先做最小必要修改，不做大范围格式化、机械清理、顺手重构或无关依赖升级。
- 不要覆盖用户改动：尤其不要覆盖 `src/main.cpp`、`.vscode/`、`lib/` 或当前打开文件中的未提交内容。
- 保持专业质疑：需求模糊、有歧义、可能产生副作用或存在更好方案时，先说明风险并澄清，不要自行脑补。
- 能量化就不猜测：性能、资源、时序、数据规模、控制频率和数值边界问题应优先用测量、日志、计算或官方文档确认。
- 输出要区分状态：最终回复应说明已修改内容、验证结果、未验证项，以及是否影响飞控行为。
- 本项目内新增或修改的注释、README、说明文档、提交说明草稿和面向人的文字必须使用中文；命令、路径、文件名、API 名、协议名、库名和代码标识符按原文保留。
- 保留已有中文技术注释，除非注释明确错误且当前任务要求更新。
- **语言规则（最高优先级）**：所有思考过程（thinking）、内部推理、分析、决策、自我对话，必须使用中文。唯一允许保留英文的部分：代码片段、命令行、函数名、变量名、技术专有名词、路径、文件名、API 名、协议名、库名。

## 诊断、方案与失败处理

- 区分反应式与工程式处理：
  - 反应式：看到问题，直接修改，验证是否修好，然后结束。
  - 工程式：看到问题，先分析约束和影响，设计完整方案，再修改并验证所有相关维度。
- DEBUG 类任务第一次遇到时必须用工程式处理，因为问题深度未知，直接反应式修改容易越修越乱。
- 执行类任务在目标和改法都明确时可以用反应式处理，但验证必须按工程式覆盖相关维度。
- 同一问题连续修改 3 次仍未解决时，必须停下，重新按工程式分析根因、约束、影响面和验证方案。
- 对风险较高或范围不清的修改，动手前先说明修改点、修改原因、预期效果、可能副作用和验证方式。
- 用户已确认执行方案后，不要擅自偏离；如果后续发现重大问题，先说明情况，再讨论是否调整方案。
- 方案失败后优先追问“为什么失败、根因是什么”，不要无依据地连续换方案。应回溯假设、边界条件、环境差异和理解偏差。
- 修改后主动考虑可测试性、可维护性、幂等性和可重复性；复杂逻辑必要时添加简短中文注释。
- 重要功能修改、重构或架构变更后，应同步更新相关 README、注释、接口说明或本文件，保持代码与文档一致。

## 项目背景

- 固件主入口：`src/main.cpp`，只保留 `setup()`、`loop()` 和必须位于单编译单元的初始化/调度胶水逻辑。
- PlatformIO 环境：`TandemVec_FCS`
- 目标板卡：`weact_mini_h743vitx`
- MCU：`stm32h743vit6`
- 框架：Arduino on STM32
- 默认上传协议：`cmsis-dap`
- 主频配置：`480000000L`
- 本地库位于 `lib/`；其中很多是随项目带入的第三方依赖，不要随意格式化、重构或替换。

### 串口分配总览

安全规则禁止随意修改串口分配，以下是全系统串口角色，改动前必须对照：

| 串口 | 硬件标识 | 角色 |
|---|---|---|
| Serial1 | USART1 | ELRS 接收机（CRSF 输入，420 kbaud） |
| Serial2 | USART2 | 发动机控制器 / 数据转发（921600 baud） |
| Serial3 | USART3 | 黑匣子数据记录（1.5 Mbaud，高速） |
| Serial4 | UART4 | DETA100 模块 **或** UBX GNSS，二选一，由上电检测锁定 |
| Serial5 | UART5 | 上位机轨迹规划接口 + 制导指令接收（921600 baud） |
| Serial6 | USART6 | AnoCom / MAVLink 地面站通信，互斥使用（921600 baud） |
| Serial7 | UART7 | 光流传感器接口（921600 baud） |
| Serial8 | UART8 | USB Type-C 调试输出（921600 baud）；板载 CH343 USB 转串口芯片，数据线直连 PC 即可监视 |

### 硬件文档与电路验证

电路权威参考是 `docs/电路拓扑参考.md`（2026-06-30 经 EDA 实测修正），包含全部 52 个 MCU 引脚映射、传感器型号、连接器定义和未使用硬件资源清单。修改任何引脚相关代码前必须先对照该文档。

**已验证关键事实（不得随意修改）：**

| 事实 | 说明 |
|------|------|
| DPS310 使用 **I2C2** (PB11/PB10, 1MHz Fm+) | 非 SPI4。`电路拓扑参考.md` v1 错标为 SPI4，已于 2026-06-30 修正。SPI4 (PE11-14) 在 PCB 上已引出但未连接任何器件。 |
| DPS310 PE15 用作 **I2C 地址选择** (HIGH→0x77) | 原理图网络名 `BARO_INT` 有误导性，实际是 I2C 地址选择引脚 |
| ICM42688 在原理图中位号为 **R12** | 非电阻，是 LGA-14 IMU 芯片。`GYRO_INT` 连接 PC4，但固件用轮询未使用中断 |
| **全部 8 路 UART / 4 路 SPI / 2 路 I2C / 8 路 PWM 引脚** 已于 2026-06-30 EDA 实测交叉验证一致 |

**立创EDA Pro Bridge 可用：** 本机已部署 EDA API Bridge（端口 49620），可实时查询原理图连接关系。`curl -X POST http://localhost:49620/execute -H "Content-Type: application/json" -d '{"code":"..."}'` 即可执行 JS 查询 EDA。详细技巧见项目记忆 `eda-bridge-analysis-skills`。

**未使用硬件资源（扩展开发时优先查阅）：**

| 资源 | MCU 引脚 | 接口 | 详见文档 § |
|------|---------|------|----------|
| CAN 总线 (MCP2515+TJA1050) | PB12/PB13/PB14/PB15/PD10 | SPI2 | 12.1 |
| W25N01GV Flash (128MB) | PA15/PC10/PC11/PB2 | SPI3 | 12.2 |
| WS2812 RGB | PD15 | GPIO | 12.3 |
| 备用 PWM (S5/S6/S8) | PB0/PB1/PC9 | TIM | 12.4 |
| 空闲 SPI4 | PE11/PE12/PE13/PE14 | SPI | 12.5 |
| 未连接 GPIO (10个) | PE2/PE3/PA8/PD11-14/PD4/PB8-9 | — | 12.6 |
| 扩展连接器 (U12 I2C, U39 GPS等) | — | — | 12.8 |

> 💡 开发新功能时先对照 `docs/电路拓扑参考.md` §12 和上表，避免引脚冲突。CAN、Flash 是最高优先级的扩展方向。

## 模块化架构

源码按数据流分层拆分。除明确的头文件工具函数外，模块间状态共享应通过 `state_data.h` 暴露的全局状态完成。

| 文件                          | 职责                                            |
| ----------------------------- | ----------------------------------------------- |
| `src/main.cpp`                | 系统初始化、`setup()`、`loop()`、主循环调度     |
| `src/state_data.h/cpp`        | 全局变量声明/定义、枚举、结构体、跨模块共享状态 |
| `src/math_utils.h`            | 纯数学工具函数，`inline` header-only            |
| `src/task_scheduler.h/cpp`    | 2 kHz 定时器驱动的任务调度器                    |
| `src/sensor_imu.h/cpp`        | ICM42688 IMU、磁力计初始化与数据采集            |
| `src/sensor_peripheral.h/cpp` | DPS310 气压计、LQS48 光流、角度传感器           |
| `src/navigation_task.h/cpp`   | EKF 15 状态组合导航、静止检测、GNSS 动态权重、双矢量航向融合、ZUPT、垂直 KF；水平 KF（`handleHorizontalEstimation`）已注释停用，由 EKF 接管 |
| `src/flight_control.h/cpp`    | 控制律、PID、TVC、混控输出                      |
| `src/communication.h/cpp`     | CRSF 遥控、MAVLink/ANO 遥测、Serial2 发动机、Serial3 黑匣子、Serial5 上位机轨迹/制导、Serial8 调试遥测 |
| `src/deta100_types.h`         | DETA100 模块类型定义                            |

高度参考系约定：EKF 内部始终保留 WGS84 绝对 LLA；`origin_*` 表示解锁时的起飞点；
`relative_*` 与 `INS_GNSS_Packet.location_*` 为相对起飞点的 NED 位置；控制和遥测高度
使用 `-relative_down`（向上为正）。DPS310 对外发布相对起飞点高度，进入 EKF 前必须通过
`origin_alt_m + baro_altitude` 恢复为与 GNSS 一致的绝对高度观测，不得直接把相对值传给
`MeasurementUpdateBaroAltitudeDetailed()`。

### `include/` 平台无关算法目录

`test_host/` 中的级联控制与闭环仿真按实际 `flight_control.cpp` 控制律、参数、单位和轴映射建立，用于宿主机验证固件行为；它们不是待接管固件输出的另一套生产控制链，不要据此提出架构统一或双链切换（来源：2026-07-29）。

`include/` 下的头文件**不依赖 Arduino / STM32 HAL**，可用 `g++` 在宿主机上直接编译和回归测试，是算法与硬件平台解耦的专用区域：

| 文件 | 职责 |
|---|---|
| `ins_static_detector.h` | 静止检测器（状态机 + 置信度评分，供 EKF ZUPT 使用） |
| `ins_gnss_dynamic_weight.h` | GNSS 动态权重 R 矩阵计算（连续衰减，替代硬门限） |
| `ins_gnss_epoch_timing.h` | GNSS epoch 消费门控及 iTOW 到 MCU `micros()` 时间轴映射 |
| `ins_static_aid_profile.h` | 静止辅助配置（按置信度调度辅助强度） |

修改这三个文件后，可以也应该在宿主机上用 `g++` 单独验证，不必每次都走 `pio run` 完整编译。

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

在仓库根目录使用 PowerShell。

```powershell
pio run
```

运行宿主机回归测试：

```powershell
pio test -e test
```

只有在用户明确要求时才执行上传：

```powershell
pio run -t upload
```

需要串口监视器时使用：

```powershell
pio device monitor
```

如果 PATH 中找不到 `pio`，必须明确报告这一点，不能声称构建通过。除非用户要求修复环境，否则不要安装或修改全局工具链。本机历史上可用的 PlatformIO 路径为 `C:\Users\12631\.platformio\penv\Scripts\pio.exe`，但仍应以当前 `Get-Command pio` 和实际命令输出为准。

## 验证选择

- 文档或注释修改：至少检查 `git diff --check`，并确认内容与 `platformio.ini`、实际源码结构不冲突。
- 非行为性代码整理：运行 `pio run`；如果影响测试入口或公共工具函数，再运行 `pio test -e test`。
- 控制、导航、通信、安全相关修改：优先运行 `pio run` 和 `pio test -e test`；无法运行时必须说明原因。
- 不要声称“通过”没有实际执行的构建、测试或上板验证。

## 仓库卫生

- 修改前后检查 `git status --short`。
- 提交前检查 `git diff --check`。
- `.pio/`、`compile_commands.json`、本地 VS Code 索引和调试产物是生成产物，不应提交，除非用户明确要求更新相关生成文件。
- 根目录文档应与 `platformio.ini` 和实际固件结构保持同步。
- 不要把 `README.md`、`AGENTS.md` 或其他上下文文件改成参数流水账；只记录稳定、可复用、能指导后续代理工作的规则。

### 串口与进程管理

COM 口是独占资源。如果不正确关闭，后续所有串口操作都会因 `PermissionError(13, '拒绝访问')` 失败。必须严格遵守：

- **`pio device monitor` 必须加超时**：`timeout 8 pio device monitor -p COM10 -b 921600`，避免无限阻塞。不要用 Ctrl+C 硬杀，超时退出最安全。
- **PowerShell 串口脚本三原则**：
  1. `$port.Open()` 后必须用 `try/finally` 或在脚本末尾显式 `$port.Close()`
  2. 读写超时设短（`ReadTimeout = 500ms`），避免死等
  3. 脚本执行完毕立即删除自身（读数据脚本属于一次性使用）
- **串口数据抓取文件**（`ano_raw.bin`、`*.ps1`、`/tmp/*.txt`）必须在分析完毕后立即清理：
  ```powershell
  rm -f d:/pio_projects/vtvl_electricdualrotor_fcs/ano_raw.bin
  rm -f d:/pio_projects/vtvl_electricdualrotor_fcs/read_com10.ps1
  ```
  这些文件是运行时产物，不应堆积在仓库目录，更不能提交到 git。
- **统计脚本保留**：`analyze_ano.js` 是项目级分析工具，应保留在仓库根目录，不受上述清理规则约束。
- **残留进程清理**：如果串口突然全部拒绝访问，通常是上次会话的 `pio device monitor` 或 PowerShell 进程未退出。强制清理：
  ```powershell
  taskkill /f /im pio.exe
  taskkill /f /im powershell.exe
  ```
  或者在 PowerShell 中：
  ```powershell
  Get-Process powershell -ErrorAction SilentlyContinue | Stop-Process -Force
  ```

## 实机安全注意事项

在建议任何实机测试前，除非请求明确不涉及实时输出，必须提醒用户断开电机动力电源，或用其他方式让推进系统处于安全状态。

涉及飞控逻辑修改时，优先采用分阶段验证：

1. 编译检查。
2. 传感器初始化与串口遥测检查。
3. 在推进动力断开的条件下检查执行机构方向和限位。
4. 进行系留或其他约束条件下的低功率测试。
5. 只有在以上步骤均验证通过后，才考虑自由飞行测试。
