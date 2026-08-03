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

- 推力垂直投影用 **R13**（cos_tilt）；R33 是激光斜距补偿（激光沿 −z_b），勿混用。
- VTOL 悬停目标姿态合成 `q_hover ⊗ Rx(-Heading)`（q_hover = 绕 NED y 转 90° 机头朝天；悬停时航向轴 = 机体 x = 差速轴），勿用"绕机体 z 航向"的多旋翼式假设。
- 命名残留：`servo_deg_roll`/`TVC_ROLL_SERVO_PIN` 实际是前摆（偏航通道），待重命名（不影响功能）。
- `include/TVC_Control_Geometric.h` / `TVC_Control_3rdOrder_Poly.h` — 原版 TVC 几何模型，**已弃用**。控制分配现由 `TandemVec_ControlAllocation.h` 处理。

## 核心参数

- 单一定义源：`include/TandemVec_Config.h`（kT/kQ/wMax/I/a/b/dMax）和 `src/state_data.cpp`（限幅/质量/推力）
- PID增益：`src/state_data.cpp` §4.1 + `src/main.cpp` §4（限幅配置）
- 舵机行程：`SERVO_HALF_TRAVEL_DEG`（`flight_control.cpp` mix函数，默认45°，待标定）

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

## 项目背景

- 固件主入口：`src/main.cpp`，只保留 `setup()`、`loop()` 和必须位于单编译单元的初始化/调度胶水逻辑。
- 本地库位于 `lib/`；其中很多是随项目带入的第三方依赖，不要随意格式化、重构或替换。

## 硬件与串口

- 电路权威参考：`docs/电路拓扑参考.md`（2026-06-30 经 EDA 实测修正），包含全部 52 个 MCU 引脚映射、传感器型号、连接器定义和未使用硬件资源清单。修改任何引脚相关代码前必须先对照该文档。
- 串口分配总览、已验证硬件事实、未使用硬件资源表：**修改串口/引脚/传感器驱动代码或排查通信问题前**，读 `.agents/docs/hardware-reference.md`。

## 模块化架构

- 除明确的头文件工具函数外，模块间状态共享应通过 `state_data.h` 暴露的全局状态完成。
- EKF 内部始终保留 WGS84 绝对 LLA；`origin_*` 表示解锁时的起飞点；`relative_*` 与 `INS_GNSS_Packet.location_*` 为相对起飞点的 NED 位置；控制和遥测高度使用 `-relative_down`（向上为正）。DPS310 对外发布相对起飞点高度，进入 EKF 前必须通过 `origin_alt_m + baro_altitude` 恢复为与 GNSS 一致的绝对高度观测，不得直接把相对值传给 `MeasurementUpdateBaroAltitudeDetailed()`。
- `test_host/` 中的级联控制与闭环仿真按实际 `flight_control.cpp` 控制律、参数、单位和轴映射建立，用于宿主机验证固件行为；它们不是待接管固件输出的另一套生产控制链，不要据此提出架构统一或双链切换（来源：2026-07-29）。
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
