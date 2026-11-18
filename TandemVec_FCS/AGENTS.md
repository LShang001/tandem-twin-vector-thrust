# AGENTS.md — TandemVec_FCS 飞控固件

本页补充 [全仓规则](../AGENTS.md)。`docs/` 指 [固件文档](docs/)，`../docs/` 指 [仓库文档总索引](../docs/README.md) 所在目录。不要混用两处 `memory/`。

## 当前实机机型与构建

- 烧录前必须确认目标机型、IMU、协议和用途并显式选环境。三个基础机型及 `health` 标识：`TandemVec_FCS` → `TandemVec`，`CoaxialVTVL` → `CoaxialVTVL`，`SingleEdf4Vane` → `SingleEdf4Vane`。不要把 `default_envs` 当作实机身份。
- 环境继承与宏以 [platformio.ini](platformio.ini) 为准，用 `pio project config` 核对展开后的标志，不从 `.pio/build/` 反推。新增组合从 `env:unignc` 继承，不复制基础宏；`P1` 是采样架构，不是环境名。换环境不需要清理其他环境的构建产物。
- 实飞构建和诊断构建不可混用；核对诊断帧、串口选择及采样率时，先读 [调试工具链 §上板验收口径](../docs/registers/固件调试工具链-2026-11-01.md#上板验收口径)。

## 安全关键修改规则

- 未经用户明确要求，不改引脚、串口、波特率、PWM、中位/行程、任务频率/调度、增益/混控/限幅、轴向、解锁、故障保护、超时或降级。不得为了编过或跑绿移除保护。
- 未经明确要求不上传或上板。实机测试前提醒断开动力；按编译 → 初始化/遥测 → 断动力核方向与限位 → 系留低功率 → 自由飞的顺序验证。烧录成功不等于运行成功；检查启动日志、机型身份和对应参数读回，不把参数数目当作唯一版本证明。
- 改控制、估计、混控、解锁或协议，汇报受影响模式、故障路径、已完成验证和上板风险。不要声称未执行的验证通过。
- 串口独占，监视命令必须设超时，脚本确保异常也关闭串口。端口被占用先确认所属进程；不杀用户或其他会话的监视工具。DBG 打印会扰动实时调度，不以持续 DBG 输出定量验收时序。

## 坐标系与执行器

- 轴、符号、姿态和投影正本：[MOD-002 §0](../docs/04-数学建模/MOD-002-坐标系与符号约定.md) 和 [FrameConventions.h](include/FrameConventions.h)。任何轴/通道注释标明坐标系，不只写 x/y/z。固件存档系悬停 `z_b` 指地，垂直投影取 `|R33|`，不用仿真模型系的 `R13`；四元数 `{w,x,y,z}`，悬停静置 roll/pitch 约零，不加悬停基态补偿。
- 纵列悬停：上摆主控滚转、下摆主控俯仰、差速主控偏航。先查轴映射再查控制律符号；几何变换不能代替力矩链符号核对。改分配先看 [flight_control.cpp](src/flight_control.cpp) `mix_and_output_commands()` 和 [TandemVec_ControlAllocation.h](include/TandemVec_ControlAllocation.h)。共轴或单涵道不可套用纵列的执行器映射。
- **IMU 轴映射**：`bX=-sZ`、`bY=+sX`、`bZ=-sY`。悬停构型静止应见 `acc≈(0,0,-1)g`、roll/pitch 约零。除非明确要求且有实机证据，不改这组映射。
- 通用速率环及分配的单位、限幅边界见 [CTL-006](../docs/05-控制与分配/CTL-006-控制机型解耦契约与分配器.md)。机型物理权限修改走对应机型参数，不手改会被 `applyFlightCtrlParams()` 覆盖的 `out_max`。

## 硬件与串口

- 改引脚、串口或传感器驱动，先对照 [硬件参考](.agents/docs/hardware-reference.md)、[电路拓扑](docs/电路拓扑参考.md) 和 [器件资产索引](../docs/08-参考资料/器件手册与驱动资产索引.md)。硬件参考包含遥控通道语义与共轴 CH8 反相规则。未解锁仍可能有手动舵输出，不能用上锁替代断动力。
- `DETA100_module.h` 只由 `main.cpp` 包含；DETA100/UBX 的数据源在 `setup()` 锁定，DETA100 任务先于状态估计；GNSS 串口重开或切速只走 `ubxGnssSerialBegin()`，不绕过 FIFO/IRQ 恢复。MAVLink 和 AnoCom 在 Serial6 互斥；Serial8 也只允许一个上行协议消费者。
- 不改 `lib/MAVLink` 供应商树。`QuaternionMath.h` / `GeoDisplacement.h` 工具函数保持 `inline`；`Vector3` 使用 `VECTOR3_TYPE_GUARD`。新增 AnoVars 在 `kAnoVars` 注册，MAVLink 截短名仍须唯一。
- 更改导航宏、查无 GNSS 退化或位置重锚边界，先读 [上板标定与验证清单](../docs/04-数学建模/代码理论核查/上板标定与验证清单-2026-11-01.md) 和 [CTL-005](../docs/05-控制与分配/CTL-005-无磁航向与运动EKF.md)。断电后运行时控制参数不视为已保存；磁标定 Flash 保留区不可供其他持久化工具复用。

## IMU 数据链与延迟排查

- [imu_odr_config.h](src/imu_odr_config.h) 的 `kOdrHz` 是芯片出帧率正本，`kIsrPollHz` 是 MCU 取数轮询频率。两者不得混称，不在别处硬编码。两个 IMU 后端均按明确型号和档位验收；45686 不允许取消 `BFS_IMU_ISR_RING` 退回低频轮询。
- 要调 ODR、ISR 或 FIFO，先读 [ISR 选档](../docs/memory/2026-11-12-IMU-ISR轮询频率选档.md) 与 [FIFO 延迟记录](../docs/memory/2026-11-11-IMU延迟根因-FIFO速率失配.md)。排查时分别看取数、有效帧、消费、数据年龄和累计丢弃；单看队列空或计数相等不足以排除硬件 FIFO 积压。

## 构建与验证

下列命令从 `TandemVec_FCS/` 运行。验证具体环境时为 `pio run` 显式加 `-e` 和目标环境名。

```bash
pio run
bash test_host/run_all.sh
bash test_host/fw_sil/run_acceptance.sh
bash test_host/fw_sil/build_and_run_failsafe.sh
```

- 回归用 `test_host/`，不用 `pio test -e test`；`env:test` 是板载自检，不是宿主回归。Windows 宿主构建需要 ASCII 输出路径时使用 [仓库包装器](../tools/with-ascii-bin.sh)，不留常驻映射。共轴 SIL 采用 rigorous plant，验收边界见 [VER-009](../docs/07-验证与确认/VER-009-SIL验收脚本判据表-2026-11-07.md)。
- 导航、控制真值用 WGS-84 正常重力，不把量程换算或未初始化的回退常量当真值。控制、导航、通信或安全改动至少构建并跑相关宿主段；默认值改动按根规则跑完整宿主回归。`.pio/`、编译数据库和本机索引不入库。
- 排查上板、串口及遥测读数，查 [固件调试工具链](../docs/registers/固件调试工具链-2026-11-01.md)；查已归档实测，读 [VER-010](../docs/07-验证与确认/VER-010-实机烧录验证记录-2026-11-07.md)。分析延迟、内存热点或 DTCM 时读 [实时性能基线](../docs/04-数学建模/代码理论核查/飞控实时性能基线-2026-11-03.md)。
