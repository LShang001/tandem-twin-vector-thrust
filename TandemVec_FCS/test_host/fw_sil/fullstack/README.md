# 全链路数字孪生（`test_host/fw_sil/fullstack`）

> 目标：让仿真跑的是**固件同一份源码**，而不是按源码重写的等效逻辑。
> 与既有 SIL（`test_host/fw_sil/*`）的区别是覆盖面，不是替代关系：
> 既有 SIL 只链接 14 个 `src/*.cpp` 中的 3 个，本目录逐步把剩下的接进来。

## 1 「完全一致性」的可验收定义

"数字孪生"不能靠形容词，按四条可判据验收：

| 判据 | 含义 | 验证方式 |
|---|---|---|
| 源码同一 | 编译的是 `src/` 下原文件，不复制、不改分支 | 构建清单只列 `src/*.cpp`，无 `*_sil.cpp` 副本 |
| 时序同一 | 调度 tick、任务间隔、相位与固件相同 | 执行次数与首次到期拍精确匹配（见 §3） |
| 输入同一 | 传感器/链路字节从同一入口进入 | 走真实 `ubx` 解析器与 `sensor_*` 驱动函数 |
| 逐拍可对拍 | 相同输入下两套构建状态一致 | 同一 trace 喂两条构建，比对逐拍输出 |

**明确不代表什么**：不代表中断延迟、Cortex-M7 浮点时序、缓存与总线竞争相同。
这些在宿主上无法复刻，只能靠上板（L4）覆盖。任何"孪生通过即可飞"的推论都不成立。

## 2 分层：哪些是真的，哪些是桩

| 层 | 内容 | 状态 |
|---|---|---|
| 算法与业务 | `task_scheduler.cpp`、`state_data.cpp`、`flight_control.cpp`、`navigation_task.cpp`、`communication.cpp`、`sensor_imu.cpp`、`sensor_peripheral.cpp`、`ubx_config.cpp`、`mavlink_bridge.cpp`、`ano_params.cpp`、`ano_vars.cpp`、`can_bus.cpp`、`imu_factory.cpp`、`main.cpp` | 逐步接入（§4） |
| 框架 | Arduino `millis/micros/delay/Print/Serial/HardwareTimer` | `stub/Arduino.h` 替身 |
| 外设寄存器 | TIM8、IWDG、SPI、I2C、GPIO | 替身，只记录调用不产生时序 |
| 物理对象 | 机体 6DOF、螺旋桨、舵机 | `fw_sil_env_main.cpp` 的 plant |

桩的边界原则：**只桩"最底层硬件访问"，不桩任何决策逻辑**。例如
`initImu()` 的返回值是硬件事实，可以桩；但它返回后 `setup()` 里"失败就进死循环闪灯"
这条策略必须跑真实代码。

## 3 阶段 P1：真实任务调度器（已完成，2026-11-20）

```bash
bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_scheduler.sh
```

链接 `src/task_scheduler.cpp` + `src/state_data.cpp`，宿主替身只提供
`HardwareTimer`（记录 6400 Hz 配置与溢出回调，`silTick()` 等价一次 TIM8 中断）。

实测（10 s 仿真，64000 tick）：

| 任务间隔 | 期望 tick | 执行次数 | 首次到期拍（0 基） |
|---|---|---|---|
| 5 ms (200 Hz) | 32 | **2000** | 31 |
| 20 ms (50 Hz) | 128 | **500** | 127 |
| 100 ms (10 Hz) | 640 | **100** | 639 |

次数与相位全部精确匹配，`setupTimer()` 的 overflow 读回 6400 Hz 且为 HERTZ_FORMAT。
这证明：**固件的任务调度、相位、到期判定、统计口径能在宿主上原样运行**，
被替换的只有"谁触发中断"。

## 4 后续阶段

| 阶段 | 接入内容 | 判据 |
|---|---|---|
| P2 | `main.cpp` 的 `setup()` 流程与**真实任务注册表** | 任务名/间隔/顺序与固件一致，个数可核对 |
| P3 | `sensor_imu.cpp`、`sensor_peripheral.cpp` | 仿真数据走真实驱动解析路径，`IMU_Packet` 由驱动产出 |
| P4 | `communication.cpp`、`mavlink_bridge.cpp`、`ano_params.cpp`、`ano_vars.cpp` | 真实遥测帧字节可从宿主导出并解析回原值 |
| P5 | `ubx_config.cpp` | 替身接收机对 CFG 命令回 ACK，快路径/全量路径均可复现 |
| P6 | 一致性对拍 | 同一输入 trace 下与既有 SIL 逐拍比对，给出最大偏差 |

## 5 环境注意

- 仓库路径含中文，MinGW `ld` 写不了非 ASCII 输出路径。`BIN_DIR` 默认落系统临时目录
  （ASCII），或经 `tools/with-ascii-bin.sh` 把 `tmp/` 映射成盘符后指过去。
- 必须定义 `-DTANDEMVEC_FIRMWARE`：否则 `FlightCtrlParams.h` 走宿主只读副本分支，
  与 `state_data.cpp` 的可变实例声明冲突。
- 必须定义 `-DBFS_TASK_FRACTIONAL_INTERVALS`：与 `[env:unignc]` 保持一致，
  否则 5 ms 这类周期会被整数 tick 截断，调度结果与固件不同。