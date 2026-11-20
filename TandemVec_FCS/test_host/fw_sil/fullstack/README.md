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

## 4 阶段 P2：`main.cpp` 编译级接入（2026-11-20，进行中）

目标是把最重的源文件接进来：`src/main.cpp`（589 行）含完整 `setup()` 顺序、
**真实任务注册表**、`loop()` 主循环。

**已完成**：`src/main.cpp` 在宿主桩下 `-fsyntax-only` **零错误**通过。
验证命令（仓库根）：

```bash
bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_scheduler.sh
```

补桩过程中确认的三件事：

1. **`setup()` 的超时等待需要虚拟时钟**。DETA100 探测是
   `while (millis() - t0 < TIMEOUT) { ...; delay(1); }`，而宿主 `millis()` 由虚拟时钟给值。
   若 `delay()` 不推进时钟，该循环永不退出。因此桩层新增 `g_sil_delay_hook`：
   默认 `nullptr`（既有 SIL 行为完全不变），全链路模式安装后按 156.25 µs 步进并触发 tick。
   语义已冻结为测试断言：`delay(1000)` 推进 1000 ms、期间**不执行**任务、
   之后单次 `taskExecutor()` 只消费一帧（到期标志被合并）——与固件"中断置位、主循环执行"一致。
2. **宿主 `ubx.h` 桩会遮蔽真实 UBX 库**（`-Itest_host/fw_sil/stub` 优先于
   `lib/ublox-main/src`）。P4/P5 接真实遥测与 GNSS 配置时必须先换掉这层，
   否则 `Config/SetProtocol/Pump` 全是空实现，接出来的"链路"是假的。
3. **`WS2812Driver::beginDma()` 在宿主不存在**：真实头只在
   `ARDUINO_ARCH_STM32 && STM32H7xx && HAL_DMA_MODULE_ENABLED` 时声明该成员。
   用同名桩头覆盖（`stub/WS2812Driver.h`），只保证能编译，不代表灯效可验证。

**剩余（P2 收尾）**：链接期桩。`setup()` 要真正跑起来还差：`initImu()` 失败会进
`while(1)` 闪灯（`ImuDevice` 有 200+ 行纯虚接口，需寄存器级 SPI 模拟或专用桩后端）、
`initMagnetometer/initDPS310/ubxAutoConfig` 的返回值、约 20 个任务函数与
`CrsfSerial`/`StatusLED`/`N25N01GV` 等对象方法的空实现。

## 5 阶段 P3 入口：真实 IMU 驱动跑通 `begin()`（已完成，2026-11-20）

```bash
bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_imu.sh
```

链接真实 `src/imu_factory.cpp` + `lib/ICM42688/src/*.cpp`，宿主只替身 SPI 总线。

实测（ICM-42688-P 后端）：

| 项 | 值 |
|---|---|
| `begin()` 返回码 | **0** |
| 上报型号 | ICM-42688-P |
| 实际 accel ODR | **2000 Hz** |
| SPI 写寄存器次数 | 12 |
| 回读 `PWR_MGMT0`(0x4E) | **0x0F**（与驱动写入值一致） |

**这条比"假 IMU 后端"强在哪**：`ICM42688::writeRegister()` 每次写完都 `delay(10)`
再**回读校验**，`begin()` 内部还要走 reset → WHO_AM_I(0x75=0x47) → PWR_MGMT0 →
FSR → ODR → LPF 的完整流程。只有寄存器文件真的保存了写入值，这 12 次写才过得去。
用假后端永远测不到这段真实驱动逻辑。

**实现要点（踩过的坑）**：真实 SPI 由**片选**界定事务，宿主桩拿不到 CS 信号，
只能约定"每次事务以地址字节开头"。第一版没在读结束后回到等地址状态，
驱动第二次读把地址字节当数据取走，`whoAmI()` 读到错值、`begin()` 返回 -3。
修正后单字节读/写与批量读结束时都把状态机复位到"等地址"。

寄存器模拟**默认关闭**（`silEnableRegMode(false)`），既有 SIL 的 `transfer()`
仍原样回传入参，行为一字未变。

## 6 阶段 P3：真实传感器模块 `initImu()` 上链路（已完成，2026-11-20）

```bash
bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_sensor.sh
```

链接真实 `src/sensor_imu.cpp`（1139 行）+ `imu_factory` + ICM 驱动 + 磁芯片驱动 + Madgwick。

实测：

| 项 | 值 |
|---|---|
| `initImu()` 返回码 | **0** |
| IMU 全局指针 | 已装配 |
| 型号 / 实际 ODR | ICM-42688-P / **2000 Hz** |
| SPI 写寄存器次数 | 12 |

**踩到的死等**：`initImu()` 采集 100 个样本时 `while (IMU->dataReady() != 1)`，
判据是 INT_STATUS(0x2D) 的 bit3(DRDY)。寄存器文件若恒 0，这个循环永不退出 ——
现场表现是"编译成功、运行无任何输出、30 s 超时"，很容易被误判成崩溃。
解决办法是**让替身如实回答硬件状态**：预置 DRDY 位，并按 TEMP_DATA1(0x1F) 起
14 字节突发格式填一帧静止数据（RUB 系 Y 轴朝上，±16g 下 2048 LSB/g）。
这条经验同样适用于后续任何"驱动轮询硬件状态位"的场景。

### 6.1 增量构建（解决迭代成本）

首次全量编译 10+ 个源文件（含 Eigen 重度实例化）超过 5 分钟，每改一行桩都要等回来，
迭代不可接受。现在新增 `build_incremental.sh`：

- 每个源文件独立编译成 `.o`，带 `-MMD` 依赖文件
- 下次按"源文件或任一依赖头比 `.o` 新"判断，只重编受影响单元
- 实测改动一个测试文件：**重编 1 个、复用 12 个**，秒级完成

配合 `-O0`（接线验证不评估性能），把单次全量压到 1 分钟内。
**注意**：若将来做逐拍对拍，两条构建必须用同一优化级别，否则浮点结果不可比。

### 6.2 FIFO 数值闭环（P3 的验收核心）

`initImu()` 返回 0 只证明"能初始化"，不证明"数据链路的数值对"。为此给 SPI 桩加了
FIFO 数据口替身：`FIFO_COUNTH/L(0x2E/0x2F)` 如实返回队列字节数，`FIFO_DATA(0x30)`
按真实硬件的连续读语义从队列弹出。

实测（注入 1 帧 Packet3，16 字节大端）：

| 注入原始值 | 期望物理量 | 实测 |
|---|---|---|
| accel Y = 2048 LSB（±8g） | 0.5 g | **0.5000 g** |
| gyro X = 820 LSB（±2000dps） | 50.0488 dps | **50.0488 dps** |
| timestamp = 0x002A | 42 | **42** |

accel X/Z 为 0 也通过。这条链走的是**真实** `readFifoPackets()`：读 FIFO 计数 →
burst 读数据口 → `parsePacket3()` 校验 header → 按 `_accelScale/_gyroScale` 换算。

两个坑：

1. **`injectFifo()` 对 ICM 后端不生效**。它只把样本写进 `_acc`/`_gyr` 并置
   `_fifo_injected`，而 `readFifoPackets()` 完全不看这个标志——仍走真实 FIFO 寄存器。
   想验证 FIFO 链路必须模拟寄存器协议，不能依赖该接口。
2. **FIFO_DATA 的连续读不能按地址递增**。第一版沿用"地址+i"的普通寄存器路径，
   结果只有第一个字节从队列弹出、其余读到寄存器文件的 0，现象是 `readFifo` 已拿到
   帧数却解析出全 0。真实硬件数据口是连续前进的，桩必须同样处理。

标度口径也记一笔：陀螺 ±2000dps 驱动用 **32768/2000 = 16.384** LSB/dps，
不是手册常见的 16.4 取整；按 16.4 算期望值会差 0.05 dps 而误判成失败。

## 7 后续阶段

| 阶段 | 接入内容 | 判据 |
|---|---|---|
| P2 | `main.cpp` 链接期桩补齐 | 任务名/间隔/顺序与固件一致，个数可核对 |
| P3 | FIFO 取帧 → `imuSampleIsr` → `IMU_Packet` 数值 | 注入的寄存器值经真实换算后与 `IMU_Packet` 一致 |
| P4 | `communication.cpp`、`mavlink_bridge.cpp`、`ano_params.cpp`、`ano_vars.cpp` | 真实遥测帧字节可从宿主导出并解析回原值 |
| P5 | `ubx_config.cpp` | 替身接收机对 CFG 命令回 ACK，快路径/全量路径均可复现 |
| P6 | 一致性对拍 | 同一输入 trace 下与既有 SIL 逐拍比对，给出最大偏差 |

## 8 明确不在覆盖范围

- **磁标定 Flash 持久化**：真实 `lib/MagCal/MagCalStore.cpp` 直接操作 `FLASH->CR2`、
  等 `FLASH_SR_BSY`，宿主无此控制器，用替身（读恒失败、写恒成功）替代。
  "标定参数掉电保存"这条路径永远不能由宿主证明。
- **中断延迟、Cortex-M7 浮点时序、总线竞争**：宿主无法复刻，只能靠上板（L4）。
- **灯效**：`WS2812Driver` 是桩，不产生任何时序。

## 5 环境注意

- 仓库路径含中文，MinGW `ld` 写不了非 ASCII 输出路径。`BIN_DIR` 默认落系统临时目录
  （ASCII），或经 `tools/with-ascii-bin.sh` 把 `tmp/` 映射成盘符后指过去。
- 必须定义 `-DTANDEMVEC_FIRMWARE`：否则 `FlightCtrlParams.h` 走宿主只读副本分支，
  与 `state_data.cpp` 的可变实例声明冲突。
- 必须定义 `-DBFS_TASK_FRACTIONAL_INTERVALS`：与 `[env:unignc]` 保持一致，
  否则 5 ms 这类周期会被整数 tick 截断，调度结果与固件不同。