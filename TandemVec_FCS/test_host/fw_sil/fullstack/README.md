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

### 6.3 迭代过程中的进程卫生（踩过）

`job_kill` **只断最外层**。本目标的进程树是 `bash → bash → fs_sensor` 三层，
杀掉 job 后中间层消失，`fs_sensor` 会被 reparent 到 PID 1 继续空转——
实测残留 4 个，最久的空转 1 小时 16 分仍在 R 状态吃 CPU，而且它们持有的是
**旧二进制**的内存映像，表现与当前源码完全对不上，排障时极易误判。

两条规范：

1. 任何会跑被测二进制的脚本都必须带超时（本目录的 `build_and_run_sensor.sh`
   已用 `timeout 30`，超时判定为 `[FAIL] 疑似外设重试循环不退出`）。
2. 收尾按命令行复查两侧进程，而不是只看 job 状态：

```bash
# WSL 侧（编译与仿真实际在这里跑）
wsl -e bash -c "ps -eo pid,ppid,etime,stat,args --no-headers \
  | grep -E 'fs_sensor|fs_sched|fs_imu|fullstack/build_and_run' | grep -v grep"
# Windows 侧
Get-Process -Name 'g++','cc1plus','bash','wsl' -ErrorAction SilentlyContinue
```

有残留就按 PID 从叶子往根 `kill -9`，再复查一次为空。

### 6.4 参数页运行期验证（P4 第一块）

```bash
bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_params.sh
```

链接真实 `src/ano_params.cpp` + `src/state_data.cpp` + `src/flight_control.cpp`
（参数写入后的 `applyFlightCtrlParams()` 同步链），实测：

| 项 | 结果 |
|---|---|
| 参数总数 | **169**（133 + 15 共轴 + 21 单涵道） |
| `att_roll.kp` 按名查找 | id=0，反查名字一致 |
| 默认值 | 2.800000，与 `kFlightCtrlParamsDefaults` 一致 |
| 写 3.25 后 | 读回 3.25，且 `kFlightCtrlParams` 实例真实改写 |
| NaN / +Inf / -Inf | 全部拒绝，原值未被污染 |
| 越界 ID(169) | 读写均拒绝 |
| bool 参数 | 写 0/1 读回 0/1 |

**同时得到一个可执行的状态判据**：写 `att_roll.kp = 1e6` 会被**接受** ——
当前实现只判 `isfinite`，没有 min/max 包线，即 GOV-006 的 FIX-02（P0）**尚未落地**。
这条不再靠读代码推断，测试会持续打印 `[NOTE] 当前无 min/max 包线`，等真正实现时
该行消失、断言改为「拒绝」即可。

### 6.5 遥测帧字节级验证（P4 核心，2026-11-20 完成）

```bash
bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_telemetry.sh
```

链接真实 `src/communication.cpp`（5291 行）+ `ubx_config.cpp` + `mavlink_bridge.cpp`
+ `sensor_imu.cpp` + `sensor_peripheral.cpp` + 各驱动库，调用真实
`sendPositionVelocityData()`，从串口桩捕获字节再解析回原值。

实测（机载链路 74 字节帧：`0xAA | 18×float32 小端 | 0x55`）：

| 字段 | 注入值 | 帧内解析值 |
|---|---|---|
| [2] 时间戳 | millis()=12345 ms | 12.345 s |
| [3][4][5] p_n/p_e/p_u | 12.5 / −3.25 / 7.75 | 完全一致 |
| [6][7][8] v_n/v_e/v_u | 1.25 / −1.5 / 0.5 | 完全一致 |
| [9][10][11] roll/pitch/heading | 0.25 / −0.5 / 1.5 | 完全一致 |
| [12][13][14] gx/gy/gz | 0.01 / −0.02 / 0.03 | 完全一致 |
| [15][16][17] acc_u/e/n | 0.11 / 0.22 / 0.33 | 完全一致 |

**为什么值得做**：帧字段偏移写错、字节序写反、单位漏换算，这些读源码都难发现
（5000 行里找一个 `memcpy` 的位移），但把真实构造出来的字节解析回来一比就立刻暴露。

**这条链的边界**：只证明"固件写出的字节符合它自己声明的布局"，**不证明上位机按同一
布局解析**。后者要在 GCS 侧另做一次解析对拍。字节序是小端（`memcpy` 直拷 STM32
内存），不是网络序。

**踩到的两个坑**：

1. **依赖链会一直延伸，不要逐个补**。`communication.cpp` 是 5000 行汇聚层，依次
   暴露 W25N01GV → magcal → ubx_config → CrsfSerial → `isGnssDataFreshForNav`。
   正确做法是**先一次性列出全部未定义符号**（`grep 'undefined reference'` 去重），
   再分两类处理：库里有实现就链接，语义明确的查询函数就做契约桩。
2. **`isGnssDataFreshForNav()` 用契约桩而不是链接 `navigation_task.cpp`**。
   后者会拖进整个 EKF 链；该函数语义单一（数据是否新鲜），桩返回 `false`
   表示"宿主没有真实 GNSS 接收"，上层因此走真实失败分支，不会伪造出假定位。

## 7 一致性判据（一键入口，2026-11-20 完成）

```bash
bash TandemVec_FCS/test_host/fw_sil/fullstack/run_consistency.sh
```

### 7.1 先说一件决定判据形状的架构事实

既有 SIL（`fw_sil_main.cpp:66-70`）把 plant 真值**直接写进 `AHRS_Packet`**，
绕过 IMU 驱动与 Madgwick；全链路目标走的是 SPI 寄存器 → 真实驱动 → `IMU_Packet`
→ Madgwick → `AHRS_Packet`。

**两者输入语义不同**，所以"整机姿态逐拍对拍"在架构上不成立 —— 把两个不同输入
喂出来的姿态相减，偏差只能说明输入不同，不能说明孪生准不准。判据因此按层拆开，
只对**同一层、同一输入**的部分做自动比对。

### 7.2 三层可自动判据

| 层 | 判据 | 实测结论 |
|---|---|---|
| **L1 输入边界** | 同一原始 LSB，真实驱动换算出的物理量与固件标度一致 | 2048 LSB→**0.5 g**、820 LSB→**50.0488 dps**、0x002A→**42** |
| **L2 调度时序** | 真实 TIM8 中断驱动的任务周期与相位精确 | 10 s/64000 tick → 2000/500/100 次，首次到期 **31/127/639** tick |
| **L3 源码同一性** | 两个构建链接同一份 `src/*.cpp`，无副本 | 共享 `flight_control.cpp`、`state_data.cpp`，**均无副本** |

L3 的意义：控制律与全局状态是**同一份源码**链接进两个构建，桩只替换硬件访问层。
脚本会扫描两个构建清单的交集，并检查除 `src/` 外是否存在同名 `.cpp` 副本
（有副本就说明有人复制了固件代码，同一性立刻不成立）。

### 7.3 L4 整机逐拍对拍（2026-11-23 完成）

```bash
bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_closedloop.sh
```

plant 姿态 → 机体比力/角速度 → **ICM-42688 原始寄存器值** → SPI 驱动 → `IMU_Packet`
→ Madgwick → 与 plant 真值逐拍比对。链路里 `sensor_imu.cpp`、`task_scheduler.cpp`、
`main.cpp` 一行未改。

实测（0.2 Hz 正弦姿态轨迹，20 s / 4000 拍）：

| 指标 | 值 |
|---|---|
| 冷启动收敛时间 | **9.90 s** |
| 收敛后 roll 峰值 / RMS | **0.301°** / 0.207° |
| 收敛后 pitch 峰值 / RMS | **1.980°** / 0.147° |
| 稳态 roll / pitch 残差 | **0.301°** / **0.047°** |

**这一层踩过三个坑，都值得记住**：

1. **寄存器布局**：数据寄存器 14 字节是「温度 2B + accel XYZ + gyro XYZ」，
   写入必须从 **0x1F** 起。曾把起始地址改成 0x1D 又保留 `pkt[2]=accel X`，
   两处偏移叠加等于没改，现象是静止时重力落到 body Y 轴、估计姿态出现绕 X
   约 −120° 的固定偏差。定位方法是写独立探针（见 `fs_axis_probe.cpp`）：
   写入可区分的 `(1000,2000,3000)` LSB，看驱动反推读出什么，一次锁定映射。
2. **轴映射是两段转换**：驱动输出芯片原始 RUB，`sensor_imu.cpp` 再做
   RUB→FRD（`bX=-sZ, bY=+sX, bZ=-sY`）。仿真注入必须写 RUB（即 FRD 的
   逆变换），写 FRD 会被再转一次。
3. **纯 IMU 的 yaw 不可观**：只给加速度和陀螺时 Madgwick 的航向没有锚点，
   估计姿态会整体绕重力轴漂。必须用 `magSensor.silInjectFrdSample()` 注入与
   姿态一致的磁场（该接口在 `#ifdef FW_SIL_REAL_NAV` 下，构建要带这个宏）。

**判据为什么要区分收敛期**：Madgwick 从单位四元数起步，前 10 s 是收敛过程，
峰值可达 80°+。把收敛段算进"全程误差"会把稳态 0.05° 的一致性淹没掉，
所以判据先要求"在仿真时长内收敛到 ±1°"，再对收敛后的拍统计。
收敛后再留 1 s 沉降期——收敛判定那一拍的误差本身就贴着阈值，从该拍开始统计
会让峰值恒等于阈值（实测 ±1° 阈值下 pitch 峰值恒为 0.95°，掩盖真实动态误差）。

### 7.4 频率扫描：这个孪生在多快动态下还准

```bash
bash TandemVec_FCS/test_host/fw_sil/fullstack/run_sweep.sh
```

同一链路、同一判据，只改激励频率（0.05–2 Hz）。实测：

| 激励频率 | roll 误差 | pitch 误差 |
|---|---|---|
| 0.05 Hz | 0.082° | 0.018° |
| 0.20 Hz | 0.301° | 0.047° |
| 0.50 Hz | 0.743° | 0.100° |
| 1.00 Hz | 1.464° | 0.193° |
| 2.00 Hz | 1.912° | 0.499° |

**误差随频率近似线性增长**，这是滤波相位滞后的特征，不是噪声或漂移。
按 roll 在 2 Hz 时 12° 幅值下 1.91° 的误差折算，等效滞后约 **12 ms**；
pitch 通道（0.7 倍频）折算约 **8 ms**。

工程含义：这个孪生可以直接替代实机验证**1 Hz 以下的姿态与姿态环行为**
（误差 < 1.5°）；要仿真 2 Hz 以上的快动态，姿态误差会到 2°，此时结论需要
带误差棒，不能当精确值用。

注意：慢激励下"峰值"与"稳态"列相等——最大误差不是瞬态冲击，而是持续的
相位滞后。这正是需要频率扫描而不是单点测量的原因。

### 7.5 传感器噪声：孪生的精度底噪约 0.9°

```bash
NOISE=1 bash TandemVec_FCS/test_host/fw_sil/fullstack/run_sweep.sh
```

在 7.4 的链路里加入加速度计/陀螺噪声与三轴零偏后重跑同一组频率：

| 激励频率 | roll 理想 | roll 带噪声 | pitch 理想 | pitch 带噪声 |
|---|---|---|---|---|
| 0.05 Hz | 0.082° | **0.993°** | 0.018° | **0.829°** |
| 0.20 Hz | 0.301° | **1.090°** | 0.047° | 0.823° |
| 0.50 Hz | 0.743° | 1.495° | 0.100° | 0.870° |
| 1.00 Hz | 1.464° | 2.167° | 0.193° | 0.880° |
| 2.00 Hz | 1.912° | 2.343° | 0.499° | 0.903° |

误差模型（**假设值，非本机标定**）：acc σ=0.004 g、零偏 (0.010,−0.012,0.008) g；
gyr σ=0.12 dps、零偏 (0.15,−0.12,0.10) dps。量级取自 ICM-42688-P 手册噪声密度
按 2 kHz 带宽折算，零偏取常温典型值。

**两条结论**：

1. **存在约 0.8–0.9° 的精度底噪**，它不随激励频率降低而减少。0.05 Hz 准静态下
   理想误差只有 0.08°，带噪声是 0.99°——**噪声主导低频，相位滞后主导高频**。
2. 因此这个孪生**不能替代实机做亚度级精度验证**；1 Hz 以下做功能级/趋势级验证
   仍然有效（误差 ~1–2°），2 Hz 以上要带误差棒。

### 7.6 整机闭环：能成立，但不稳定（`--hover`）

```bash
bash TandemVec_FCS/test_host/fw_sil/fullstack/build_and_run_closedloop.sh --hover --dur 50
```

从"姿态开环对拍"推到"控制律闭环"。**成功时**定高稳定在 10.00 m（稳态区间
[9.997, 9.999]，悬停垂直速度 −0.00 m/s，`estimated_height` 与真实高度逐拍一致，
`ekf_converged=1`）。链路：plant → 真实 ICM-42688 驱动 → Madgwick → 真实 EKF →
真实控制律 → plant，`navigation_task.cpp` / `flight_control.cpp` / `sensor_imu.cpp`
一行未改。

**但不可靠**：6 次连跑是 5 成功 1 失败，失败时一路下坠（`alt_mean` 出现过
−327 m、−1673 m、−2132 m、−6174 m，以及 NaN）。

**发散机制已定位**（成功与失败在 28 s 之前完全一致，分叉只发生在"接近目标"那一刻）：

| 时刻 | 成功案例 | 失败案例 |
|---|---|---|
| t=30 | `alt=9.69` `v_d=−0.38` `motor=48.8%` | `alt=9.74` `v_d=−0.07` `motor=48.0%` |
| t=32 | `alt=9.94` `v_d=−0.02`（稳住） | `alt=6.92` `est_h=9.21` `v_d=+4.20`（掉下去） |
| t=36 | 稳定 10.00 m | `alt=−98` 彻底失控 |

两例在 t=30 都把油门降到 **48–49%**，低于悬停所需的约 50%。成功案例那一刻
`v_d=−0.38`，还有上升动量，惯性把高度带过 9.94 m 并稳住；失败案例 `v_d=−0.07`
已近零，没有缓冲动量，推力略小于重力就直接下坠。下坠后定高环响应慢
（t=36 才回到 57%），来不及拉回。失败时 EKF 高度还比真实滞后两米多
（`est_h=9.21` 对 `alt=6.92`）。

**结论**：这不是测试的错，是定高环在目标附近**稳定裕度不足**——油门会降到略低于
悬停值，能否稳住取决于接近目标时的残余速度。对实机调参有直接参考价值。

**五个根因**（前四个都"看起来像 EKF 坏了、其实不是"）：

| # | 根因 | 症状 | 性质 |
|---|---|---|---|
| 1 | 测试预设 `nav_system_initialized = true` | 初始化分支永不进入，四元数全零 → 判自由落体，`est_h` −50 → −708 m | 测试错 |
| 2 | 漏 `#ifdef BFS_EKF_BARO_ALTITUDE_UPDATE` | 气压融合整段没编进去 | 构建错 |
| 3 | 无 GNSS 时 `relative_down` 被置 0 | `estimated_height` 恒 0 | 安全设计 |
| 4 | 只发 NAV-PVT、不发 NAV-EOE | `fix=3` 但 epoch 从不入队，`PopEpoch` 恒 false | 协议理解错 |
| 5 | 先设目标高度、后切模式 | 首次进 AUTO_ALTITUDE 时 `target_altitude = 当前高度` 覆盖回 0 | 时序错 |

**三个关键实现点**：

1. **NAV-EOE 是 epoch 入队的开关**。ubx 解析器把 NAV-PVT 组装成"历元"，但要收到
   NAV-EOE（`0x01 0x61`）才认为历元完整并**入队**。只发 PVT 时帧被解析、`fix` 都
   变 3 了，可 epoch 队列始终空——`fix=3` 与 `fake_origin=1` 并存就是铁证。
2. **注入节拍要按消费节拍设计**。`has_gnss_epoch = ubx.PopEpoch()` 每拍调一次（200 Hz），
   而 EKF 初始化实测在 **0.18 s** 就发生。全程 10 Hz 注入时初始化那拍取不到数据的
   概率约 1/20；全程 200 Hz 又把 EKF 打崩（`pos σ` 涨到 2 万）。最终只在
   **0–0.6 s** 初始化窗口内逐拍注入，其余时间 10 Hz。
3. **定高指令必须晚于模式激活**。`flight_control.cpp:615` 在首次进入 `AUTO_ALTITUDE`
   时执行 `target_altitude = current_altitude_for_control`。测试若早于这一刻设 10 m，
   会被覆盖回 0。

**同时修掉一个构建隐患**：`build_incremental.sh` 的重编判据原先只哈希源文件与依赖头，
不含 `CXXFLAGS`，导致多个共用 `OBJ_DIR` 但宏不同的构建会复用错误 `.o`。已把编译
选项纳入组合哈希。

**边界**：GNSS 仍是构造的 NAV-PVT 帧（位置由 plant 真值换算）；气压已走**真实 DPS310
驱动**——测试按高度算标准大气压、反推 raw 写进 FIFO 数据寄存器，由驱动自己做
getFIFOvalue → calcPressure → calculateAltitude → 滤波 → `baro_altitude`。

打通这条链路踩了三个坑，全是宿主侧模拟写错：①预置到了 `Wire`，而驱动用的是独立
实例 `IIC2`；②样本类型位反了（`getFIFOvalue` 返回 `buffer[2]&0x01`，**1 才是压力**、
0 是温度，写反会让每个样本都被当成温度，驱动返回 `DPS__FAIL_UNFINISHED`）；
③`FIFO_EMPTY`（0x0B bit0）初始值必须是 0 才表示有样本。

排查捷径：直接调 `Dps310Sensor.getLatestResults()` 取返回码，按 dps_config.h 对照
（0=成功、-2=初始化失败、-4=未完成、-5=溢出），一个数字就能把范围砍到具体那一层。

**加噪声后的鲁棒性**（`--hover --noise`，含 IMU 噪声/零偏 + GNSS 位置 1.5 m /
垂直 3 m / 速度 0.1 m/s 误差）：

| 工况 | 稳态高度 | 区间 |
|---|---|---|
| 理想传感器 | 10.00 m | [9.997, 9.999]（±0.2 cm） |
| 带噪声 | 10.00 m | [9.98, 10.02]（±2 cm） |

噪声下闭环仍稳定，波动放大约一个数量级但未发散。
### 7.7 明确不可比的层

- **L5 硬件时序**：中断延迟、Cortex-M7 浮点时序、总线竞争、D-Cache 一致性，
  宿主一律不成立，只能上板（VER-009 的 L4 层）。
- **L4 自身的边界**：本层用理想传感器——无噪声、无零偏、无总线延迟，度量的是
  「驱动轴映射 + 姿态滤波」的纯机理传递误差，不含实机噪声传播。收敛时间 9.9 s
  是冷启动过程，实机有初始对准，不代表上电收敛指标。

## 8 后续阶段

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
