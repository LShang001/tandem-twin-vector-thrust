# ICM-42688-P FIFO Packet 3 解析与驱动审计（2026-11-03）

> 目的：为 2 kHz 零丢样采样做地基。先按 DS-000347 Rev1.2 §6.1/§6.2 把
> FIFO 包格式固定下来，并记录现有 `ICM42688_FIFO` 驱动与手册的偏差。
> 本阶段不改飞控主路径，只提供纯解析器 + 宿主单测。

## 1. 手册定义（Packet 3）

`FIFO_CONFIG1 (0x5F)` 使能 accel+gyro 后，普通分辨率包为 16 字节：

| 字节 | 内容 |
|---|---|
| 0 | FIFO Header |
| 1..6 | Accel X/Y/Z（大端 int16） |
| 7..12 | Gyro X/Y/Z（大端 int16） |
| 13 | Temperature[7:0] |
| 14..15 | Timestamp[15:0] |

Header（0x5F 配置为 accel+gyro、非 20-bit）：
- bit7=1：FIFO 空；
- bit6=1：包含 accel；bit5=1：包含 gyro；
- bits3:2=10/11：包含 ODR timestamp；
- 合法高半字节 = `0x6`，即 `0x68..0x6F`。

## 2. 寄存器配置（后续集成用）

| 寄存器 | 值 | 说明 |
|---|---|---|
| `FIFO_CONFIG (0x16)` | `0x40` | bits7:6=01，Stream-to-FIFO |
| `FIFO_CONFIG1 (0x5F)` | `0x03` | bit0 accel + bit1 gyro；加温度可置 bit2 |
| `FIFO_COUNTH/L (0x2E/0x2F)` | 读 | FIFO 中有效字节数 |
| `FIFO_DATA (0x30)` | 读 | FIFO 数据口 |
| `SIGNAL_PATH_RESET (0x4B)` | bit2 | FIFO_FLUSH（异常时清） |

## 3. 现有 `ICM42688_FIFO` 驱动的问题

| 项 | 驱动实现 | 手册 | 结论 |
|---|---|---|---|
| 使能寄存器 | `FIFO_EN=0x23` | `FIFO_CONFIG1=0x5F` | ❌ 0x23 不是 FIFO 配置寄存器，可能写坏数据区 |
| 包长 | `6+6+2=14` | accel+gyro 包 16 B（含 header+temp+timestamp） | ❌ 少 2 B |
| 首字节 | 当 accel X 高字节 | 首字节是 header | ❌ 数据整体错位 |
| 温度 | 2 B | Packet 3 为 1 B（0x0D） | ❌ 偏移错 |
| 读取 | 每帧一次 SPI 事务 | 可 FIFO_DATA 连续读 | ⚠ 效率低 |

因此**不能直接启用现有 `ICM42688_FIFO`**，否则姿态/导航会读到错位数据；
必须先替换为按 header 解析的批量读取。

## 4. 已落地的地基

- 新解析器：`lib/ICM42688/src/icm42688_fifo_packet.h`
  - `isPacket3Header()`：校验 bit7/高半字节/timestamp 位；
  - `parsePacket3()`：按大端解析 accel/gyro/temp/timestamp，短包/null 拒绝；
  - 不依赖 Arduino，输出原始 LSB，换算仍用驱动 `_accelScale/_gyroScale`。
- 宿主单测：`test_host/test_icm42688_fifo_packet.cpp`
  - 合成 0x68 包，校验 6 轴、温度、时间戳、±8g/±2000dps 换算；
  - 拒绝 FIFO 空（0xE8）、仅 accel（0x48）、短包、null；
  - 0x6C（FSYNC timestamp）也接受。
- 已加入 `test_host/run_all.sh`（`icm42688fifo`）。

## 5. 下一步集成（未做）

1. 在 `ICM42688` 中增加正确的 `fifoEnableStream()` / `fifoReadPackets()`，
   读 `FIFO_COUNT` 后按 16 B 包批量读 `FIFO_DATA`；
2. `ImuDevice` 增加批量接口；`sensor_imu` 每个 FIFO 包逐个送入现有
   校准/滤波/增量缓冲，保留每样本 dt=1/ODR；
3. 默认关闭（宏 `BFS_IMU_FIFO`），A/B 验证 `imu_rate=1600`、
   `imu_delta_overflow=0`、姿态/导航不劣化后再考虑默认开启；
4. 异常处理：FIFO overflow/header 非法时 flush，并计数上报。

> 在完成 1–4 之前，飞控仍走现有 UI 轮询路径；本文件只证明包格式与解析正确。
