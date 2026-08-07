# 硬件参考 — 串口分配与已验证电路事实

> 从 AGENTS.md 外移（2026-08 精简）。**何时读**：修改任何串口、引脚、传感器驱动相关代码之前，或排查通信/外设问题时。

## 串口分配总览（安全规则：改动前必须对照）

| 串口 | 硬件标识 | 角色 |
|---|---|---|
| Serial1 | USART1 | ELRS 接收机（CRSF 输入，420 kbaud） |
| Serial2 | USART2 | 发动机控制器 / 数据转发（921600 baud） |
| Serial3 | USART3 | 黑匣子数据记录（1.5 Mbaud，高速） |
| Serial4 | UART4 | DETA100 模块 **或** UBX GNSS，二选一，由上电检测锁定 |
| Serial5 | UART5 | 上位机轨迹规划接口 + 制导指令接收（921600 baud） |
| Serial6 | USART6 | AnoCom / MAVLink 地面站通信，互斥使用（**波特率见 `state_data.h` 的 `SERIAL6_BAUDRATE`**，2026-08-07 对齐 2.4G 数传为 2M，随数传模块可调；调试模式入口 "DBG\n"，见 AGENTS.md §调试工具链） |
| Serial7 | UART7 | 光流传感器接口（921600 baud） |
| Serial8 | UART8 | USB Type-C 调试输出（921600 baud）；板载 CH343 USB 转串口芯片，数据线直连 PC 即可监视 |

## 已验证关键事实（不得随意修改）

| 事实 | 说明 |
|------|------|
| DPS310 使用 **I2C2** (PB11/PB10, 1MHz Fm+) | 非 SPI4。`电路拓扑参考.md` v1 错标为 SPI4，已于 2026-06-30 修正。SPI4 (PE11-14) 在 PCB 上已引出但未连接任何器件。 |
| DPS310 PE15 用作 **I2C 地址选择** (HIGH→0x77) | 原理图网络名 `BARO_INT` 有误导性，实际是 I2C 地址选择引脚 |
| ICM42688 在原理图中位号为 **R12** | 非电阻，是 LGA-14 IMU 芯片。`GYRO_INT` 连接 PC4，但固件用轮询未使用中断 |
| **全部 8 路 UART / 4 路 SPI / 2 路 I2C / 8 路 PWM 引脚** 已于 2026-06-30 EDA 实测交叉验证一致 | — |

## 未使用硬件资源（扩展开发时优先查阅）

| 资源 | MCU 引脚 | 接口 | 详见文档 § |
|------|---------|------|----------|
| CAN 总线 (MCP2515+TJA1050) | PB12/PB13/PB14/PB15/PD10 | SPI2 | 12.1 |
| W25N01GV Flash (128MB) | PA15/PC10/PC11/PB2 | SPI3 | 12.2 |
| 备用 PWM (S5/S6/S8) | PB0/PB1/PC9 | TIM | 12.4 |
| 空闲 SPI4 | PE11/PE12/PE13/PE14 | SPI | 12.5 |
| 未连接 GPIO (10个) | PE2/PE3/PA8/PD11-14/PD4/PB8-9 | — | 12.6 |
| 扩展连接器 (U12 I2C, U39 GPS等) | — | — | 12.8 |

> **已启用（2026-08-08）**：WS2812 RGB (PD15) 已由 `lib/WS2812Driver/` 驱动（见 12.3），不再属于未使用资源。
> 💡 开发新功能时先对照 `docs/电路拓扑参考.md` §12 和上表，避免引脚冲突。CAN、Flash 是最高优先级的扩展方向。

## 立创 EDA Pro Bridge

本机已部署 EDA API Bridge（端口 49620），可实时查询原理图连接关系：

```bash
curl -X POST http://localhost:49620/execute -H "Content-Type: application/json" -d '{"code":"..."}'
```

详细技巧见项目记忆 `eda-bridge-analysis-skills`。
