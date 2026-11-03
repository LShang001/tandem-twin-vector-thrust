# REF-177 · u-blox F10 SPG 6.00 接口说明（UBX-23002975 R02）

- 转换：MinerU 4.0.12（standard tier，181 页）
- 来源：https://content.u-blox.com/sites/default/files/documents/u-blox-F10-SPG-6.00_InterfaceDescription_UBX-23002975.pdf
- 固件：u-blox F10 SPG 6.00，**protocol version 40.00**（p1）。
- 与本项目关系：飞控 UBX 解析/配置的协议事实源；`src/ubx_config.cpp`、
  `lib/ublox-main/src/ubx.cpp` 的字段/键值依据。

## 必读结论（按重要性）

1. **F10 接口说明没有 `UBX-CFG-MSG`、`CFG-PRT`、`MON-HW` 章节**：CFG 类只有
   CFG-CFG/RST/VALDEL/VALGET/VALSET，MON 类用 MON-HW3（0x0a 0x37）。F10 的
   端口/消息速率用 `CFG-VALSET/VALGET/VALDEL` + `CFG-MSGOUT-*`（§3.10.5 p.56、
   §4.9.10 p.142–143）。**但本板实测 legacy CFG-MSG 仍回 ACK 并恢复 PVT 输出**，
   属未文档化的兼容层；长期路径必须是 VALSET，CFG-MSG 仅作老接收机/应急兼容。
2. **VALSET 载荷**：`version(1B)=0 | layers(1B, bit0 RAM/bit1 BBR/bit2 Flash) |
   reserved(2B) | 重复[key(4B LE) + value(1B)]`（§3.10.5 p.56）。
3. **本项目使用的键**：
   - `CFG-MSGOUT-UBX_NAV_PVT_UART1 = 0x20910007`（p.143）
   - `CFG-MSGOUT-UBX_NAV_EOE_UART1 = 0x20910160`（p.142）
   值为“每 N 个导航解输出一次”，1 = 每历元输出。
4. **NAV-PVT 固定 92 字节**（§3.14.11 p.102）；**NAV-EOE 固定 4 字节**
   （§3.14.5 p.97）。固件解析器按这两个长度做 epoch 配对。
5. **MON-VER**（§3.13.9 p.94）：boot screen/MON-VER 含 `PROTVER=40.00` 与
   `ANTSUPERV=AC SD OD PDoS SR`（天线监督能力位），可用于确认接收机身份与
   天线检测能力。
6. **8 字节 CFG-MSG 的端口顺序**（F9 兼容文档 §3.10.10，**F10 文档无此消息**）：
   `msgClass, msgID, rateDDC, rateUART1, rateUART2, rateUSB, rateSPI, reserved`。
   ★本项目 2026-11-03 实测 bug：旧实现把 rate 只写进 rateDDC，UART1/2 全 0，
   接收机 ACK 但串口不输出 PVT；已修为全端口同速率，并补 VALSET（文档路径）。
7. **天线/端口**：NEO-F10N 只有一个 UART；默认 38400 8N1；不支持硬件流控
   （集成手册 REF-176 §3.2 p.21）。

## 调试命令映射（固件）

| 目的 | DBG 命令 | 关键观测 |
| --- | --- | --- |
| 配置结果 | `ubxcfg status` | prt/rate/msg/val/nav5/gnss/sbas/verify、帧/ACK 数 |
| 解析健康 | `ubxstat` | rx_bytes/valid_msgs/cksum/oversize/pvt/eoe/qovf/last cls-id-len |
| 协议状态 | `gpsproto` | protocol/fix/sv/pdop/NMEA 句统计 |
| 接收机身份 | `ubxcfg monver`（建议新增） | PROTVER、ANTSUPERV |
| 天线 | `ubxcfg ant` | SHORT/OPEN |
| 总览 | `health` | GNSS pvt/eoe/fix/sv/age + EKF + IMU/气压 + 上行 |

## 归档说明

- `paper.md` 由 MinerU 生成；转换审计见同目录 `conversion-audit.md`。
- 引用表格/十六进制键值时以 PDF 页码为准；不要凭 Markdown 渲染猜列。
