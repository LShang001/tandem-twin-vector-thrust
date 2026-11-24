# ELRS 接收机 MAVLink 模式接入 RC（波特率与协议切换的两个硬约束）

**触发条件**：给飞控接 ELRS 的 MAVLink RC（`RC_CHANNELS_OVERRIDE`）；或串口"有字节但 CRSF/MAVLink 两种解析器都解不出帧"；或准备把 RC 输入源从 CRSF 切到 MAVLink。

## 1. ELRS 3.x 不自动跟随，改完必须给接收机重新上电

官方文档写的「TX/RX 固件 ≥ 3.5.0」前提之外，还有一条**实践约束**：4.x 由 TX 的 Link Mode 自动下发协议给 RX，**3.x 必须在 Lua 的 `Other Devices` 里单独把接收机的 Serial Protocol 设成 MAVLink，且设置后要重新上电才生效**。

本次实测踩到：Lua 里改完、不重新上电，飞控侧无论怎么折腾都收不到合法帧；重新上电后立刻 `link=up`。

## 2. 波特率由协议决定，MAVLink 是硬编码 460800

ELRS 源码 `src/src/rx_main.cpp` `setupSerial()`（本地快照 `docs/reference/elrs-official`）：

```
PROTOCOL_CRSF    → serialBaud = firmwareOptions.uart_baud   // Lua 可配，出厂 420000
PROTOCOL_MAVLINK → serialBaud = 460800                      // 硬编码，与 uart_baud 无关
```

所以切协议**必须同步切飞控侧波特率**：CRSF 420000、MAVLink 460800。漏改的表现不是"没数据"，而是**有数据、但两种解析器都解不出帧**。

固件侧已实现：`rcInputSetProto()` 里按目标协议自动设波特率，编译期 `BFS_RC_PROTO_MAVLINK=1` 时上电初值也是 460800（`s_rc_baud` 初值随 `s_rc_proto` 定）。

## 3. 诊断判据：先分"没数据/有数据不是这协议/收了被过滤"

DBG `rcproto` 一次给全：

| 读数 | 含义 |
|---|---|
| `bytes` 不涨 | 串口根本没数据（接线/接收机没输出） |
| `bytes` 涨、`frames=0`、`dropped=0` | 有字节但不是合法该协议帧 → **波特率错**或接收机还在另一种协议 |
| `dropped` 涨、`frames` 不涨 | 帧收到了但被 target/sysid 过滤 → 调 `rc_rx_sysid` |
| `frames` 涨、`link=up` | 通 |

`hex` 抓最近 16 字节：`FD` = MAVLink v2、`C8/EE` = CRSF。注意 `FD` 也可能只是数据里的巧合字节，要配合长度/incompat 字段判断（本次实测 `FD A1 0D` 中 len=161、incompat=0x0D 均非法，故不是帧头）。

CRSF 侧的对应判据在 `health`：`rc_frames` 几乎不涨而 `rc_crc`/`rc_len` 疯涨 = 波特率错（本次实测 420000 下 3 秒 `rc_frames` +17、`rc_crc` +5622）。

## 4. 实测数据（共轴 CoaxialVTVL_ICM45686 台架）

- 波特率扫描（每档 1.5 s）：460800 收 195 帧，420000 / 115200 / 400000 / 921600 全部 0 帧
- 稳定后：2 秒 `frames` +291（≈145 Hz）、`dropped=0`、`lq=98→99`、`link=up`
- 通道值合理：CH1-4 = 1500/1500/988/1500，CH5-9 = 1000（开关在下位，未解锁），CH13-16 = 881（ELRS 未映射通道的填充值）

## 5. 遗留与边界

- ELRS 侧 MAVLink 的 RC 更新率上限 100 Hz（`MAVLINK_RC_PACKET_INTERVAL = 10`），实测 145 Hz 是计数窗口误差，不外推。
- 未验证：CH5 拨动到解锁位后的端到端解锁动作（需断动力台架拨杆）、飞控长时间运行下 100 Hz RC 对控制环的跟随质量。