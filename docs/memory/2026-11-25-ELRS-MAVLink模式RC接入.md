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
- 稳定后：`frames` 3 秒 +134（约 45 帧/秒，短窗口误差大）、`dropped=0`、`lq=98→99`、`link=up`
- 通道值合理：CH1-4 = 1500/1500/988/1500，CH5-9 = 1000（开关在下位，未解锁），CH13-16 = 881（ELRS 未映射通道的填充值）

## 5. 库层缺陷：CrsfSerial::begin(baud) 不同步 _baud

`lib/CrsfSerial/CrsfSerial.cpp` 原实现只重开串口、**不更新 `_baud`**：

- `getBaud()` 返回的仍是构造时的出厂值，与实际速率不一致；
- 之后任何无参 `begin()` 都会用旧 `_baud`（420000）重开串口，**静默覆盖**调用方设的速率。

本次就是这样踩到的：`rcInputBegin()` 设好 460800，`main.cpp` 里遗留一句无参 `crsf.begin()` 又把它改回 420000，症状是"波特率读数对、frames 恒 0、hex 乱码"。

已修（2026-11-25）：显式传参时同步 `_baud`。配套规则：**重开 RC 串口只走 `rcInputBegin()` 或 DBG `rcbaud`，不要裸调 `crsf.begin()`**。

## 6. 验收数据（共轴 CoaxialVTVL_ICM45686 台架，最终版）

三段一次跑完（`env:CoaxialVTVL_ICM45686_RcMav`）：

| 阶段 | proto | baud | frames | link |
|---|---|---|---|---|
| 冷启动（不切换） | mavlink | 460800 | 1063 | **up** |
| `rcproto crsf` | crsf | 420000 | 冻结 | down（RX 在发 MAVLink，属预期） |
| `rcproto mavlink` | mavlink | 460800 | 1666 | **up** |

- 冷启动直接可用：不需要先切一次协议才纠正波特率。
- 切换自动带波特率且自动重建链路，`frames` 继续增长、`dropped=0`、`lq=98~100`。
- 通道值：CH1-4 = 1500/1500/988/1500，CH5-9 = 1000（开关下位，未解锁），CH11-12 = 1500/1533，CH13-16 = 881（ELRS 未映射通道填充值）。

## 7. 回传也必须换协议（否则遥控器无数据）

ELRS 的 RX↔飞控是**一根 UART 双向、协议二选一**：

- CRSF 模式：飞控发 CRSF 遥测帧（0xC8/0xEE），RX 转发 → TX → 遥控器
- MAVLink 模式：RX 只认 MAVLink（`SerialMavlink::processBytes`），CRSF 帧被丢弃

链路机制（源码）：RX 把飞控上行字节收进 `mavlinkInputBuffer`，`GetNextPayload()` 打包成
`[CRSF_ADDRESS_USB][len][raw mavlink]` 走 OTA；TX 侧 `tx_main.cpp:1538` 看到该地址且
Link Mode=MAVLink 时调 `convert_mavlink_to_crsf_telem()` 翻译成 CRSF 传感器给遥控器。

**白名单**（`lib/MAVLink/MAVLink.cpp`，发别的遥控器不显示）：HEARTBEAT / SYSTEM_TIME /
GPS_RAW_INT / SCALED_PRESSURE / ATTITUDE / GLOBAL_POSITION_INT / VFR_HUD / ALTITUDE /
BATTERY_STATUS / HOME_POSITION / STATUSTEXT / HIGH_LATENCY2。且 TX 有硬过滤
`msg.compid != MAV_COMP_ID_AUTOPILOT1` 直接跳过。

固件实现 `src/rc_mav_telem.{h,cpp}`（RC=MAVLink 时由 `rcInputPoll()` 调用）：

| 消息 | 频率 | 用途 |
|---|---|---|
| HEARTBEAT | 1 Hz | 模式名（ArduPilot Copter custom_mode）+ 未解锁加 `*` |
| ATTITUDE | 20 Hz | 姿态页 |
| GPS_RAW_INT | 2 Hz | 坐标/星数/地速 |
| GLOBAL_POSITION_INT | 5 Hz | 相对高度 → VARIO |
| BATTERY_STATUS | 1 Hz | 电压（`id` 必须 0） |
| VFR_HUD | 5 Hz | 油门/地速/升降率 |

合计约 1.3 kB/s；F1000 + Telem 1:2 回传上限约 2.5 kB/s（占用约 52%）。

**协议互斥**：MAVLink 模式下 7 处 CRSF 遥测（`sendElrsBattery/Attitude/BaroAltitude/
FlightMode/Gps/Temp` 与通道回显）全部门控让位，避免混发污染 MAVLink 帧流。

## 8. RC 帧率：MAVLink 死限 100 Hz，与包率无关

10 秒窗口实测 **99.9 Hz**（39430 → 40501 帧 / 10.72 s），与源码常量完全吻合：

| 实现 | `sendRCFrame` 返回值 | 语义 | F1000 下 RC 帧率 |
|---|---|---|---|
| `SerialMavlink` | `MAVLINK_RC_PACKET_INTERVAL = 10` | 10 ms 后再次调用 | **100 Hz（死限，与包率无关）** |
| `SerialCRSF` | `DURATION_IMMEDIATELY`（0） | 每 loop 都发 | 跟随包率，F1000 下约 **250 Hz** |

返回值语义见 `device.h`：返回的是"多久后再调 `timeout()`"的毫秒数。

**F1000 的 1000 包/s 去哪了**：`OTA4_PACKET_SIZE = 8`、载荷 5 B = 4 通道 × 10 bit，16 通道需 4 个包拼一帧 → 空口完整 RC 上限 = 1000 ÷ 4 = **250 Hz**。MAVLink 固定在 100 Hz 只用掉 40%。

**改 `MAVLINK_RC_PACKET_INTERVAL` 的带宽影响**（评估结论，未实施）：RC 帧是 RX→FC 方向、且 RC 数据本来就在空口包固定字段里，**不占遥测 payload**。改 10→5（200 Hz）后 FC→RX 串口占用从 6.6 涨到 11.9 kB/s（460800 的 14%→26%），无线回传 2.5 kB/s 占用不变（仍 52%）。收益是 RC 更新率 100→200 Hz、延迟 10–13→5–8 ms，逼近空口 250 Hz 上限。

## 9. 遗留与边界

- 未验证：CH5 拨到解锁位后的端到端解锁动作（需断动力台架拨杆）、长时间运行下 100 Hz RC 对控制环的跟随质量。
- 未做：把 `MAVLINK_RC_PACKET_INTERVAL` 改 5 需要改 ELRS RX 固件并重刷接收机，属上游改动、无官方背书，未实施。