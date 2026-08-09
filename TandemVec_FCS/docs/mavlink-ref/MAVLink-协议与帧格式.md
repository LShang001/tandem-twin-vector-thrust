# MAVLink 协议与帧格式（官方参考）

> **来源**: [mavlink.io](https://mavlink.io/en/) + [serialization](https://mavlink.io/en/guide/serialization.html) + [crc](https://mavlink.io/en/guide/crc.html)，2026-08-10 拉取。
> **触发条件**: 改固件 MAVLink 收发/调参桥接/帧解析时读本文件 + `MAVLink-参数服务.md` + `MAVLink-命令服务.md`。
> **本地权威实现**: `TandemVec_FCS/lib/MAVLink/`（官方 mavlink-arduino 生成版，2024-11-27，v2.0）——消息结构/CRC 以库头文件为准，本文档是协议语义参考。

## 一、协议定位

- MAVLink = 轻量级消息协议，现代混合发布-订阅 + 点对点
- 消息由 XML 方言文件定义，`common.xml` 是参考消息集
- **MAVLink 1**：每包 8 字节开销；**MAVLink 2**：12 字节开销（更安全、可扩展、支持签名 signing）

## 二、MAVLink 2 帧格式（逐字节）

| 字节索引 | 字段 | 长度 | 说明 |
|---|---|---|---|
| 0 | magic | 1 | 固定 `0xFD` |
| 1 | len | 1 | 载荷长度 0-255（可能因截断变小） |
| 2 | incompat_flags | 1 | 不兼容标志，不理解则丢弃 |
| 3 | compat_flags | 1 | 兼容标志，不理解可忽略 |
| 4 | seq | 1 | 包序列号 |
| 5 | sysid | 1 | 发送者系统 ID |
| 6 | compid | 1 | 发送者组件 ID |
| 7-9 | msgid | 3 | 消息 ID（低/中/高字节，小端） |
| 10 起 | payload | 0-255 | 消息数据，字段按大小重排，v2 截断尾部零字节 |
| payload 后 | checksum | 2 | CRC-16/MCRF4XX，**含 CRC_EXTRA 字节** |
| 可选 | signature | 13 | 仅签名标志设置时存在 |

- 多字节字段小端序
- **CRC_EXTRA**：检测收发双方消息定义兼容性（基于 over-the-air 重排后字段计算）
- MAVLink 2 扩展字段不参与 CRC_EXTRA 计算

## 三、MAVLink 1 帧格式（对比）

| 字节 | 字段 |
|---|---|
| 0 | magic `0xFE` |
| 1 | len |
| 2 | seq |
| 3 | sysid |
| 4 | compid |
| 5 | msgid（1 字节！v2 是 3 字节） |
| 6 起 | payload |
| 尾 | checksum 2B |

最小 8 字节，最大 263 字节；无 flags、无截断、无签名。

## 四、解析入口（库 API）

```c
// mavlink_helpers.h:990 —— 逐字节喂给解析器，返回：
//   MAVLINK_FRAMING_OK=1 完整帧 / MAVLINK_FRAMING_INCOMPLETE=0 / MAVLINK_FRAMING_BAD_CRC=2
uint8_t mavlink_parse_char(uint8_t chan, uint8_t c,
                           mavlink_message_t* r_message,
                           mavlink_status_t* r_mavlink_status);

// 通道缓冲：mavlink_types.h 默认 MAVLINK_COMM_NUM_BUFFERS=4（嵌入式）
// 单串口用 MAVLINK_COMM_0；静态零初始化即用（parse_state=UNINIT 与 IDLE 同分支），无需 setup
mavlink_status_t* mavlink_get_channel_status(uint8_t chan);
```

## 五、发送 API（固件现行模式）

```c
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
mavlink_msg_xxx_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, ...);   // sysid=1, compid=1
uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
Serial6.write(buf, len);
```

## 六、本项目固件消息清单（mavlinkSendTelemetry，communication.cpp:1597）

| 消息 | 槽位 | 频率 | 备注 |
|---|---|---|---|
| HEARTBEAT | %200==0 | 1Hz | QGC 识别飞控的必需消息 |
| SYS_STATUS | %200==1 | 1Hz | 传感器状态/电池 |
| ATTITUDE | %5==0 | 40Hz | 欧拉角 |
| ATTITUDE_QUATERNION | %5==1 | 40Hz | 四元数 |
| GLOBAL_POSITION_INT | %10==2 | 20Hz | GPS+速度+航向 |
| VFR_HUD | %10==3 | 20Hz | MP HUD 主界面 |
| RC_CHANNELS_RAW | %10==4 | 20Hz | 8 通道 |
| RAW_IMU | %5==3 | 40Hz | 原始 IMU |
| BATTERY_STATUS | %200==5 | 1Hz | 电池 |
| NAMED_VALUE_FLOAT | %5==2 | 40Hz | AnoVars watch 变量轮转（名称≤9 字符） |
| DEBUG_VECT | %5==4 | 40Hz | 3 元素向量（M_ff/alpha_ref/error） |
| PARAM_VALUE | %10==1 | 20Hz | 参数广播（双向扩展新增，见参数服务文档） |
| STATUSTEXT | %200==2 | 1Hz | 状态文本队列（双向扩展新增） |

**空闲槽**：%10==0/5/6/7/8/9、%200==3/4（新增消息用）。
