# MAVLink 枚举表（本项目使用的权威值）

> **来源**: 本地库 `TandemVec_FCS/lib/MAVLink/mavlink/common/common.h`（官方生成版 2024-11-27）——**以库头文件为唯一权威**，本文件只是速查。
> **触发条件**: 构造/解析 MAVLink 消息需要枚举数值时先查这里或库头文件；**库版本升级后以库为准**。

## 消息 ID + CRC_EXTRA（MAVLink 2）

| 消息 | ID | CRC_EXTRA |
|---|---|---|
| HEARTBEAT | 0 | 50 |
| SYS_STATUS | 1 | 124 |
| PARAM_REQUEST_READ | 20 | 214 |
| PARAM_REQUEST_LIST | 21 | 159 |
| PARAM_VALUE | 22 | 220 |
| PARAM_SET | 23 | 168 |
| COMMAND_LONG | 76 | 152 |
| COMMAND_INT | 75 | 152 |
| COMMAND_ACK | 77 | 143 |
| ATTITUDE | 30 | 39 |
| ATTITUDE_QUATERNION | 31 | 246 |
| GLOBAL_POSITION_INT | 33 | 104 |
| RC_CHANNELS_RAW | 35 | 244 |
| VFR_HUD | 74 | 20 |
| RAW_IMU | 27 | 144 |
| BATTERY_STATUS | 147 | 154 |
| DEBUG_VECT | 250 | 49 |
| NAMED_VALUE_FLOAT | 251 | 170 |
| STATUSTEXT | 253 | 83 |

## MAV_CMD（本项目用）

| 值 | 名称 | 参数 |
|---|---|---|
| 176 | MAV_CMD_DO_SET_MODE | param1=模式, param2=custom mode |
| 246 | MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN | param1: 0=无操作, **1=重启飞控**, 2=关机, 3=重启进 bootloader |
| 400 | MAV_CMD_COMPONENT_ARM_DISARM | param1: **0=disarm, 1=arm**, 21196=force |

## MAV_PARAM_TYPE

| 值 | 名称 |
|---|---|
| 1 | UINT8 |
| 2 | INT8 |
| 3 | UINT16 |
| 4 | INT16 |
| 5 | UINT32 |
| 6 | INT32 |
| 7 | UINT64 |
| 8 | INT64 |
| 9 | REAL32（float——本项目统一用此） |
| 10 | REAL64 |

## MAV_RESULT（COMMAND_ACK.result）

| 值 | 名称 |
|---|---|
| 0 | ACCEPTED |
| 1 | TEMPORARILY_REJECTED |
| 2 | DENIED |
| 3 | UNSUPPORTED（本项目拒绝 ARM/SET_MODE 用此） |
| 4 | FAILED |
| 5 | IN_PROGRESS |
| 6 | CANCELLED |

## MAV_SEVERITY（STATUSTEXT.severity）

| 值 | 名称 | 本项目用途 |
|---|---|---|
| 2 | CRITICAL | IWDG 看门狗初始化失败 |
| 4 | WARNING | DETA100 掉线 |
| 6 | INFO | 解锁/Home Point 序列 |
