# MAVLink 命令服务（Command Protocol）

> **来源**: [mavlink.io/en/services/command.html](https://mavlink.io/en/services/command.html)，2026-08-10 拉取。
> **触发条件**: 实现/修改 QGC 命令处理（固件 COMMAND_LONG 分支）、排查 QGC 按钮无响应时读本文件。
> **本项目实现**: 固件 MAVLink RX 分支 COMMAND_LONG 处理（communication.cpp），命令枚举值见 `MAVLink-枚举表.md` 或库 `common.h`。

## 消息 ID

| 消息 | ID | 方向 | CRC_EXTRA |
|---|---|---|---|
| COMMAND_LONG | 76 | GCS→飞控 | 152 |
| COMMAND_INT | 75 | GCS→飞控 | 152 |
| COMMAND_ACK | 77 | 飞控→GCS | 143 |

## 一、消息字段

- **COMMAND_LONG**：`command`（MAV_CMD 枚举）+ `confirmation` + `param1..param7`（**全 float**）
- **COMMAND_INT**：同上但 param5/6 为**缩放整数**（适合高精度经纬度）；含 `frame` 坐标系字段
- 位置/导航类命令优先 COMMAND_INT；纯浮点参数（尤其 param5/6）必须 COMMAND_LONG

## 二、ACK 与重试（QGC 行为，飞控实现须知）

- 发送命令后**必须等待匹配的 COMMAND_ACK**
- 未收到 ACK → 自动重发，**递增 `confirmation` 字段**（不是相同值）
- 重试次数由飞控决定（本项目：QGC 默认 3 次）
- **长时命令**：先回 `MAV_RESULT_IN_PROGRESS`（带 progress 0-100%），最终回终态 ACK（ACCEPTED/FAILED/CANCELLED）；收到 IN_PROGRESS 后 QGC 大幅延长超时
- 飞控必须每条命令**都回 ACK**（即使 UNSUPPORTED/FAILED）

## 三、MAV_RESULT 枚举（COMMAND_ACK.result）

| 值 | 含义 |
|---|---|
| 0 ACCEPTED | 命令有效且将被执行（≠已完成） |
| 1 TEMPORARILY_REJECTED | 目标忙，稍后重试 |
| 2 DENIED | 被拒绝（如权限） |
| 3 UNSUPPORTED | 不支持该命令 |
| 4 FAILED | 执行失败 |
| 5 IN_PROGRESS | 长时命令进行中（带 progress） |
| 6 CANCELLED | 已取消 |

## 四、本项目命令处理策略

| MAV_CMD | 处理 |
|---|---|
| 246 PREFLIGHT_REBOOT_SHUTDOWN（param1=1） | ✅ 软复位（= DBG reset 的 MAVLink 版） |
| 400 COMPONENT_ARM_DISARM | ❌ 拒绝（MAV_RESULT_UNSUPPORTED + STATUSTEXT 提示"解锁由 RC CH5 控制"）——解锁=RC 硬判定（isLinkUp && ch5>1500），MAVLink 置位会被下帧覆盖，且地面站远程解锁是安全红线 |
| 176 DO_SET_MODE | ❌ 同拒（模式由 RC 每帧判定） |
| 其他 | ❌ MAV_RESULT_UNSUPPORTED |

**所有命令一律回 COMMAND_ACK**（文档硬性要求）。
