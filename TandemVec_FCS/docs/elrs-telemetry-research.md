# ELRS 遥测机制调研与 CrsfSerial 三方对比

> 调研日期：2026-08-08
> 官方源码沉淀：`docs/reference/elrs-official/`（ExpressLRS commit 5909f77, 2026-07-31）
> 相关代码：`lib/CrsfSerial/`（自魔改版）、`src/communication.cpp`（sendElrsBatteryData 等）
> 来源：ExpressLRS 官方仓库（https://github.com/ExpressLRS/ExpressLRS）+ TBS CRSF 规范

---

## 一、ELRS 遥测回传机制

### 1. 链路拓扑

```
飞控 FC (本机, Serial1) --CRSF 420kbaud--> ELRS RX 接收机 --2.4G 射频--> ELRS TX 模块 --CRSF--> 遥控器(EdgeTX/OpenTX)
    ▲  回传遥测帧                                              ▲                                 ▲
    └──── BATTERY/GPS/ATTITUDE/... ────────────────────────────┴── 遥控器显示 ─────────────────┘
```

### 2. RX 端如何处理 FC 遥测（核心：SerialCRSF）

`src/src/rx-serial/SerialCRSF.cpp`（沉淀源码内）：

- **`processBytes()`**：解析 FC 发来的 CRSF 帧，仅做两个标记检测：
  ```cpp
  if (message->type == CRSF_FRAMETYPE_BATTERY_SENSOR)  crsfBatterySensorDetected = true;
  if (type == CRSF_FRAMETYPE_BARO_ALTITUDE || type == CRSF_FRAMETYPE_VARIO) crsfBaroSensorDetected = true;
  ```
- **`sendRCFrame()`**：向遥控器发 RC 通道帧（`CRSF_FRAMETYPE_RC_CHANNELS_PACKED`），
  并把 **LQ 映射到 CH14、RSSI dBm 映射到 CH15**（非 16 通道模式时）。
- **`forwardMessage()`**：teamrace 模式下原样转发 MSP 数据。

**关键结论：ELRS RX 对 FC 遥测帧是"透明转发"**——不挑帧类型，只要 FC 发标准 CRSF 帧就回传。唯一的类型检测只是用于某些模式自动切换。

### 3. 遥测传输的两种封装（与协议选择相关）

| 路径 | 触发条件 | 机制 |
|---|---|---|
| **直接 CRSF** | 默认（`SerialCRSF`） | FC 的 CRSF 帧原样经射频回传 |
| **MSP over CRSF** | FC 用 MSP（如 Betaflight） | `CRSF2MSP`（`src/lib/CRSF2MSP/`）把 CRSF 封装的 MSP 分片重组 |
| **MAVLink** | RX 配 `PROTOCOL_MAVLINK` | `MSP_ELRS_MAVLINK_TLM`(0xFD) 通道转发（`rx_main.cpp:1243`） |
| **MSP DisplayPort** | RX 配 `PROTOCOL_MSP_DISPLAYPORT` | OSD 用（`rx_main.cpp:1304`） |

**对本项目（FC 直连 CRSF）**：走**直接 CRSF 路径**，无需 MSP/MAVLink 转换。FC 用 `queuePacket()` 发标准 CRSF 帧即可。

### 4. RX 侧其他遥测源（FC 之外）

- **SerialGPS**：RX 串口直连 GPS 时自组 `CRSF_FRAMETYPE_GPS` 帧（`SerialGPS.cpp:244 sendTelemetryFrame`）
- **SerialHoTT_TLM**：HoTT 遥测转换（`SerialHoTT_TLM.cpp`）
- **AnalogVbat**：RX 板载电压（`devAnalogVbat.cpp`）

---

## 二、CRSF 回传帧类型全景（官方定义）

### ELRS 官方 `src/include/crsf_protocol.h` 全部帧类型

| 帧类型 | 值 | 用途 | 本机是否可用 |
|---|---|---|---|
| GPS | 0x02 | 经纬度/地速/航向/高度/卫星数 | ✅ |
| **GPS_TIME** | 0x03 | GPS 时间 | ✅ 需补枚举 |
| **VARIO** | 0x07 | 垂直速度（cm/s） | ✅ 需补枚举 |
| BATTERY_SENSOR | 0x08 | 电压/电流/容量/剩余% | ✅ 已用 |
| **BARO_ALTITUDE** | 0x09 | 气压高度+垂直速度 | ✅ 需补枚举 |
| AIRSPEED | 0x0A | 空速（0.1km/h） | ✅ |
| **HEARTBEAT** | 0x0B | 心跳 | ✅ 需补枚举 |
| RPM | 0x0C | 电机转速 | ✅ |
| TEMP | 0x0D | 温度 | ✅ |
| CELLS | 0x0E | 电芯电压 | ✅ |
| OPENTX_SYNC | 0x10 | OpenTX 同步 | 库内部用 |
| LINK_STATISTICS | 0x14 | RSSI/LQ/SNR | RX 自动发 |
| RC_CHANNELS_PACKED | 0x16 | 通道 | RX 自动发 |
| ATTITUDE | 0x1E | 姿态（弧度×10000） | ✅ |
| FLIGHT_MODE | 0x21 | 模式名 | ✅ |
| DEVICE_PING/INFO | 0x28/0x29 | 设备管理 | — |
| PARAMETER_* | 0x2B-0x2D | 参数读写 | — |
| COMMAND | 0x32 | 命令 | — |
| **ELRS_STATUS** | 0x2E | 好坏包计数 | RX 内部用 |
| **HANDSET** | 0x3A | 时序同步 | TX 内部用 |
| **KISS_REQ/RESP** | 0x78/0x79 | KISS 协议 | — |
| MSP_REQ/RESP/WRITE | 0x7A-0x7C | MSP 封装 | FC 用 MSP 时 |
| **ARDUPILOT_RESP** | 0x80 | ArduPilot | — |

### 本机自魔改版缺失的枚举（需补齐）

```
GPS_TIME 0x03, VARIO 0x07, BARO_ALTITUDE 0x09, HEARTBEAT 0x0B,
ELRS_STATUS 0x2E, HANDSET 0x3A, KISS_REQ 0x78, KISS_RESP 0x79, ARDUPILOT_RESP 0x80
```

对火箭遥测最有价值：**VARIO**（垂直速度）、**BARO_ALTITUDE**（高度）。

---

## 三、CrsfSerial 三方对比

| 维度 | BobbyIndustries 原版 (2022-08 停更) | 本机自魔改版 | ELRS 官方 (活跃) |
|---|---|---|---|
| 最后更新 | 2022-08 | — | 2026-07 |
| queuePacketChannels/setChannel | ❌ | ✅ | 架构不同 |
| onOobData 回调 | ❌（onShiftyByte） | ✅ | 架构不同 |
| 帧类型枚举 | 缺 AIRSPEED/RPM/TEMP/CELLS | 全（缺 9 个 ELRS 新增） | 全 |
| 帧长校验 | 宽松 | 严格（PAYLOAD_LEN+2） | 严格 |
| 适用 | 教学参考 | **本项目现状** | ESP32 专用架构 |

**结论**：
1. BobbyIndustries 原版停更 4 年，无升级价值
2. 本机魔改版 API 已超原版，**库本身无需升级**
3. ELRS 官方架构（CRSFRouter/CRSF2MSP）是 ESP32 专属，不能直接替换；
   但 **crsf_protocol.h 是权威协议定义，值得同步补全枚举**

---

## 四、本机现状与扩展建议

### 现状（`communication.cpp:700-733`）
- 仅 `sendElrsBatteryData`（25Hz）：借用 BATTERY_SENSOR 帧传氧压 P1/P2

### 建议扩展（按价值排序）
1. **ATTITUDE 回传**（0x1E）：`AHRS_Packet.Roll/Pitch/Heading` → 遥控器姿态球
2. **BARO_ALTITUDE 回传**（0x09）：DPS310 高度 → 遥控器高度显示
3. **VARIO 回传**（0x07）：垂直速度 → 变率计
4. **FLIGHT_MODE 回传**（0x21）：飞行模式名

### 实现要点
- 结构体 `crsf_sensor_attitude_t` / `crsf_sensor_baro_vario_t` 本机已有
- 大端转换：`htobe16/htobe32`（ELRS 官方同样用 htobe）
- 发送：`crsf.queuePacket(CRSF_FRAMETYPE_XXX, &data, sizeof(data))`
- 注意 `sendElrsBatteryData` 的 TX 缓冲非阻塞保护模式（`availableForWrite` 检查）

---

## 五、沉淀内容索引

| 资源 | 位置 |
|---|---|
| ELRS 官方源码 | `docs/reference/elrs-official/src/` |
| 版本标记 | `docs/reference/elrs-official/VERSION.txt` |
| CRSF 协议定义 | `docs/reference/elrs-official/src/include/crsf_protocol.h` |
| RX 遥测转发 | `docs/reference/elrs-official/src/src/rx-serial/SerialCRSF.cpp` |
| MSP 转换 | `docs/reference/elrs-official/src/lib/CRSF2MSP/` |
| TX 遥测组装 | `docs/reference/elrs-official/src/lib/Handset/CRSFHandset.cpp` |
