# MAVLink 参数服务（Parameter Protocol，QGC 调参流程）

> **来源**: [mavlink.io/en/services/parameter.html](https://mavlink.io/en/services/parameter.html)，2026-08-10 拉取。
> **触发条件**: 实现/修改 QGC 参数读写桥接（固件 `src/communication.cpp` MAVLink 分支 + `ano_params` 桥）、排查 QGC 参数面板异常时读本文件。
> **本项目实现**: 固件 MAVLink RX 分支（PARAM_REQUEST_LIST/READ/SET → PARAM_VALUE 回传），参数表 = ano_params 121 参数注册表。

## 消息 ID

| 消息 | ID | 方向 | CRC_EXTRA |
|---|---|---|---|
| PARAM_REQUEST_READ | 20 | GCS→飞控 | 214 |
| PARAM_REQUEST_LIST | 21 | GCS→飞控 | 159 |
| PARAM_VALUE | 22 | 飞控→GCS | 220 |
| PARAM_SET | 23 | GCS→飞控 | 168 |

## 一、读取全部参数（PARAM_REQUEST_LIST）

1. GCS 发 `PARAM_REQUEST_LIST(target_system, target_component)`
2. 飞控收到后**逐条广播 PARAM_VALUE**（不是只回一次）
3. 每条 PARAM_VALUE 含 `param_index`（当前序号）和 `param_count`（总数）
4. GCS 用这两个字段判断是否收全（缺失检测）
5. **发送间隔**：逐条之间留间隔，避免占满链路带宽（本项目 20ms/条 ≈ 20Hz）

## 二、读取单参数（PARAM_REQUEST_READ）

- GCS 发 `PARAM_REQUEST_READ(target_system, target_component, param_id, param_index)`
- **优先按 param_id（名称）读**——索引可能因参数动态增删而变化
- param_index 与 param_id 二选一（index=-1 表示用名称；id 全 0 表示用索引）

## 三、写入参数（PARAM_SET）

1. GCS 发 `PARAM_SET(target_system, target_component, param_id, param_value, param_type)`
2. **飞控写入后必须回发 PARAM_VALUE 作为确认**——即使写入失败也要回当前值（本项目：读回当前值，QGC 比对期望值判断成功）
3. 超时未收到回发 → GCS 重发

## 四、param_id 字符串规则（char[16]）

- `param_id` 为 `char[16]`，**小于 16 字符时末尾 `\0` 结尾**
- **恰好 16 字符时无 NUL 终结符**
- 超过 16 字符**无法在标准协议中表达**（只能截断或自定义扩展）
- 本项目 121 参数中 27 个 >16B（`rate_pitch.int_limit` 等）+ 8 个恰 16B——MAVLink 桥统一**截断到 15B**（探索已核实无功能冲突：`..._int_lim` vs `..._thresho` 第 2 字符即不同；`out_mi` vs `out_ma` 仅末字符不同，QGC 界面易混淆但功能正确）

## 五、参数类型与编码

- `MAV_PARAM_TYPE` 枚举（常见）：UINT8=1 / INT8=2 / UINT16=3 / INT16=4 / UINT32=5 / INT32=6 / UINT64=7 / INT64=8 / REAL32=9 / REAL64=10
- 本项目参数全 float + u8 enabled → **统一按 REAL32(9) 上报**（enabled 转 0.0/1.0），C-style cast 编码
- 两种编码方式：**byte-wise**（按字节复制，整数精度无损）vs **C-style cast**（转 float，>2^24 整数丢精度）——通过 AUTOPILOT_VERSION.capabilities 的 `MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE/_C_CAST` 告知；**两者都不设也可用**（本项目不设）

## 六、已知实现差异（其他飞控的坑，参考勿踩）

| 实现 | 差异 |
|---|---|
| PX4 | 仅 FLOAT/INT32；发送 PARAM_VALUE 前先发 hash（param_index=INT16_MAX）做缓存校验 |
| ArduPilot | 写参数后**不主动回发 PARAM_VALUE**（与文档不一致）；PARAM_SET 的 param_type 被忽略按内部类型处理；参数集可运行中启停→索引不稳定 |

## 七、本项目桥接设计

- 参数表接口（ano_params.h 导出）：`anoParamCount()`=121 / `anoParamNameAt(id)` / `anoParamReadFloat(id)` / `anoParamWriteFloat(id, val)`（写后 applyFlightCtrlParams 无扰同步）
- PARAM_REQUEST_LIST → 逐条 PARAM_VALUE（param_index 0..120，20ms 节流）
- PARAM_REQUEST_READ → 按截断名匹配或 param_index 读单条
- PARAM_SET → 写入 + 回读确认（文档要求写失败也回当前值）
