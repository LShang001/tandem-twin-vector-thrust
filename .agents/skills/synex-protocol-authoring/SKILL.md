---
name: "synex-protocol-authoring"
description: "在 Synex 上位机里给自定义二进制遥测帧（ANO 0x03/0x05/0x07/0x08/0x21/0xF1 等）建解析协议并跑到出曲线：核实运行实例与 AI Bridge 发现文件、回固件源码定字段与缩放、建协议→发布→示例帧核帧长；无设备时用 UDP 仿真源做端到端验证（一个数据报一帧），含 Plot 通道订阅、通道数变化清订阅、Qt 界面自动化与 ConvertTo-Json 深度截断等实测坑。"
---

# Synex 自定义二进制协议建模

在 Synex 上位机里为一类二进制遥测帧建解析协议，并通过 AI Bridge 完成建协议、发布、示例帧核帧长。

## When to Use

- 要在 Synex 里新增/修正一帧自定义二进制协议的解析，用来画曲线。
- 现有协议字段错位、帧长对不上、曲线明显是假的。
- 需要把固件某帧结构（ANO 0x03 姿态 / 0x05 高度 / 0x07 速度 / 0x08 位置 / 0x21 姿态控制量 / 0xF1 执行器 等）搬到上位机复现。

## Prerequisites

- **先核实运行中的实例，别凭安装直觉**：`Get-Process Synex | Select-Object Id,Path,StartTime`。协议文件落在**该 exe 同目录的 `protocols\`** 下。本机曾出现两个副本并存（解压目录 vs `D:\Program Files\Synex`），只有正在运行的那个会收到新协议，另一份的配置会分叉。
- AI Bridge 发现文件一般形如 `%LOCALAPPDATA%\Temp\Synex\ai_bridge.json`；每次桥接返回里的 `_discoveryFile` 会直接给出实际路径。
- 手上有固件源码。字段顺序、类型、缩放系数必须回源码核，不凭印象。

## Procedure

1. **定字段**：按符号名在固件里搜帧构造函数（如 `AnoComProtocol.cpp` 的 0x03/0x05、0x07/0x08 分支，`communication.cpp` 的 0xF1 分支），抄出字段顺序、类型（i16/i32/u8）、缩放。记录用「文件 + 符号名 + 偏移」，不用固定行号——行号会漂。
   已知缩放：角度/速度 ×0.01，气压高度 ×0.01，电机百分比 ×0.1，差速 ×0.001。
2. **算帧长**：帧长 = 帧头 + 负载 + 尾巴。先手算一张「期望字节数」表，后面拿它当断言。
3. **建协议**：逐个调用桥接 import，命名统一 `ANO_0xNN_功能`（名字后面 publish 要用它定位）。
4. **发布**：用 name 定位 publish，确认 `ok=true`，且返回的 `filePath` 落在第 0 步核实过的实例目录。
5. **核帧长**：对每份协议生成示例帧，比对 `byteLength` 与第 2 步期望值，全部相等才算过。
6. **落表**：交付一张 功能码 / 名称 / 帧长 / 曲线数 / 字段构成 表，附源码依据（文件名 + 符号）；产物给完整路径。
7. **说明限制**：一个连接只能挂一个协议，一帧最多 7 条曲线；想切看别的帧，得回连接页把协议换掉。
8. **端到端验证（无设备时）**：写 UDP 仿真源，**一个数据报 = 一帧**（Synex 按固定帧长切帧）；按第 1 步的字段与缩放生成正弦扫掠（如姿态 roll ±10°/0.5 Hz、pitch ±8°/0.3 Hz、yaw ±30°/0.1 Hz）。参考实现见本技能 `assets/udp_ano_sim.py`。Synex 连接页配 `UDP / 127.0.0.1 / 本地端口 / CustomBinary / ANO_0xNN_xxx` 后打开连接开关，状态栏应出现 `↓ X B/s`，且 **X ≈ 帧长 × 帧率**（20 B × 50 Hz ≈ 1000 B/s）。
9. **核数值而非只看字节**：桥接侧 `get_latest_channel_values` 应返回 `connected=true`、`sequence` 持续递增、各通道值落在物理量程内——值带小数（如 `-8.449999809265137`）说明 ×0.01 缩放已生效，若看到 `-845` 这类整数则是缩放没配。
10. **建 Plot 并订阅**：Plot 的 `config.selectedChannels=[...]` 必须在 `addTile` 时一起传（见 Pitfalls）。曲线出来算真正跑通，判据是 `Pts` 持续增长而不是 `0w`。

## Pitfalls

- **PowerShell `ConvertTo-Json` 默认深度 2**：嵌套对象被静默截断，桥接端收不到子字段，报出来的却是「请提供 protocolId、rowIndex 或 name 来定位协议」这类缺参错。别顺着错误提示去加参数——先看是不是序列化把参数吃了。对策：`-Depth 10` 以上，或把 JSON 写文件后以 `--data-binary @file` 传，绕开序列化层。
- 产物没出现在预期目录时，先查进程实际路径，不要继续重试调用。
- 协议整帧不校验（sum/add 被当 skip 丢弃）：一旦错位会一路画出假曲线，只能用于观察趋势，不能拿来判断结果。
- 没接设备时，验证只到「协议合法、帧长对」，不能宣称能出曲线——除非按 Procedure 第 8–10 步跑了 UDP 仿真。
- **`updateTileConfig` 改不动运行中的 Plot**：调用返回 `ok=true`，回读 `selectedChannels` 仍为 `[]`、`yRange` 不变。可靠做法是 `deleteTile` + `addTile`，把 `selectedChannels`、`yRangeLower/yRangeUpper`、`customTitleText`、`unit`、`useBoundChannelName=false` 全部放进新建时的 `config`。Numeric/Gauge 的自定义标题同理——只给 `customTitleText` 而 `useBoundChannelName` 还是 true，界面仍显示 `CH0`。
- **通道数一变，Plot 订阅就被清空**：切协议时状态栏会提示「检测到绘图通道数变化：N -> M 通道」，原有订阅作废，曲线停画而 `FPS` 仍正常——这就是 `Pts: 0w` 的成因。切完协议必须按新通道重建 Plot。
- **`deleteWorkspace` 是 patch operation，不是工具**：当 tool 直接调会报 `Unknown tool: deleteWorkspace`；要放进 `apply_plot_workspace_patch` 的 `operations`。
- **MCP 没有连接配置能力**：37 个工具只覆盖协议编辑器、指令台、绘图工作台，**连接类型/端口/解析器/协议选择只能在 GUI 里配**，切协议也得靠界面。
- **Qt 界面自动化要点**：UIA 能看到导航 TreeItem 和 ComboBox 弹窗的 ListItem，但 `background:Invoke` 对它们无效（返回 `sent` 却不生效），必须 `dispatch=foreground` 真实鼠标。屏幕坐标 = UIA 逻辑坐标 + 窗口原点（本机窗口原点 −13,−13）。ComboBox 展开的列表是**独立窗口** `Qt672QWindowPopupSaveBits`，主窗口截图里看不到，`list_windows` 里按 className 找；更省事的办法是让 ComboBox 拿到焦点后直接按 `Up`/`Down`，状态栏会回显「已切换自定义协议：xxx」。
- **`auto_build_plot_workspace` 即使 `dryRun:true` 也会建一个空页**，用完要 `deleteWorkspace` 清掉。

## Verification

- `publish` 返回 `ok=true`，`filePath` 在已核实实例的 `protocols\` 下；
- `ls protocols\` 能看到本次时间戳的 .json；
- 每份协议的示例帧 `byteLength` == 手算期望值；
- 端到端（接了 UDP 仿真后）：状态栏 `↓ X B/s` ≈ 帧长 × 帧率；`get_latest_channel_values` 的 `connected=true`、`sequence` 递增、通道值在物理量程内（缩放生效）；Plot 的 `Pts` 持续增长且 `FPS` ≈ 发送帧率；
- 切换协议后重查一次通道数，并确认 Plot 订阅已按新通道数重建（否则曲线会静默消失）；
- 交付表格每一行都有源码依据（文件 + 符号），没有「凭印象」项。
