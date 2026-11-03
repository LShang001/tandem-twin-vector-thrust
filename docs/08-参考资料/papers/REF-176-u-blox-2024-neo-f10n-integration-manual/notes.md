# REF-176 · u-blox NEO-F10N 集成手册（UBXDOC-963802114-12193 R04）

- 转换：MinerU 4.0.12（standard tier，72 页）
- 来源：https://content.u-blox.com/sites/default/files/documents/NEO-F10N_IntegrationManual_UBXDOC-963802114-12193.pdf
- 器件：u-blox NEO-F10N，L1/L5 双频标准精度 GNSS 模组；固件 EXT SPG 6.00。
- 与本项目关系：飞控 `src/ubx_config.cpp` 的接收机初始化、端口/消息速率/动态模型配置依据；
  实测排障见 `docs/04-数学建模/代码理论核查/F10N-GNSS配置与调试-2026-11-03.md`。

## 与本项目直接相关的结论

| 主题 | 结论 | 章节/页 |
| --- | --- | --- |
| 默认星座 | GPS + Galileo + BeiDou 并发，SBAS 使能；L5 默认参与双频接收（GPS L5 健康度预运营问题单独说明） | §2.1.3 p.10 |
| 默认串口 | 38400 baud、8 数据位、无校验、1 停止位；支持 9600/19200/38400/57600/115200/230400/460800/921600 | §3.2 p.21 |
| 无流控 | UART 不支持硬件握手/流控 | §3.2 p.21 |
| 改波特率时序 | 发完改波特率消息后至少等约 100 ms，再按新速率发数据 | §3.2 p.21 |
| RX 保护 | 1 s 内 >100 个帧错误会关闭 UART RX 一秒（错波特率/接地常见） | §3.2 p.21 |
| 配置模型 | 推荐 CFG-VALSET/CFG-VALGET/CFG-VALDEL（新配置项），UBX-CFG-* 传统消息逐步弃用 | §2.1/§2.2 |
| 天线监督 | 模组支持天线短路/开路检测与自动恢复（MON-VER 的 ANTSUPERV 字段可见 AC/SD/OD/PDoS/SR） | 数据手册/接口手册 |
| 导航率 | 多星座 10 Hz；单星座可 20 Hz（数据手册 REF-175） | REF-175 |

## 调试要点

- 先 `UBX-MON-VER` 确认固件/协议版本与 ANTSUPERV 配置（固件目前用 `ubxcfg status`/串口工具读，`ubxcfg monver` 为待补命令）。
- 室内/无天线时 `fix=0` 正常，但 NAV-PVT 仍应持续输出；**PVT 计数为 0 是配置/端口问题，不是无星问题**。
- 天线 `SHORT/OPEN` 时先查天线与馈线，不要把 `fix=0` 归因于固件解析。
- 配置成功后必须用 `ubxstat` 看 `pvt/eoe` 是否增长，不能只看 `ubxcfg status` 的 ACK。

## 转换审计与人工修正

- 审计报告：同目录 `conversion-audit.md`（子代理逐页对照，词元覆盖 99.32%）。
- 已按审计修正 `paper.md` 两处 P0：
  1. §3.8.2.1 time pulse 示例：伪 LaTeX、`0→O`、`1→T`、`µs→ps`、`TP1→TPT` 已还原为 12 条纯文本；
  2. Table 29 天线规格：塌陷的 3 列表已按原文重建（被动/可选放大两段）。
- 标题层级仍为单级 `##`；它是 MinerU 渲染限制，不影响数值引用。引用表格时以 PDF 页码为准。

