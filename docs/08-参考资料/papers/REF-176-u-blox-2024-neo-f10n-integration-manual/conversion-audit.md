# REF-176 NEO-F10N Integration Manual — MinerU 转换质量审计

- **受审对象**
  - 原文：`paper.pdf`，u-blox *NEO-F10N Integration manual*，文档号 **UBXDOC-963802114-12193 R04**，日期 12-Dec-2024，C1-Public
  - 转换件：`paper.md`（MinerU，tier=standard），179,971 字节，2,932 行，72 个页标记
- **核对方法**：`pdftotext -layout/-table -enc UTF-8` 提取原文做逐段/坐标级核对（`-layout` 版在表格列对齐上会错位，故表格类以 `-table` 版 + PyMuPDF 词坐标为准）；不修改 `paper.pdf` / `paper.md`。
- **审计日期**：2026-11-03

---

## 0. 前置事实更正

任务单中有五处与实物不符，先予以更正，避免后续引用错位：

| 项 | 任务单表述 | 实测 | 说明 |
| --- | --- | --- | --- |
| 页数 | 42 页 | **72 页** | `pdfinfo` 与 72 个 `<!-- page N of 72 -->` 标记一致 |
| "Default interface settings" 章节 | 需核对 | **不存在** | 该章属 u-blox *Interface description*（本文 [3]，UBX-23002117），非本手册 |
| "UBX messages" / "port configuration" 章节 | 需核对 | **不存在** | 同上；本手册中相关信息散在 §2.1.5、§2.1.6、§3.2.1 |
| `CFG-MSG` | 需核对描述 | **本手册无 `UBX-CFG-MSG`** | 报文速率用 `CFG-MSGOUT-*`（p12）；配置接口用 `UBX-CFG-VALSET/VALGET/VALDEL`（p9） |
| `NAV-EOE` | 重点核对 | **原文 0 次** | 非转换丢失，是该手册本就没有 |

> 结论：任务单中"Default interface settings / UBX messages / port configuration / antenna supervisor / power modes"这几项预期，是按 u-blox **接口说明（Interface Description）**的目录写的，与**集成手册（Integration Manual）**目录不同。本手册实际章节为 1 System description / 2 Receiver configuration / 3 Receiver functionality / 4 Hardware integration / 5 Product handling / 附录 A·B·C。

---

## 1. 结论

**判定：可用（需修）**

`paper.md` 可用于检索、通读与一般性引用；**但不得直接把 §3.8.2.1（p37）与 Table 29（p68）当作参数依据** —— 这两处存在会传导到实际配置的数值/单位/标识符错误与表格结构塌陷，须先修复。

判定依据：

- **覆盖充分**：72/72 页、98/98 目录条目、22/22 图、33 张表中 32 张表注齐全；词元级覆盖率 **99.32%**（24,504 词元，缺 166，绝大多数为连字符断行造成的分词差异）。未发现整段、整节丢失。
- **关键参数正确**：默认波特率 38400、固件版本 EXT SPG 6.00、默认星座、10/20 Hz 上限、NMEA V4.11、CFG-VALSET/VALGET、NAV-PVT 全部与原文一致，二进制配置串（Table 3/4/5）逐字节正确。
- **但有两类高严重度缺陷**：① p37 时脉冲示例被渲染成伪 LaTeX 并篡改了数字与单位；② p68 天线规格表 12 行数据塌进表头。二者都会让人读出错误参数。
- **结构性缺陷**：PDF 的 20/15/12/10 pt 四级标题被全部压平成 `##`，且有 5 段正文被误提升为标题。

---

## 2. 覆盖率统计

| 指标 | 原文 | paper.md | 覆盖率 |
| --- | --- | --- | --- |
| 页数 | 72 | 72（`<!-- page N of 72 -->`） | 100% |
| 目录条目（章节标题） | 98 | 98 | **100%** |
| 一级章 | 5 + 附录 A/B/C | 5 + A/B/C | 100% |
| 图 | 22 | 22 | **100%** |
| 表 | 33 | 32 表注 | 97%（缺 Table 7） |
| 表实例 | 33 | 48（42 管道表 + 6 HTML 表） | 跨页拆分 |
| 图片块 | — | 61 | 超配 39（图标/水印/图内元素） |
| 词元覆盖 | 24,504 | 缺 166 | **99.32%** |
| 标题层级 | 4 级（20/15/12/10 pt） | 1 级（148 个 `##`，0 个 `#`） | **0% 层级** |
| Note/Caution 提示框 | 12 处 CAUTION + 1 处 Note | 12 处（文字保留，语义丢失） | 100% 文字 / 0% 语义 |

### 2.1 页级词元覆盖率明细（低于 99% 的页）

| 页 | 覆盖率 | 缺口原因 |
| --- | --- | --- |
| p62 | 94.4% | 迁移表窄列换行丢空格（`2.7 V - 3.6 VNEO-7M/M8M`） |
| p69 | 95.0% | 元件型号跨行拆分（非实质缺失） |
| p59 | 95.8% | Table 26 单元格丢空格 |
| p64 | 97.3% | `C/N0`→`C/NO`、`default.In M10` |
| p37 | 97.5% | 伪 LaTeX（词元被打散为单字母） |
| p63 | 97.7% | 迁移表窄列换行丢空格 |
| p68 | 97.5% | Table 29 塌陷 |
| p60 | 98.8% | Table 26 续页丢空格 |

---

## 3. 重点参数逐项核对

| # | 参数 | 页码 | 原文 | paper.md | 结果 |
| --- | --- | --- | --- | --- | --- |
| 1 | 默认串口波特率 | p21 §3.2.1 | `The default baud rate is 38400 baud.` | L865 一致 | ✅ |
| 2 | UART 可选速率表 | p21 Table 15 | 9600…921600，8/none/1 | 8 行齐全，规范化为 4 列 | ✅ |
| 3 | 固件版本 | p2 | `EXT SPG 6.00` | L47 一致 | ✅ |
| 4 | 文档版本 | p2 | `R04`，`12-Dec-2024` | 一致 | ✅ |
| 5 | NMEA 协议版本 | p64 Table 28 | V4.11/V4.10/V4.0/V2.3/V2.1，默认 V4.11 | 一致 | ✅ |
| 6 | 配置接口报文 | p9 | `UBX-CFG-VALSET / VALGET / VALDEL` | 一致 | ✅ |
| 7 | 报文输出速率 | p12 | `CFG-MSGOUT-*`，例 `CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1` | 一致 | ✅ |
| 8 | `UBX-CFG-MSG` | — | 本手册未出现 | 未出现 | ✅（无错） |
| 9 | `UBX-NAV-PVT` | p12/14/31/40 | 17 处引用 | 17 处 | ✅ |
| 10 | `UBX-NAV-EOE` | — | **0 处** | 0 处 | ✅（原文即无） |
| 11 | 默认星座 | p10 §2.1.3 | GPS(L1C/A,L5)+Galileo(E1-B/C,E5a)+BeiDou(B1C,B2a)+SBAS | L383 一致 | ✅ |
| 12 | 导航更新率上限 | p64 Table 28 | 单星座 20 Hz / 多星座 10 Hz（F10） | 一致 | ✅ |
| 13 | 天线监测（antenna supervisor） | p22 §3.2.2 | 仅作为 PIO 复用备选提及（`CFG-I2C-ENABLED`）；**无独立章节、无检测状态字** | 一致 | ✅（原文即无此功能章） |
| 14 | 天线断开状态位 | p26 Table 18 / p27 Table 19 | `jammingState/spoofingState = 0: Unknown` → "…or the antenna is disconnected" | 正确拼接回整句 | ✅ |
| 15 | time pulse 配置键 | p37 §3.8.2.1 | 12 条 `CFG-TP-*` 赋值 | **7 条被篡改** | ❌ **严重** |
| 16 | time pulse 配置项 | p36 §3.8.2 | 16 条项目符号说明 | 完整保留 | ✅ |
| 17 | 电源管理模式 | p28 §3.5 | 连续模式 / 硬件备份 / 软件待机；**PSM 不支持** | 一致（p64 亦确认 "PSM is not supported"） | ✅ |
| 18 | 二进制配置串 | p10 Table 3、p11 Table 4/5 | 3 组 × 17 字节 | 逐字节一致 | ✅ |
| 19 | 内部 LNA 默认 | p10 §2.1.2 | normal gain | 一致 | ✅ |
| 20 | 保护级最大更新率 | p39 | 1 Hz | 一致（但前缀为"了"） | ⚠️ 低 |

---

## 4. 逐项差异表

严重度：**严重** = 会产生错误技术结论/错误配置值；**中** = 需人工复核，影响可读性与检索；**低** = 噪声/样式，不影响事实。

| # | 页码 | 原文 | 转换结果 | 严重度 |
| --- | --- | --- | --- | --- |
| D1 | **p37** §3.8.2.1 | `CFG-TP-PULSE_DEF = 0 (PERIOD)` | `$\mathsf{...PULSE\_DEF} = \mathsf{O}\;(\mathsf{PERIOD})$` → **数字 0 变字母 O** | **严重** |
| D2 | **p37** | `CFG-TP-PULSE_LENGTH_DEF = 1 (LENGTH)` | `...= T (LENGTH)` → **数字 1 变字母 T** | **严重** |
| D3 | **p37** | `CFG-TP-ALIGN_TO_TOW_TP1 = 1` | `...TOW\_TPT = \mathsf{T}` → **1→T，且 TP1→TPT** | **严重** |
| D4 | **p37** | `CFG-TP-USE_LOCKED_TP1 = 1` | `...LOCKED\_TPT = \mathsf{T}` → **1→T，TP1→TPT** | **严重** |
| D5 | **p37** | `CFG-TP-LEN_TP1 = 100 000 µs` | `...LEN\_{TPT} = 1 0 0 \; 0 0 0 \; \mathsf{ps}` → **µs 变 ps、TP1→TPT** | **严重** |
| D6 | **p37** | `CFG-TP-POL_TP1 = 1` | `...POL\_TPT = 1` → TP1→TPT | 中 |
| D7 | **p37** | `CFG-TP-PERIOD_LOCK_TP1 = 1 000 000 µs` | `...= 1000000 \; \mathsf{\mu s}` → TP1→TPT、千位空格丢失 | 中 |
| D8 | **p68** Table 29 | 3 列 17 行天线规格（Parameter/Specification/Condition） | **前 12 行塌陷**：5 行塞进表头、7 行塞进首数据行；值被连写为 `1166 - 1186 MHz50 ΩMax 2` | **严重** |
| D9 | p13 Table 8 | 表头 `Platform｜Max altitude [m]｜Max horizontal velocity [m/s]｜Max vertical velocity [m/s]｜Sanity check type｜Max position deviation` | 第 4、5 列被切错：`Max vertical [m/s]` / `velocity Sanity check type`（数据行 9 行**全部正确**） | 中 |
| D10 | p13 | `Table 7: Dynamic platform models` 表注 | **整条表注缺失**，Table 7 与 Table 8 两表直接相邻；正文"see Table 7 and Table 8"成悬空引用 | 中 |
| D11 | p64 Table 28 | `C/N0`（数字零）×5 | `C/NO`（字母 O）×5 | 中 |
| D12 | p64 Table 28 | `…is enabled by default. In M10 and F10…` | `…enabled by default.In M10…` → 句间空格丢失 | 低 |
| D13 | p64 Table 28 | `(i.e. , ,). There is no configuration` | `(i.e., ), There is no configuration` → 空 C/N0 的逗号示意丢失、句号变逗号 | 低 |
| D14 | p59–60 Table 26 | `residual humidity is dried out. Note that the preheating phase…` | `residualhumidityisdriedout.Notethatthepreheatingphase` （8 处同类） | 中 |
| D15 | p59–60 Table 26 | `If the temperature is too low, non-melting tends to be caused` | `Ifthetemperatureistoolow,non-meltingtendstobecaused` | 中 |
| D16 | p59–60 Table 26 | `The temperature rises above the liquidus temperature of 217 °C.` | `Thetemperaturerisesabovetheliquidustemperatureof217 °C.` | 中 |
| D17 | p59–60 Table 26 | `possible mechanical tensions in the products. Controlled cooling helps` | `possiblemechanicaltensionsintheproducts.Controlledcoolinghelps` | 中 |
| D18 | p42 Table 25 | `AssistNow Online｜AssistNow Oﬄine｜AssistNow Autonomous` | `AssistNowOnline｜AssistNowOffline｜AssistNowAutonomous` | 中 |
| D19 | p42 Table 25 | `Almanac in data` | `Almanacindata` | 中 |
| D20 | p62–63 Table 27 | `Feature` 列 `Dual-band L1+L5 / SAW-LNA-SAW / RF front-end` | `Dual-band L1+L5SAW-LNA-SAWRF front-end` | 中 |
| D21 | p62–63 Table 27 | `NEO-F10N: 2.7 V - 3.6 V` ⏎ `NEO-7M/M8M: 1.65 V - 3.6 V` | `NEO-F10N: 2.7 V - 3.6 VNEO-7M/M8M: 1.65 V - 3.6 V` | 中 |
| D22 | p62–63 Table 27 | `Antenna, External` / `RF path`；`Absolute max` / `RF input power` | `Antenna, ExternalRF path`；`Absolute maxRF input power` | 中 |
| D23 | 全文 | 4 级标题：20 pt（章）/15 pt（X.Y）/12 pt（X.Y.Z）/10 pt（X.Y.Z.W） | 全部压成 `##`，无 `#`，层级信息全失 | 中 |
| D24 | p25, p26, p33, p61 | 10 pt **常规正文**：`It is not recommended to restart the receiver when it is indicating spoofing.` / `The following example illustrates how this works:` / `Conformal coating of the module voids the warranty.` / `Casting voids the warranty.` | 被误提升为 `##` 标题（同义的 jamming 句在 L1088 却仍是正文 → 处理不一致） | 中 |
| D25 | p57–61 | 10 pt 粗体小标题：`Jamming indicator`、`Evaluation`、`Preventing electrostatic discharge`、`Optical inspection`、`Cleaning`、`Rework` 等 | 一律渲染为 `##`（原文为无编号引导段，层级被抬高） | 低 |
| D26 | p59 | Figure 19 图内横轴图例 `Preheating / Heating / Cooling` | 被渲染成一张只有表头的空表（`\| Preheating \| Heating \| Cooling \|`） | 低 |
| D27 | 全文 | 页眉水印 `u-blox` | **`Cblox` ×71 + `blox` ×2**（几乎每页一行 OCR 噪声） | 低 |
| D28 | 全文 | 告警/注记图标 | **`心` ×5**（p21、p22、p31×2、p40）+ **`了` ×1**（p39 保护级说明前缀）+ `4` ×2（p11 两处） | 低 |
| D29 | p1 | 页首图形 | 遗留私用区字符 `U+E002`（不可见乱码） | 低 |
| D30 | p21 | `one-second period` | `onesecond period`（连字符丢失） | 低 |
| D31 | p10 | `high-gain external active antenna` | `highgain external active antenna` | 低 |
| D32 | p54 | `Temperature-sensitive components` | `Temperaturesensitive components` | 低 |
| D33 | p57 | `grounded workstation` | `groundedworkstation`（**该缺陷在 PDF 提取阶段即存在，md 继承**） | 低 |
| D34 | 全文 | 61 个图片块 | 全部为 `![Image block](doc:…)`，**无任何图注性 alt 文本**；图片本体也不落盘（仅保留 `doc:` 引用） | 低 |
| D35 | 全文 | Note/Caution 提示框（蓝框 + 图标） | 全部退化为普通段落，0 个 blockquote，警示语义丢失 | 低 |
| D36 | p66 | `Figure 20: Typical design` | `**Figure 20: Typical design**`（加粗；其余 21 条图注为普通文本，样式不一致） | 低 |
| D37 | p2 | 表头 5 列：`Product name｜Type number｜FW version｜IN/PCN reference｜RN reference` | 前两列合并为 `colspan="2"` 单元格 `Product name Type number` | 低 |
| D38 | p59–60, p63–65, p68–69 等 | 33 张表 | 拆成 48 张（Table 1/26/27/28/31 等跨页被切成两张，续页重复表头） | 低 |

---

## 5. 抽样逐段对照记录（≥5 页，含表格页与正文页）

### 5.1 p2 — 文档信息（表格页）

原文表 `Title/Subtitle/Document type/Document number/Revision and date/Disclosure restriction` + 产品表（5 列）。
md：两组表格齐全，`UBXDOC-963802114-12193`、`R04 12-Dec-2024`、`C1-Public`、`NEO-F10N-00B-00`、`EXT SPG 6.00`、`UBXDOC-963802114-12320`、`UBXDOC-963802114-12318` **全部逐字一致**。
缺陷：产品表表头两列并入一个 `colspan="2"` 单元格（D37）。版权段、免责声明段完整。

### 5.2 p7–8 — 引脚定义（大表格页）

原文 24 引脚表 + 脚注 1。md 引脚 1–24 全部正确，`VCC_RF`/`VCC`/`V_BCKP`/`RF_IN`/`LNA_EN` 等名称无误，脚注以上标 `<sup>1</sup>` 保留并单独成段。`Oﬀ` 连字被规范化为 `Off`（属于改进）。表格在 p7/p8 交界拆成两张、续页重复表头（D38，可接受）。

### 5.3 p10 — 正文 + Table 3（二进制串表格页）

`2.1.2` 三段正文逐句一致；Table 3 三行 17 字节配置串（`B5 62 06 8A …`）**逐字节正确**；编号步骤 1–5 保留为有序列表。
缺陷：`high-gain`→`highgain`（D31）；`E1-B/C` 被断开为 `E1- B/C`（同段落，低）。

### 5.4 p13 — Table 7 / Table 8（复杂表格页）

Table 7 九行平台描述逐行正确。
Table 8 数据值经坐标级复核**全部正确**（含 `Airborne <1g 80000/100/6400`、`<2g 80000/250/10000`、`<4g 80000/500/20000` —— 这些"6400/10000/20000"确实是原文数值，非转换错误；注意此处 `pdftotext -layout` 反而错位，md 的 `-table` 对齐是对的）。
缺陷：Table 8 表头第 4/5 列切错（D9）；Table 7 表注整条缺失（D10）。

### 5.5 p21 — 正文 + Table 15（UART 波特率表）

正文四段一致；Table 15 由原文的"奇偶校验列纵向合并"结构规范化为 8 行 × 4 列，`9600/19200/38400/57600/115200/230400/460800/921600` 与 `8/none/1` 全部正确，**默认 38400 句完整保留**。
缺陷：`心` 图标噪声（D28）、`one-second`→`onesecond`（D30）。

### 5.6 p36–37 — Time pulse（正文 + 伪公式）

p36 §3.8.1 五条建议、§3.8.2 十六个配置项说明**逐条完整**。
p37 §3.8.2.1 十二条 `CFG-TP-*` 赋值中，**7 条被渲染成 `$\mathsf{...}$` 伪公式**，并发生 0→O、1→T（×3）、µs→ps、TP1→TPT（×5）的实质篡改（D1–D7）。这是本次审计最严重的问题：照抄会写出非法配置。

### 5.7 p42 — Table 25（小表格页）

`Requirements / AssistNow Online / AssistNow Offline / AssistNow Autonomous` × `Almanac in data / Yes / Yes / Yes`。
数据 `Yes` 全对，但 4 个单元格丢空格（D18/D19），列名变成 `AssistNowOnline` 之类不可检索形式。

### 5.8 p48 与 p28 — 纯正文页（对照组）

p48（4 Hardware integration、4.1、4.1.1、4.1.2）与 p28（3.5、3.5.1、3.5.2、3.5.2.1、3.5.2.2）**逐段、逐句、标点与转义（`V\_BCKP`、`VCC\_RF`）均与原文一致**，无漏段、无改写。说明正文层转换质量高，问题集中在表格与公式类版面。

### 5.9 p62–65 — 附录迁移表（最复杂表格页）

Table 27（硬件差异）与 Table 28（固件差异）的**行—列对应关系全部正确**（经 `pdftotext -table` 交叉验证；注意 `-layout` 版在此处会误把 `GLONASS` 的 `Not supported` 错配给 `SBAS`，md 是对的）。`GLONASS: Not supported`、`SBAS: New SBAS PRN selection: 123, 126-129, 131, 133, 136-138`、`PSM is not supported`、`80 000 m`、`20 Hz/10 Hz` 均可直接引用。
缺陷：窄列换行处丢空格（D20–D22）、`C/N0`→`C/NO`（D11）、拆表重复表头（D38）。

---

## 6. 建议修复项

按优先级排序。**所有修复请另存/另打补丁，勿直接改写 `paper.md`**（该文件为 MinerU 产物）——建议由 lead 决定采用"重跑 MinerU（对 p68 换用更强的表格识别）"还是"手工补丁文件"。

### P0（必须修，否则会误导配置）

1. **重写 p37 §3.8.2.1 的 7 条伪公式为纯文本**，并逐条校正：
   ```text
   • CFG-TP-TP1_ENA = 1
   • CFG-TP-PULSE_DEF = 0 (PERIOD)
   • CFG-TP-PULSE_LENGTH_DEF = 1 (LENGTH)
   • CFG-TP-PERIOD_TP1 = 1 000 000 µs
   • CFG-TP-LEN_TP1 = 100 000 µs
   • CFG-TP-TIMEGRID_TP1 = 1 (GPS)
   • CFG-TP-ALIGN_TO_TOW_TP1 = 1
   • CFG-TP-USE_LOCKED_TP1 = 1
   • CFG-TP-POL_TP1 = 1
   • CFG-TP-PERIOD_LOCK_TP1 = 1 000 000 µs
   • CFG-TP-LEN_LOCK_TP1 = 100 000 µs
   • CFG-TP-DRSTR_TP1 = 1 (4 mA drive strength)
   ```
2. **重建 p68 Table 29**（天线规格，17 行 × 3 列），值按原文照录，特别注意 `1166 - 1186 MHz` 与 `50 Ω`、`Max 2` 之间必须分开。

### P1（建议修，影响可引用性）

3. 补回 `Table 7: Dynamic platform models` 表注（p13，位于 Table 7 与 Table 8 之间）。
4. 修正 p13 Table 8 表头第 4/5 列：应为 `Max vertical velocity [m/s]` 与 `Sanity check type`。
5. 全文 `C/NO` → `C/N0`（5 处，均在 p64 Table 28 的 NMEA 行）。
6. 恢复 Table 25、Table 26、Table 27 单元格内的词间空格（约 14 处，见 D14–D22）。
7. 恢复标题层级：按 20/15/12/10 pt 分别映射为 `#`/`##`/`###`/`####`。
8. 把 5 段被误提升的正文（D24）降回普通段落。

### P2（可选，提升可读性）

9. 清理页眉水印噪声 `Cblox`（71）、`blox`（2）、`NEO-F10N - Integration manual`（71）等每页重复行。
10. 清理图标 OCR 残留：`心`（5）、`了`（1）、`4`（2）、`U+E002`（1）。
11. 恢复连字符：`onesecond`、`highgain`、`Temperaturesensitive`。
12. 将 `![Image block](...)` 换成带图号的可读 alt（如 `![Figure 3: Position output in static hold mode]`），并确认图片是否需随文档落盘。
13. 统一图注样式（Figure 20 的加粗）。
14. 将 Note/Caution 提示框还原为引用块（`>`），便于区分安全提示。
15. 合并跨页拆分的表格（Table 1/26/27/28/31 等）并删除续页重复表头。

---

## 7. 附：审计用命令留痕

```bash
# 原文提取（-layout 用于通读；-table 用于表格列对齐核对）
pdftotext -layout -enc UTF-8 paper.pdf ref176.txt
pdftotext -table  -enc UTF-8 paper.pdf ref176_tbl.txt

# 结构统计
pdfinfo paper.pdf | grep -i pages        # 72
grep -c "<!-- page .* of 72 -->" paper.md # 72
grep -c "^## " paper.md                   # 148（全为二级，无一级）
grep -o -E "Figure [0-9]+:|Table [0-9]+:" paper.md | sort -u | wc -l

# 逐页词元覆盖（md 按页标记切分，与原文逐页比对）
# 口径：小写化 + 连字归一 + 词元化后做集合差；结果 24504 缺 166 = 99.32%
```

> 说明：`pdftotext -layout` 在 Table 8（p13）、Table 28（p64）等宽表上会出现列错位，**不可**用它判定 md 的表格是否符合原文；本报告表格类结论均以 `-table` 提取 + PyMuPDF 词坐标二次确认。
