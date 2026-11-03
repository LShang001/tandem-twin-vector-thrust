# REF-177 转换核对报告（PDF ↔ MinerU Markdown）

> 审计对象：`paper.pdf`（u-blox F10 SPG 6.00 Interface description，UBX-23002975 R02，2024-06-13，181 页，protocol version 40.00）
> 转换产物：`paper.md`（MinerU，标准 tier；首次解析于 2026-11-03 19:38 报 `Local parse-server is not ready`，19:48:53 落盘 575,834 B，此后大小不变）
> 审计日期：2026-11-03　审计方式：`pdftotext -layout` 仅供检索定位 + pdfplumber 坐标级行重建 + PyMuPDF 页面渲染目视复核（不作为入库文本）
> 未改动 `paper.pdf` / `paper.md`；仅新增本报告。

---

## 一、结论

**可用（需修）**。

- **可用于阅读、检索与数值引用**：181 页全部带页标记且页号与 PDF 一致；315 条目录条目 100% 命中；67 个 UBX 消息小节全部有对应标题；91 个 `Table N:` 题注全部保留；CFG-MSGOUT 180 个 Key ID 100% 保留且"项名↔Key ID"配对正确；NAV-PVT 33 个字段偏移/类型/比例/单位全对；PROTVER、NAV-EOE、MON-VER 结构等关键事实无错误。
- **不可直接用于程序化解析表格**：存在表头列合并（`Key ID` 与 `Type` 并成一列，15 处）、`CK_A CK_B` 被拼成 `CK_ACK_B`、单元格内相邻文本缺空格等结构性降级；7 处图示被替换为不可解析占位符。
- **未发现臆造内容**：本固件**没有** `UBX-CFG-MSG (0x06 0x01)`、`UBX-MON-HW (0x0a 0x09)`、`CFG-PRT` 消息章节，markdown 中同样没有——这一点与任务清单的预设不同（见 §三.1），md 未擅自"补全"，值得肯定；后续修订**不得**反向把这些消息加进去。

---

## 二、覆盖率统计

| 指标 | PDF 基准 | paper.md 实测 | 覆盖率 |
|---|---|---|---|
| 页标记（`<!-- page N of 181 -->`） | 181 页 | 181 条，范围 1–181，页号与 PDF 正文页码一致 | **100%** |
| 目录条目（§1–§4 全部小节标题） | 315 条 | 315 条均可检索到 | **100%** |
| UBX 消息小节 | 67 个（ACK 2 / CFG 5 / INF 5 / MGA 9 / MON 9 / NAV 26 / RXM 4 / SEC 3 / TIM 3 / UPD 1） | 67 个均有 `## 3.x.y` 标题 | **100%** |
| 表题注 `Table N:` | 91 个（Table 1–91） | 91 个 | **100%** |
| CFG-MSGOUT Key ID | 180 个（p138–145） | 180 个，缺失 0 | **100%** |
| NAV-PVT 字段行 | 33 个（偏移 0…90） | 33 个，偏移/类型/比例/单位全对 | **100%** |
| 表格块 | 91 个带题注表（跨页续排） | 383 个表块（HTML 203 + 管道表 180） | 完整 |
| 图片/图示 | 2 个位图（p1 logo、p11 u-center 截图）+ 5 处矢量示意图 | 7 个 `![…](doc:a53f2ec/…)` 占位符，**无可解析图像** | **0%（图示内容丢失）** |
| 页眉/页脚 | 每页各 1 行 | 混入正文，约 899 行（见 §三 #21） | 噪声 100% 保留 |

**已知非缺陷（易误判为缺陷，已逐一目视复核）**

- p176/p169–178 配置默认值表：`pdftotext` 文本层出现"项名与 Key ID 错行"，**md 是正确的**（如 `CFG-QZSS-USE_SLAS_RAIM_UNCORR 0x10370007`、`CFG-RATE-MEAS 0x30210001`），与 PDF 版面一致（已渲染 p176 目视确认）。这是 PDF 自身文本层列序问题，不是转换错误。
- p85–p90 信号使能位表：`pdftotext` 把 `sig_gps_en_val` 断成 `sig_gps_en_` + `val`，**md 已正确拼接**，召回率低是度量假象。
- p11 截图 OCR 表中的 `PROTVER=34.10`、`ANTSTATUS=DONTKNOW`、`HW UBX 10 000S0000` 均**确为截图内文字**（已渲染 p11 目视比对），非臆造；正文表格的 `PROTVER=34.00` 亦完整保留。

---

## 三、逐项差异表

> 严重度：**高**=影响数值正确性；**中**=影响结构/可解析性/可引用性；**低**=格式与噪声。

### 3.1 任务清单项的适用性（先澄清基准，避免"照单抓药"）

| # | 页 | PDF 原文（基准） | 转换结果 | 判定 / 严重度 |
|---|---|---|---|---|
| 1 | p48 §3.8 | UBX 消息总表中 **UBX-CFG 类仅 5 条**：`UBX-CFG-CFG 0x06 0x09`、`UBX-CFG-RST 0x06 0x04`、`UBX-CFG-VALDEL 0x06 0x8c`、`UBX-CFG-VALGET 0x06 0x8b`、`UBX-CFG-VALSET 0x06 0x8a`——**无 `UBX-CFG-MSG (0x06 0x01)`** | md 同为 5 条；全文 `CFG-MSG` 0 命中、`0x06 0x01` 0 命中、`rateDDC/rateUART2/rateUSB` 0 命中 | **一致**。清单中的"CFG-MSG 8 字节 payload 端口顺序"在本固件**不适用**；端口速率由 CFG-MSGOUT 逐端口 Key 表达。严重度 —（说明项） |
| 2 | p50 §3.10.1 | "The behavior of this message has changed for protocol versions greater than 23.01. Use UBX-CFG-VALSET and UBX-CFG-VALDEL with the appropriate layers instead."——**主语是 UBX-CFG-CFG，不是 CFG-MSG** | md 保留该句 1 处；p51 "Old functionality of this message is not available in protocol versions greater than 23.01…" 亦保留 | **一致**。须防止把该注记误挂到 CFG-MSG 名下。严重度 低（误读风险） |
| 3 | p48 §3.8 | UBX-MON 类 **9 条**：MON-COMMS 0x0a 0x36、MON-GNSS 0x0a 0x28、MON-HW3 0x0a 0x37、MON-PATCH 0x0a 0x27、MON-RCVRSTAT 0x0a 0x40、MON-RF 0x0a 0x38、MON-RXR 0x0a 0x21、MON-SPAN 0x0a 0x31、MON-VER 0x0a 0x04——**无 `UBX-MON-HW (0x0a 0x09)`** | md 同为 9 条。`UBX-MON-HW` 仅在 `CFG-MSGOUT-UBX_MON_HW_{I2C\|SPI\|UART1}` 键名中出现（PDF p141 同样如此） | **一致**。严重度 —（说明项） |
| 4 | p134–135 §4.8 | 配置组总览含 `CFG-I2C`、`CFG-SPI`、`CFG-UART1`（及 INPROT/OUTPROT），**无 `CFG-USB`、无 UART2**；端口配置以 §4.9.x 配置组形式给出（p136 §4.9.4、p159 §4.9.21、p162 §4.9.26） | md 一致；MSGOUT 表亦只有 I2C/SPI/UART1 三端口 | **一致**。严重度 —（说明项） |

### 3.2 关键数值与结构

| # | 页 | PDF 原文（基准） | 转换结果（md） | 判定 / 严重度 |
|---|---|---|---|---|
| 5 | p1 / p12 | 封面 "Protocol version 40.00"；p12 表：`SPG 6.00 \| EXT SPGL1L5 6.00 (041e8a) \| 40.00` 与 `ROM SPGL1L5 6.00 (041e8a) \| 40.00` | `\| SPG 6.00 \| EXT SPGL1L5 6.00 (041e8a) \| 40.00 \|`（两行俱在） | **正确** |
| 6 | p42 §3.2 | 帧 = `0xb5 0x62` + class(1B) + ID(1B) + length(2B，仅 payload，U2 LE) + payload + `CK_A CK_B` | 逐条保留；`0xb5 0x62` 出现 98 次、`CK_A CK_B` 97 次 | **正确** |
| 7 | p143 §4.9.10 | `CFG-MSGOUT-UBX_NAV_PVT_UART1 0x20910007 U1`（同表 I2C `0x20910006`、SPI `0x2091000a`） | `\| CFG-MSGOUT-UBX_NAV_PVT_UART1 \| 0x20910007 \| U1 \| - \| - \| Output rate of the UBX-NAV-PVT message on port UART1 \|` | **正确** |
| 8 | p142 §4.9.10 | `CFG-MSGOUT-UBX_NAV_EOE_UART1 0x20910160 U1`（I2C `0x2091015f`、SPI `0x20910163`） | 同上，配对无歧义 | **正确** |
| 9 | p102 §3.14.11 | `UBX-NAV-PVT`，`0xb5 0x62 \| 0x01 \| 0x07 \| 92` | `0xb5 0x62 \| 0x01 \| 0x07 \| 92 \| see below \| CK_A CK_B` | **正确** |
| 10 | p102–105 | NAV-PVT 33 字段：0 iTOW U4 / 4 year U2 / 6 month / 7 day / 8 hour / 9 min / 10 sec / 11 valid X1 / 12 tAcc U4 / 16 nano I4 / 20 fixType / 21 flags / 22 flags2 / 23 numSV / 24 lon I4 1e-7 / 28 lat / 32 height I4 mm / 36 hMSL / 40 hAcc U4 / 44 vAcc / 48 velN I4 / 52 velE / 56 velD / 60 gSpeed I4 / 64 headMot I4 1e-5 / 68 sAcc U4 / 72 headAcc U4 1e-5 / 76 pDOP U2 0.01 / 78 flags3 X2 / 80 reserved0 U1[4] / 84 headVeh I4 1e-5 / 88 magDec I2 1e-2 / 90 magAcc U2 1e-2 | 33/33 全部命中，偏移、类型、Scale、单位一致；位域 `bit 0 <sup>U</sup>:1 invalidLlh`、`bits4…1 <sup>U</sup>:4 lastCorrection Age`、`bit 13 authTime`、`bit 14 nmaFixStatus` 均在 | **正确**（唯一瑕疵见 #16） |
| 11 | p97 §3.14.5 | `UBX-NAV-EOE`，Type **Periodic**，Length **4**，payload `0 \| U4 \| iTOW \| - \| ms` | `\| 4 \|`、`Type Periodic`、iTOW 行一致 | **正确** |
| 12 | p94 §3.13.9 | poll 长度 0；polled 长度 `40 + [0..n]·30`；`0 CH[30] swVersion`、`30 CH[10] hwVersion`、`40 + n·30 CH[30] extension` | HTML 表逐格一致，`40 + [0..n]·30` 原样保留 | **正确** |
| 13 | p56 §3.10.5 | `UBX-CFG-VALSET 0x06 0x8a`，payload `0 version U1 (0x01)`、`1 layers X1`、`2 transaction U1`、`3 reserved0 U1`、`4 + n cfgData`，长度 `4 + [0..n]` | 帧结构行 `0xb5 0x62 \| 0x06 \| 0x8a \| 4 + [0..n] \| see below \| CK_A CK_B` | **正确** |
| 14 | p54 §3.10.4 | `UBX-CFG-VALGET 0x06 0x8b`，poll `4 + [0..n]·4`（version 0x00 / layer / position U2 / keys U4）；"limited to containing a maximum of 64 key IDs" | 结构一致；"maximum of 64 key-value pairs" 3 处保留 | **正确** |
| 15 | p162 §4.9.26 | 6 列：`CFG-UART1-BAUDRATE 0x40520001 U4`、`STOPBITS 0x20520002 E1`、`DATABITS 0x20520003 E1`、`PARITY 0x20520004 E1`、`ENABLED 0x10520005 L`；**每行各自一句描述** | 5 个项名与 Key 全对，但**表结构 6 列压成 4 列**，且 STOPBITS 的描述被并入 BAUDRATE 单元格：`<td rowspan="2">The baud rate that should be configured on the UART1 Number of stopbits that should be used on UART1</td>` | **描述错行（行归属错）**　**中** |
| 16 | p104 | 字段名 `lastCorrection` 与 `Age` 是同一字段名的两行折行（实为 `lastCorrectionAge`） | `bits4…1 <sup>U</sup>:4 lastCorrection Age` | **字段名被拆开**　**低** |

### 3.3 表格结构降级与字符损坏

| # | 页 | PDF 原文 | 转换结果 | 判定 / 严重度 |
|---|---|---|---|---|
| 17 | p94、p120 | `Header \| Class \| ID \| Length (Bytes) \| Payload \| Checksum`；`… see below \| CK_A CK_B` | 表头合并为单格 `Header Class ID Length (Bytes) Payload Checksum`（4 处，页 94/105/120/128）；校验和写成 **`CK_ACK_B`**（2 处） | **字符合并 + 表头塌缩**　**中** |
| 18 | p138、p149、p162、p169–p178（13 页） | 6 列表头 `Configuration item \| Key ID \| Type \| Scale \| Unit \| Description(Default value)` | 15 处出现合并表头 `Key ID Type`（其中 3 处为 `Configuration item Key ID Type`），数据行也变成 `CFG-RATE-MEAS \| 0x30210001 U2 \| 0.001 \| s \| 1000`——Key ID 与 Type 落在同一列 | **结构降级，程序化解析会取到"键+类型"混合串**　**中** |
| 19 | p102 | 单元格内两行：`UBX-NAV-POSLLH` / `Geodetic position solution` | `UBX-NAV-POSLLHGeodetic position solution`（无分隔）；同类 `UBX-MON-VER Pollreceiverandsoftwareversion`、`UBX-NAV-EOEEnd of epoch` | **相邻文本粘连**　**中**（影响按消息名精确匹配与引用） |
| 20 | p2 Document information | `Document type \| Interface description`、`Subtitle \| Standard precision GNSS firmware`、`Revision and date \| R02 13-Jun-2024` | `Documenttype \| Interfacedescription`、`StandardprecisionGNSSfirmware`、`Revisionanddate` | **单元格内空格整体丢失**　**低** |
| 21 | p1–181 | 页眉 `u-blox F10 SPG 6.00 - Interface description`（180 次）、页脚 `UBX-23002975 - R02 C1-Public`（181 次）、`Page N of 181`（180 次）、章节面包屑如 `3 UBX protocol` / `Configuration defaults`（177 次）、logo 误识 `Cblox`（181 次） | 全部作为正文段落保留，合计约 **899 行**；页码/章节词因此可被误检索为正文 | **页眉页脚混入正文**　**低**（有页标记可剥离） |
| 22 | p102 等 | 句间正常空格：`…navigation epoch. See section…` | 70 处 HTML 单元格 + 55 处正文出现 `epoch.See`／`figures.Note` 形式（共 125 处 `.Xxx` 无空格） | **句间空格丢失**　**低–中** |
| 23 | p3–9 目录 | `3.2 UBX frame structure....42` | 保留点线 `3.2 UBX frame structure....42`（315 条） | **格式残留**　**低** |
| 24 | p97 vs p102 | `3.14.5.1 End of epoch`、`3.14.11.1 Navigation position velocity time solution` 同级 | 前者渲染为 `## 3.14.5.1 End of epoch`，后者为普通段落 | **标题层级不一致**　**低** |

### 3.4 图示与图文混排

| # | 页 | PDF 原文 | 转换结果 | 判定 / 严重度 |
|---|---|---|---|---|
| 25 | p130 §4.2 | 配置项结构图，图内标注 `size (3 bits) group (8 bits) ID within group (12 bits)`、`reserved (1 + 4 + 4 = 9 bits)` | `![Image block](doc:a53f2ec/tier:standard/page:130/block:9)`；`reserved (1 + 4 + 4 = 9 bits)` 保留，**`size (3 bits)` 与 `ID within group (12 bits)` 0 命中** | **图示丢失 + 图内标注丢失**　**中**（位域含义在正文列表 §4.2 中仍有等价表述，故未致命） |
| 26 | p13 / p17 / p22 / p132 / p133 / p179 | NMEA 帧结构图、GLL 无效数据示例图、配置层示意图、事务状态图、Related documents 页图 | 6 个 `![Image block]/![Chart block](doc:a53f2ec/…)` 占位符，**无任何可解析图像文件** | **图示全部不可用**　**中**（示意性内容，正文文字未丢） |
| 27 | p11 | 位图 u-center 截图：`$GNTXT,…,PROTVER=34.10*11`、`ANTSUPERV=*22`、`ANTSTATUS=DONTKNOW*2D`；正文表格 `PROTVER=34.00`、`ANTSUPERV=AC SD PDoS SR` | 截图被 OCR 成两张表（Time/Message 表、UBX-MON-VER 表）并与正文表并列；两套数值并存且无来源标注。OCR 细节：`GPS:GLO;GAL:BDS`（原文 `GPS;GLO;GAL;BDS`） | **忠实但易误引**　**低–中**（建议加"截图示例值"标注） |

---

## 四、抽页逐段对照（10 项，含键值表页）

| 抽检页 | PDF 段落/表格 | md 结果 | 结论 |
|---|---|---|---|
| **p48** §3.8 消息总表 | 67 条消息（跨 p47–49）+ 10 个类分组标题 | HTML 表 3 列（Message/Class-ID/Description），类分组用 `colspan=3` 保留；CFG 5 条、MON 9 条与 PDF 一致 | 通过（无漏行） |
| **p94** §3.13.9 MON-VER | poll 表 + polled 表（3 字段） | polled 表 HTML 逐格正确；poll 表退化为管道表：表头合并、`Pollrequest`、`Thismessagehasnopayload`、`CK_ACK_B` | 内容通过，结构不通过 |
| **p97** §3.14.5 NAV-EOE | 标题 + Type/Comment + 帧结构 + 1 字段 | Type `Periodic`、长度 `4`、iTOW U4/ms 均正确；`UBX-NAV-EOEEnd of epoch` 粘连 | 通过（1 处粘连） |
| **p102–105** §3.14.11 NAV-PVT | 4 页、33 字段、位域展开 | 33/33 偏移类型比例单位全对；位域 `<sup>U</sup>:1` 保留；`lastCorrection Age` 拆名 | 通过 |
| **p142–143** §4.9.10 MSGOUT | 每页 26 行键值（NAV 段） | 行序、项名、Key、Type、描述全对；`0x20910007`/`0x20910160` 定位准确；部分页表头塌缩为 `Key ID Type` | 内容通过，结构不通过 |
| **p162** §4.9.26 CFG-UART1 | 6 列 × 5 行 + 2 个常量表 | 5 个项名/Key 全对；列数塌缩为 4，STOPBITS 描述错挂到 BAUDRATE | 部分不通过 |
| **p172** 配置默认值 | 每页 ~15 行 `项名/Key/Type/Scale/Unit/默认值` | 项名↔Key↔默认值全对（与 p176 同源问题，md 正确） | 通过（表头塌缩） |
| **p176** 配置默认值 | QZSS/RATE/RINV 三张表 | 与版面逐格一致（`CFG-QZSS-USE_SLAS_RAIM_UNCORR 0x10370007`、`CFG-RATE-MEAS 0x30210001` 等均正确） | 通过 |
| **p130** §4.2 配置项结构 | 图 + 位域文字列表 | 图为占位符；`reserved (1+4+4=9 bits)` 保留；`size (3 bits)…` 丢失；位域列表 `Bits 30…28 / 23…16 / 11…0` 完整 | 部分不通过 |
| **p11–p12** 启动信息/协议版本 | 截图 2 张 + 表格 + 协议版本表 | 截图 OCR 表与正文表并存；p12 `SPG 6.00 → 40.00` 正确 | 通过（需标注） |

> 召回率辅助指标（8-gram 字符重合，去除空白与标记）：全篇均值 0.867、中位 0.893；最低页 p85 (0.383)、p176 (0.546)、p86 (0.565)。经逐页目视复核，**低分页均为 PDF 文本层自身折行/列序问题**（字段名折行、跨页续表），md 侧无实质缺失，故召回率不作为缺陷判据，仅用于定位可疑页。

---

## 五、建议修复项（按优先级）

1. **【中】修复表头塌缩**：将 `Key ID Type` 拆回 `Key ID` + `Type` 两列（涉及 p138、p149、p162、p169–p178 共 15 处，含 3 处 `Configuration item Key ID Type`）；将 `Header Class ID Length (Bytes) Payload Checksum` 拆回 7 列（p94、p105、p120、p128 共 4 处）。这是唯一会直接导致程序化取键失败的缺陷。
2. **【中】修复 `CK_ACK_B` → `CK_A CK_B`**（p94、p120 共 2 处）。
3. **【中】修复 p162 CFG-UART1 行归属**：把 `Number of stopbits that should be used on UART1` 从 BAUDRATE 的 `rowspan=2` 单元格移回 STOPBITS 行。
4. **【中】图示落地**：导出 7 处 `doc:a53f2ec/…` 图像块为 `images/` 并改为相对路径；至少补回 p130 图内位域标注 `size (3 bits) group (8 bits) ID within group (12 bits)`（可作为图注文字补写）。
5. **【中】拆开单元格内粘连**：在 `</td>` 内对"全大写消息名 + 首字母大写词"（如 `UBX-NAV-POSLLHGeodetic`、`UBX-MON-VERReceiver`）及 `[a-z]\.[A-Z]`（125 处）之间补空格；补齐管道表单元格内被整体吞掉的空格（p2 Document information 表为典型）。
6. **【低】剥离页眉页脚**：以 `<!-- page N of 181 -->` 为锚，删除 `Cblox`、`u-blox F10 SPG 6.00 - Interface description`、`UBX-23002975 - R02 C1-Public`、`Page N of 181` 及章节面包屑行（约 899 行）。
7. **【低】统一子标题层级**：`3.x.y.z` 四级子标题一律 `###`（现 `3.14.5.1` 为 `##`、`3.14.11.1` 为段落）。
8. **【低】p11 截图段落加来源标注**（如 `> 下列两表为 u-center 截图 OCR，示例值 PROTVER=34.10，与正文表格 PROTVER=34.00 不同`），避免误引为固件协议版本。
9. **【低】目录去点线**：把 `标题....页码` 改为 `标题`（页码由页标记承担）。
10. **【低】字段名折行拼接**：`lastCorrection Age` → `lastCorrectionAge`；同类需检查被行宽拆开的字段名。
11. **【禁项】不得反向"补全"**：不得因清单提到而新增 `UBX-CFG-MSG (0x06 0x01)`、`UBX-MON-HW (0x0a 0x09)`、`CFG-PRT` 消息章节或 `rateDDC/rateUART2/rateUSB` 字段——本固件（F10 SPG 6.00，protocol 40.00）不存在这些消息，端口速率一律经 `CFG-MSGOUT-<MSG>_<PORT>`（仅 I2C/SPI/UART1）。

---

## 六、审计方法可复现说明

```bash
# 1) 提取（仅作核对基线，不作入库文本）
pdftotext -layout -enc UTF-8 paper.pdf /tmp/ref177.txt     # 13186 行 / 692,987 B / 182 个分页块
# 2) 坐标级行重建（还原表格真实列位，规避 pdftotext 列串行）
python rows.py paper.pdf <page>                            # pdfplumber extract_words + y 聚类
# 3) 页级比对（页标记切分 + 目录覆盖 + 键值集合 + 页眉页脚统计）
python compare.py paper.md expected.json toc.json result.json
# 4) 关键表目视复核
PyMuPDF get_pixmap(dpi=110~130) → 与 md 表逐格对照（p11 / p176 / p130）
```

> `rows.py` / `compare.py` 为本次审计的临时辅助脚本（未纳入仓库，逻辑见上述注释；`expected.json`/`toc.json` 由 PDF 现场抽取生成）。

- 目录基线：解析 PDF p3–9 目录得 315 条（第 1 章 11、第 2 章 70、第 3 章 194、第 4 章 40）。
- 键值基线：p138–145 去重得 180 个 `0x209…` Key ID；md 命中 180/180，且无"同名不同键"冲突。
- 页标记与 PDF 页码一致性：抽查 p48/p94/p97/p102/p104/p105/p142/p143/p162/p172/p176 全部对齐。
- 同目录 `manifest.json` / `notes.md` 由同批另一任务生成（19:49），不在本次核对范围内。
