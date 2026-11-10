# GCS 扩展指南 —— 怎么加功能、怎么被调用、怎么验证

> 目标：地面站的每个能力都能被**人点、脚本调、agent 调**，三者走同一条代码路径。
> 想加东西时照这份做，不需要先读懂 `server/main.py` 的一千八百行。

---

## 1. 三层结构

```
链路层   server/serial_link.py  串口        ┐
         server/udp_link.py     UDP         ├ 同一个接口：connect/is_open/write/close + on_data
         server/udp_link.py     open_link() ┘ ← 唯一的链路选择入口（按 udp: 前缀分派）
              ↓ 字节流
命令层   server/main.py          GCS 类：解析遥测、维护快照与缓冲、46 条命令
         server/caps.py          能力清单（模块 / 命令 / 参数 / 只读标记）★ 唯一登记处
              ↓ JSON over WebSocket / HTTP
前端层   web/src/*.mjs           一页一个模块，无打包器，ES module 直接加载
```

**关键设计**：命令层不知道底层是线还是网（链路层负责），前端不知道数据来自哪个模块
（命令层归一成同一套字段）。所以加一台设备、加一种传输、加一个页面，都只动一层。

---

## 2. 加一条命令（三步，缺一不可）

以加 `battery_detail` 为例。

### ① 登记：`server/caps.py` 对应模块的 `cmds` 里加一条

```python
_c('battery_detail', '电池详细状态：单体电压与温度',
   [_a('cell', 'n', '单体序号，省略=全部')], 'battery_detail')
```

- `readonly=True`（默认）= 只读查询；**会改飞控或改本机状态的必须写 `readonly=False`**，
  自动化工具据此决定能不能批量跑。
- 参数必须真的被该分支读取，否则守卫会报"声明了但没用"。

### ② 实现：`server/main.py` 的 `handle_ws_message` 加一个 `elif`

```python
elif cmd == 'battery_detail':
    cell = msg.get('cell')
    await ws.send_json({'type': 'battery_detail', 'cells': _batt(cell)})
```

分支里只做三件事：读参数、算/取数据、`ws.send_json` 回一条带 `type` 的消息。

### ③ 验证

```bash
py -3.12 tools/check-gcs-caps.py                       # 清单 ⇄ 实现 一致性（0 问题才算过）
py -3.12 tools/gcs_ctl.py demo start                   # 无硬件也能验
py -3.12 tools/gcs_ctl.py cmd battery_detail cell=2
py -3.12 tools/gcs_ctl.py demo stop
```

**加命令时不要**：在 main.py 里另写一套参数解析（用 `msg.get` 就地取）；
不要直接 `print` 结果（用 `send_json`，否则 API 与界面都收不到）。

---

## 3. 加一种链路 / 换个传输

`server/udp_link.py` 的 `open_link()` 是**唯一**的选择入口：

```python
def open_link(port, baud, on_data, read_size=256):
    if port.startswith('udp:'):  return UdpLink(...)
    if port.startswith('tcp:'):  return TcpLink(...)   # ← 就这样加
    return SerialLink(...)
```

新链路类只实现五个方法 + 一个回调，与 `SerialLink` 完全同形状：

```python
class XxxLink:
    port, baud                    # 属性（幂等判断与界面显示用）
    is_open                       # 属性
    def connect(self)             # 建连并起读线程
    def write(self, data: bytes)  # 发送
    def set_read_size(self, size) # 读粒度（DBG 导出会调大）
    def close(self)               # 幂等
    def info(self) -> dict: ...   # 自描述（/api/snap 的 link 字段）
```

`on_data(bytes)` 回调在**读线程**里被调用，回空字节串表示链路异常（与串口同语义）。

**当前支持**：

| 端点写法 | 含义 |
|---|---|
| `COM5` / `/dev/ttyUSB0` | 串口（波特率取 `baud`） |
| `udp:14550` | 监听 14550，**自动学习对端**（飞控主动推遥测时不用填它 IP） |
| `udp:14550@192.168.1.100:14550` | 监听 14550 并固定发往该地址 |

UDP 无对端时 `write()` **丢弃并计数**（`link.info()['dropped']`），不抛异常——
否则"先改参数再等遥测"这种操作会直接失败。

---

## 4. 加一个前端页面（三处注册 + 一个模块文件）

1. 新建 `web/src/xxx.mjs`，导出 `export function activate()`（页面激活时调用）
   和 `export function deactivate()`（可选，离开时清理定时器/rAF）
2. `web/src/main.mjs`：`const pages = [...]` 加 `'xxx'`
3. `web/src/main.mjs`：`PAGE_LOADERS` 里加 `xxx: () => import('./xxx.mjs')`
4. `web/index.html`：导航加 `<button class="nav-btn" data-page="xxx">`，
   并加 `<section class="page" id="page-xxx">`

数据获取统一走两入口：`bus.addEventListener('telemetry', ...)`（50 Hz 推送）
或 `send({cmd: '...'})`（请求-应答）。**不要自己 `new WebSocket`**。

布局页（仪表盘）的面板受用户配置管理，加面板要同时在 `web/src/layout.mjs` 的
`PANELS` 里登记 `{id, sel, name}`，否则显隐/排序对它无效。

---

## 5. 调试与自动化接口

### HTTP（给脚本/agent）

| 方法 | 路径 | 用途 |
|---|---|---|
| GET | `/api/caps` | 能力清单（模块/命令/参数/只读标记） |
| GET | `/api/snap` | 当前遥测快照（一次性，不订阅） |
| POST | `/api/cmd` | 执行任意命令，同步返回它的回复消息 |
| GET | `/api/status` | 后端与链路统计 |

```bash
curl -s http://127.0.0.1:8091/api/caps | py -3.12 -m json.tool | head -40
curl -s -X POST http://127.0.0.1:8091/api/cmd \
     -H 'Content-Type: application/json' \
     -d '{"cmd":"spectrum","channel":"roll_deg"}'
```

`/api/cmd` **不新增业务逻辑**：它把你的参数原样交给 `handle_ws_message`，
再用一个收集器把 `send_json` 的消息攒成数组返回。所以界面能做的事，
API 一定能做，且行为逐字一致。

### CLI（给人和 agent，一条命令搞定）

```bash
py -3.12 tools/gcs_ctl.py caps                        # 有什么能力
py -3.12 tools/gcs_ctl.py modules                     # 模块 + 命令名一览
py -3.12 tools/gcs_ctl.py snap --keys roll_deg,pitch_deg
py -3.12 tools/gcs_ctl.py cmd scope_stats channel=roll_deg
py -3.12 tools/gcs_ctl.py cmd spectrum_multi 'channels=["roll_deg","pitch_deg"]'
py -3.12 tools/gcs_ctl.py demo start | stop           # 起停演示后端（无硬件验证）
py -3.12 tools/gcs_ctl.py selftest                    # 端到端自检（会自己起停后端）
```

参数写 `key=value`，值优先按 JSON 解析（`cell=2` → 数字，`file=a.csv` → 字符串，
`channels=["a","b"]` → 数组）。

---

## 6. 验证矩阵（改完照跑）

| 改动范围 | 必跑 |
|---|---|
| 加/改命令 | `tools/check-gcs-caps.py` + `gcs_ctl.py selftest` + `pytest tests/ -q` |
| 改链路层 | `pytest tests/test_udp_link.py -q`（真实 socket 回环） |
| 改前端 | 浏览器实测 + `tools/tablet_check.py`（平板视口/触控/死空白） |
| 提交前 | `py -3.12 tools/verify-all.py`（27 项快档，含 GCS 单测与冒烟） |

---

## 7. 已知边界（不要在这些地方浪费时间）

- **MAVLink 任务（MISSION_*）**：固件对所有任务帧回 `MAV_MISSION_UNSUPPORTED`，
  地面站侧上传接口已就绪但飞控不落地，属固件侧待实现。
- **远程解锁/切模式**：固件安全拒绝（解锁与模式由 RC CH5 硬判定），
  `MAV_CMD_COMPONENT_ARM_DISARM`/`DO_SET_MODE` 会回 ACK 但不执行。这是设计红线。
- **参数持久化**：固件 `param save` 只回校验帧、不写 Flash，重启回默认值。
  需要保存就用 `param dump` 落盘、下次 `param load` 重放。
- **AnoCom 专有帧**（0xF1 摆角/0xF2 变量/DETA100 状态）在 MAVLink 链路上不存在，
  界面对应面板会显示"本链路不上报"——这是正确的降级，不是 bug。

---

## 8. 局域网 / 平板访问

**默认只绑 `127.0.0.1`**（不主动把飞控控制面暴露到网络）。要让平板、手机、
另一台电脑打开同一个界面，加 `--lan`：

```bash
py -3.12 app.py --lan          # 绑 0.0.0.0，启动时打印所有可访问地址
py -3.12 app.py --host 0.0.0.0 # 等价写法（也可指定某个网卡地址）
```

启动后终端会列出形如下面的地址，平板浏览器直接打开即可（同一 WiFi/网段）：

```
[GCS] 局域网访问已开启，平板/手机用下面任一地址打开（同一 WiFi/网段）：
         http://192.168.0.208:8091/
         http://10.8.0.2:8091/          <- WireGuard 等虚拟网卡也会列出
```

实测（2026-11-10，本机 192.168.0.208）：HTTP `/api/caps` 返回 200、
WebSocket `/ws` 返回 `101 Switching Protocols` 并持续收到 telemetry 流，
前端 JS/CSS 静态资源均 200。**不存在额外的跨域或协议限制**——前端与后端
同源，WebSocket 走同端口同源升级。

注意事项：

- **Windows 防火墙**：本机三个 profile 均为关闭状态，故无需放行。若你的机器
  开了防火墙，需要放行 Python 的入站 TCP（或该端口），否则平板连不上。
- **安全**：开启 `--lan` 后，局域网内任何设备都能看到遥测并能执行连接/改参数
  等操作（`/api/cmd` 不设鉴权）。仅在可信网络使用；不可信网络请保持默认的
  `127.0.0.1` 并用本机窗口。
- **这不是"装 APP"**：这是浏览器直连后端，平板不需要安装任何东西。要真正
  离线独立运行（含平板自己插数传读串口），属于 Android 封装范畴，见下一节。

---

## 9. 多平台（Android 平板）的三条路

| 方案 | 平板需要什么 | 串口 | 说明 |
|---|---|---|---|
| A. 浏览器直连后端 | 什么都不装 | 由后端那台机器接串口 | 就是上一节，最简单，`--lan` 即可 |
| B. PWA 装到主屏 | 装成图标（全屏、有图标） | 同上（仍连后端） | 体验像 APP，实际仍是网页；离线只有壳 |
| C. 原生 APK（Capacitor） | 安装 APK | 平板 OTG 插数传，走 Android USB 串口 | 需要原生 USB 串口插件；后端逻辑要么随之移植、要么仍连局域网 |

**当前状态**：A 已实现并实测；B 需补 `manifest.json` + Service Worker；
C 需要 Gradle/Capacitor 工程与真机构建（本机 JDK 17 与 Android SDK 已具备，
但没有 USB 串口插件，平板插数传这条还没打通）。

**飞控侧配合**：走局域网时飞控必须能发 UDP——`server/udp_link.py` 已支持
`udp:14550`（自动学习对端）与 `udp:14550@<飞控IP>:<端口>`（固定对端）。
飞控侧若是串口数传，需要数传本身支持 UDP 透传（如某些 WiFi 数传模块），
或者用一台小主机做串口与 UDP 之间的桥。
