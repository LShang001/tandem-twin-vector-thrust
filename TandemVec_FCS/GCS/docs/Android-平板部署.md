# 平板 / Android 部署：三条路与各自的前提

> 结论先说：**当前架构（Python 后端 + Web 前端）里，「平板独立运行 + 平板自己插数传」
> 需要平板能跑 Python 后端**。Android 上这条路存在（Termux），但**我没有 Android
> 设备，无法实机验证**——下面把它需要什么、卡在哪写清楚，不含未验证的承诺。

---

## 路 1：浏览器直连（已验证 ✅）

平板什么都不用装，浏览器打开后端地址即可。

```bash
# 在跑后端的机器上
cd TandemVec_FCS/GCS && py -3.12 app.py --lan
# 终端会打印：http://192.168.0.208:8091/  等地址
```

平板浏览器输入该地址 → 完整界面。**串口/链路都在后端那台机器上**，平板只做显示与操作。

实测数据（本机 192.168.0.208）：HTTP 200、WebSocket 101 并持续收 telemetry、
前端 JS/CSS 均 200、`/api/caps` 正常。

---

## 路 2：装到主屏（PWA，组件已就绪 ✅ / 离线缓存受限）

`web/` 下已有 `manifest.webmanifest`、`sw.js`、`icon-192/512.png`，`index.html`
已引用 manifest 并注册 Service Worker。线上实测四个资源均 200。

**怎么装**：平板 Chrome 打开地址 → 菜单「添加到主屏幕」→ 桌面出现独立图标，
点开是全屏无地址栏（`display: standalone`）。

**已知限制（不是 bug，是浏览器安全模型）**：
Service Worker 只在 **secure context**（`https://` 或 `localhost`）生效。
局域网 `http://192.168.x.x:8091` 不是 secure context，浏览器会拒绝注册 SW。
所以：

| 能力 | 局域网 HTTP | localhost / HTTPS |
|---|---|---|
| 添加到主屏、全屏独立窗口 | ✅ | ✅ |
| 断网仍能打开界面（离线壳） | ❌ | ✅ |

要在局域网获得完整 PWA，需要给后端加 HTTPS（自签或反向代理）。当前未做。

---

## 路 3：平板独立运行（Termux + Python 后端）

**这是唯一能让「平板自己插数传」的路径**，因为：

- Android 上普通 App **不能直接打开 `/dev/ttyUSB0`**，必须走 Android USB Host API；
- 现成的 APK 壳（Capacitor/Cordova）里 WebView **不支持 Web Serial API**，
  拿不到串口；
- 而协议解析（AnoCom 帧、MAVLink 状态机）现在全在 Python 后端里
  （`anocom.py` 462 行 + `mavlink_service.py` 513 行），没有前端 JS 版本。
  所以纯前端 APP 也解析不了数据。

于是只有「平板跑 Python 后端」一条路。具体：

```bash
# 1) 装 Termux（必须从 F-Droid 装，Play 商店版已停更）
# 2) 基础环境
pkg update && pkg install python python-numpy termux-api
#    ★ numpy 必须用 pkg 装的版本：pip 装 numpy 在 Termux 上没有预编译 wheel，
#      会尝试源码编译（几十分钟且常失败）。spectrum/stats 依赖 numpy。
pip install fastapi uvicorn pyserial
# 3) 起服务（只在平板本机，浏览器开 127.0.0.1:8091）
cd TandemVec_FCS/GCS && python -m uvicorn server.main:app --host 127.0.0.1 --port 8091
```

**插数传（OTG）**：普通 pyserial 打不开 `/dev/ttyUSB0`（无权限）。需要
`termux-usb` 把设备 fd 转进来：

```bash
termux-usb -l                              # 列出 USB 设备
termux-usb -r -e ./start-gcs.sh /dev/bus/usb/001/002
#   -e 会执行命令并把 fd 号作为最后一个参数传入（$1）
```

`start-gcs.sh` 里把 fd 交给地面站（当前需要手动触发一次 connect）：

```bash
#!/data/data/com.termux/files/usr/bin/bash
FD="$1"
cd ~/TandemVec_FCS/GCS
python -m uvicorn server.main:app --host 127.0.0.1 --port 8091 &
sleep 3
# 让后端用这个 fd 作为链路（HTTP 调试接口就是为这种场景准备的）
curl -s -X POST http://127.0.0.1:8091/api/cmd \
     -H 'Content-Type: application/json' \
     -d "{\"cmd\":\"connect\",\"port\":\"fd:$FD\",\"protocol\":\"anocom\"}"
```

**当前状态：链路类已实现并通过逻辑测试（8 项），真实 USB 设备未验证。**

已落地：

- `server/termux_usb_link.py` 的 `TermuxFdLink`——与 `SerialLink` 同接口的 fd 链路；
- `open_link()` 支持 `fd:N` 端点，`connect` 命令传 `port=fd:3` 即走它；
- 测试 `tests/test_termux_usb_link.py`（8 项，用管道验证，不依赖 Android）：
  读、写、双向、无效 fd 立即报错、写前校验、关链不误关借来的 fd、
  可选的 `close_fd=True`、`open_link` 分派。

实现上两个必须知道的点：

1. **fd 必须设成非阻塞**。阻塞的 `os.read` 会一直挂在读线程里，`close()` 的
   stop 标志它看不到，`join` 只能靠超时返回、线程和 fd 都悬着——实测会让
   pytest 卡死（改成非阻塞轮询后 8 项 0.25 s 跑完）。
2. **`close()` 默认不关 fd**。fd 的属主是 `termux-usb` 的父进程，我们只是借用，
   关掉它会让 termux-usb 侧的设备句柄状态变脏。

**fd 之后的整条链路已端到端验证**（`tests/test_usb_fd_e2e.py`，3 项）：

- 从 fd 写入真实 MAVLink 字节 → 后端收下 3030 字节 → 解析出 `roll=166.16`、
  `lat=28.23` 等具体字段 → 快照可用；
- UDP 端点同样端到端：模拟飞控发 15 组帧 → 后端学会对端并解析出 28 个字段；
- `g.link.info()` 正确报告 `kind=termux-usb-fd`。

这说明「termux-usb 把设备 fd 交给我们」**之后**的所有环节都是通的——
剩下的唯一未知是 termux-usb 本身在你那台平板上能否成功授权并读到数传字节，
那是 USB 硬件/系统权限层，必须在真机上确认。

**一键脚本**：`android/termux-setup.sh`

```bash
bash termux-setup.sh              # 装依赖 + 自检（pkg 装 numpy，pip 装 fastapi/uvicorn/pyserial）
bash termux-setup.sh --run        # 起服务，浏览器开 127.0.0.1:8091
bash termux-setup.sh --usb        # 列出 USB 设备 → 指定后授权并自动接上数传
```

`--usb` 的内部流程（也是 termux-usb 的用法要点）：脚本用
`termux-usb -r -e "$0" "$USB_DEV"` 重新调用自己，termux-usb 把 **fd 号作为最后
一个参数**追加；脚本识别到纯数字参数就走「起后端 → POST /api/cmd 让它用
`port=fd:<号>` 连接」这条，然后打开浏览器。

**为什么服务只绑 127.0.0.1**：平板自己就是后端，浏览器访问 `localhost` 是
secure context，PWA 的 Service Worker 才能生效；而且不把控制面暴露到网络。

**启动即接链路的通用参数**（不限于 USB，`app.py`）：

```bash
py -3.12 app.py --link udp:14550             # 启动就监听 UDP 并自动连
py -3.12 app.py --link COM5 --proto anocom   # 启动就连串口
py -3.12 app.py --usb-fd 7                   # Termux：用 termux-usb 给的 fd
```

风险与前提：

- 需要真平板 + OTG 线 + 数传模块，**端到端只能实机验证**；
- `termux-usb -r` 会弹系统授权框，每次插拔都要授权一次；
- 平板休眠/切后台时 Termux 可能被系统杀掉，长时任务要 `termux-wake-lock`。

---

## 为什么不在 Windows 上直接打 APK

| 方案 | Windows 上能否构建 | 能否平板插串口 |
|---|---|---|
| Capacitor / Cordova 壳 | 能（本机 JDK17 + Android SDK 已具备） | ❌ WebView 无 Web Serial，且解析在后端 |
| Buildozer / python-for-android（含 Python 解释器） | **不能**（只支持 Linux/macOS，需 WSL） | 理论可以，但 FastAPI+uvicorn 在 Android 上很重 |
| Termux 直接跑（路 3） | 不需要构建 | ✅（经 termux-usb） |

所以「出个 APK」和「平板能插数传」是两件事：APK 壳出得来但不解决串口；
解决串口的那条路（Termux）根本不需要打 APK。

---

## 建议的推进顺序

1. 先用路 1 把飞控跑通（平板只当显示端，串口在电脑上）；
2. 若飞控侧有 WiFi 数传：让飞控直接发 UDP，后端用 `udp:14550` 收
   （`server/udp_link.py` 已实现并测过），平板仍走路 1；
3. 只有确实需要「平板单独拎着走、不依赖电脑」时，才投入路 3，
   并且必须先在真平板上跑通 `termux-usb` 那一步再补代码——否则写了也是未验证代码。
