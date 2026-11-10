# tools/fw_flash —— 跨机烧录与实机抓日志

> **触发条件**：要把 revo 编译出的 `firmware.elf` 烧到另一台只读节点、或烧完要抓启动日志时用这套脚本。

跨机烧录流程（铁律见根 `AGENTS.md` §多会话并发与变更联动规范）：代码与编译留在 revo，
另一台机器只负责「把 elf 写进 Flash + 抓一次启动日志」。

## 脚本

| 文件 | 作用 |
|---|---|
| `flash_elf.ps1` | 用 openocd 烧 elf（已在 Flash 节点上执行）。路径自动转正斜杠。 |
| `capture_boot.ps1` | 先起串口读取、再 openocd 复位，抓完整启动日志。 |
| `read_serial.py` | 读某串口 N 秒原始字节（抓日志 / 确认数口有流）。 |

典型用法（在 Flash 节点上，脚本与 elf 都在用户目录）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File flash_elf.ps1 C:\Users\<me>\fw45686.elf
powershell -NoProfile -ExecutionPolicy Bypass -File capture_boot.ps1 COM12 14
```

从 revo 远程驱动：`scp` 过去再 `ssh <node> "powershell ... -File C:\Users\<me>\flash_elf.ps1 <elf>"`。

## 三条硬约束（都是实际踩过的）

1. **elf 路径必须正斜杠**。openocd 的 `-c` 字符串由 **Tcl** 解析，反斜杠是转义符：
   `\U`→`U`、`\126`→八进制 `V`、`\f`→换页符，于是
   `C:\Users\12631\fw.elf` 变成 `C:UsersV31<FF>w.elf`，报
   `couldn't open ...` + `Programming Failed`。脚本里已自动转换，**不要手工拼反斜杠**。
   详见 `docs/memory/2026-11-10-openocd烧录Tcl路径转义.md`。
2. **`.ps1` 必须纯 ASCII**。Windows PowerShell 5.1 把无 BOM 的 UTF-8 当 ANSI/GBK 读，
   中文注释会变乱码并可能破坏后续行解析（实测：`$elfTcl` 静默变空，命令成了 `program ""`）。
   需要中文说明就写进本 README，不要写进 `.ps1`。
3. **`Verified OK` ≠ 固件能启动**。必须再抓一次 boot log，确认出现
   `[Init] IMU: <型号> ...Hz`、`Onboard IMU initialized successfully`、
   `System Setup Complete. Starting main loop...` 三行才算烧成功。

## 端口归属别搞混

| 口 | 固件侧 | 用途 |
|---|---|---|
| USB-C 调试口 | `Serial8`（UART8，PE0/PE1） | `[NAVP]`/`[Init]` 等调试文本 → 抓启动日志用这个 |
| 数传 | `Serial6`（USART6，PC7/PC6） | AnoCom 二进制遥测 + **DBG 控制台**（`DBG\n` 进入） |

发 DBG 命令走「数传」口（`tools/dbg_console.py`），抓启动日志走「USB-C」口（本目录 `read_serial.py`）。
发错口只会看到文字、没有命令响应。
