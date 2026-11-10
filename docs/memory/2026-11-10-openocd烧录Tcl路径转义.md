> **触发条件**：跨机烧录固件、用 openocd `-c` 传绝对路径、写 PowerShell 启子进程脚本时报 `couldn't open ...` 或 `编程失败` 时必读。

# 2026-11-10 openocd 烧录：Tcl 反斜杠转义坑（三次烧录失败的真正根因）

> 背景：共轴 42688 固件已编译好、elf 已在目标机、openocd 能识别 DAP，但连续三次
> 报 `** Programming Failed **`。前两次误判为「PowerShell 引号问题」，
> 第三次看到 openocd 自己的报错才定位。

## 1. 根因：openocd `-c` 的字符串由 Tcl 解析，反斜杠是转义符

`-c "program <elf> verify reset exit"` 里那段字符串不是普通 C 字符串，
openocd 把它交给 **Tcl** 解释器 eval。Tcl 双引号内反斜杠按转义序列处理：

| 输入片段 | Tcl 解释结果 | 后果 |
|---|---|---|
| `\U` | `U` | `C:\Users` → `C:Users` |
| `\1` | 八进制 escape 起始 | `\126` = 八进制 126 = `V` |
| `\f` | 换页符 0x0C | 路径里插入不可见字符 |

实测报错原文（路径 `C:\Users\12631\fw42688.elf`）：

```
Error: couldn't open C:UsersV31w42688.elf
```

`C:\Users\12631\fw42688.elf`
→ `C:` + `U`(来自 `\U`) + `sers` + `V`(来自 `\126`) + `31` + `\f`换页符 + `w42688.elf`

**完全对得上。所以不是引号没转义，是反斜杠被 Tcl 吃掉了。**

## 2. 正确做法

首选：**传正斜杠路径给 openocd**（Tcl 里 `/` 无特殊含义，Windows 内核同样接受）。

```powershell
$elfTcl = $elfWin.Replace('\', '/')      # C:/Users/12631/fw42688.elf
$cmd = 'program "' + $elfTcl + '" verify reset exit'
& $ocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg -c $cmd
```

备选：Tcl 花括号阻止全部替换 —— `program {C:\Users\...\fw42688.elf} verify reset exit`。
但花括号与 PowerShell 的引号嵌套容易再出事，**正斜杠是唯一稳的写法**。

成功输出特征：`** Programming Finished **` → `** Verify Started **` → `** Verified OK **`。

## 3. 同时踩到的 PowerShell 坑

- **`Start-Process` 的 `-NoNewWindow` 与 `-WindowStyle` 互斥**：同时给两个参数直接抛
  `InvalidOperationException`，`-PassThru` 返回 `$null`，后续 `$proc.WaitForExit()` 报
  `不能对 Null 值表达式调用方法`。后台读串口 + 复位触发这种组合要么只用 `-NoNewWindow`
  拿 stdout 重定向，要么只用 `-WindowStyle Hidden` 且不要 `-PassThru`。
- **内联 `powershell -Command "...\"...\""` 多层引号必然出事**：写成 `.ps1` 文件 → `scp` →
  `powershell -ExecutionPolicy Bypass -File` 执行，比任何内联转义都可靠。
- 目标机 python 是 `C:\Python314\python.exe`（3.14），不是 `py -3.12`；
  写脚本前用 `Get-Command python` 探测，别假设。

## 4. 排查顺序（下次直接照这个走）

1. openocd 能出 `SWD DPIDR` / `Examination succeed` → 调试器与目标板链路正常，问题在命令串。
2. 报 `couldn't open <路径>` → **先把 openocd 打印出来的路径与真实路径逐字符比对**，
   差异在哪就是哪个转义被吃了。这一步能在 30 秒内定位，比反复改引号快一个数量级。
3. 报 `program_error` 但无 `couldn't open` → 才去看 elf 是否存在、是否被 `Move-Item` 挪走。
4. 烧完必须抓一次启动日志（`[Init] IMU:` / `System Setup Complete`）才算完成——
   `Verified OK` 只证明字节写进了 Flash，不证明固件是新的、更不证明它能启动。
