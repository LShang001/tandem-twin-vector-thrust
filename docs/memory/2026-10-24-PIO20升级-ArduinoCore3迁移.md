# 2026-10-24 PlatformIO 20 / Arduino Core 3.0 升级经验

## 1. 升级范围

- PlatformIO Core：6.1.19 → 6.2.0（SCons 4.11.1、Windows 单测修复、`pio run -p`）。
- ststm32：19.7.1 → 20.0.0。
- framework-arduinoststm32：4.21200.0 → 4.30000.0（Arduino Core 2.12 → 3.0）。
- GCC 工具链保持 12.3.1；CMSIS 2.60300.0、CMSIS-DSP 1.16.2、OpenOCD 3.1200.7 不变。

## 2. 工具链/网络踩坑

1. `pio upgrade` 在 Windows 上可能因 `pio.exe` 被占用中断（WinError 32），随后 `pio` 报
   `No module named 'platformio'`。恢复命令：
   `C:\Users\12631\.platformio\penv\Scripts\python.exe -m pip install -U platformio==6.2.0`。
   安装后 `pio --version` 应为 6.2.0；`~latformio` 的 invalid distribution 警告是中断残留。
2. PlatformIO 包下载必须走代理；当前 shell 未继承 `HTTPS_PROXY` 时会卡在
   `Resolving ... dependencies`。使用：
   `eval "$(proxy env --shell bash)"` 后再执行 `pio pkg install`；长命令放后台。
   代理端口为 `127.0.0.1:7897`，`proxy status` 可确认出口与进程。
3. 不要同时启动多个 `pio pkg install`；会争抢包锁/重复下载。发现多个 `pio.exe` 时先停掉重复进程。
4. `pio pkg list` 在 framework 尚未安装完时只显示 CMSIS/工具链，不显示
   `framework-arduinoststm32`；以 `pio pkg install` 结束后的列表为准。

## 3. Arduino Core 3.0 固件迁移

1. 具体串口类由 `HardwareSerial` 改名为 `Uart`；`arduino::HardwareSerial` 变成抽象基类。
   项目 `src/include` 内 `HardwareSerial` 共 77 处、13 个文件，必须迁移为 `Uart`。
2. `availableForWrite()` 只在 `Uart` 中，不在 `HardwareSerial` 基类中。凡是可能调用它的
   引用/函数参数（尤其 `opticalFlowSerial`、`debugFlashCommand`、`handleDebugConsole`）
   必须是 `Uart&`，不能继续用 `HardwareSerial&`。
3. `Print` 进入 `arduino::` 命名空间；`task_scheduler.h` 的全局 `class Print;` 会与
   `using namespace arduino;` 产生二义性（g++ 报 `reference to 'Print' is ambiguous`）。
   删除前置声明，签名写 `arduino::Print&`。
4. Core 3.0 的 `Serial1..Serial8` 声明为 `extern Uart`；项目自定义同名对象必须用 `Uart`，
   否则 `mavlink_bridge.cpp` 的本地 `extern HardwareSerial Serial6;` 等会类型冲突。
5. 当前 WeAct H743 variant 的 `SERIAL_UART_INSTANCE=101` 只让核心定义 `SerialLP1`，
   不会与项目自定义 `Serial1..8` 重复；仍需在首次编译时确认。
6. `lib/CrsfSerial` 和 `lib/ublox-main` 只使用 `HardwareSerial` 的基类引用/指针，
   传入 `Uart&`/`Uart*` 可向上转型，无需改库；不要为了统一而改 vendor 接口。
7. 宿主 SIL 替身要同步：
   `test_host/fw_sil/stub/Arduino.h` 增加 `using Uart = HardwareSerial;` 和
   `namespace arduino { using ::Print; }`，否则 `state_data.cpp` 与 `task_scheduler` 无法编译。
   `test_host/fw_sil/stub/SPI.h` 还需补 `<cstddef>`（`size_t`）。
8. Core 3.0 移除了 `SPIClass::setDataMode()`；DPS310 的 `DpsClass.cpp` 不再调用它，
   SPI 模式由每次 `beginTransaction(SPISettings(..., SPI_MODE3))` 应用。

## 4. 工作流经验

- 主工作树有大量未提交改动时，用独立 `git worktree` 从 HEAD 建升级分支，避免把用户改动混入
  平台升级提交；完成后 `rebase main` 再继续。
- 长任务（pio install/run、宿主测试、SIL）一律后台运行，前台只做审查、编辑和提交准备。
- 提交前把参考书解析图片、`.dsh-ops`、测试二进制和生成报告加入 `.gitignore`，避免误提交
  数千张图片/数十 MB 产物。
- 升级提交按“平台版本、固件 API 迁移、宿主测试适配、文档同步”分类，便于回滚。

## 5. 验证记录（随实施更新）

- 现有 19.7.1 基线：`pio run` 主环境 + test 环境均 SUCCESS。
- 2026-10-24 升级后：`pio run` 主环境 SUCCESS（RAM 35.9% / Flash 16.5%），
  test 环境 SUCCESS（RAM 35.7% / Flash 16.5%）。
- 宿主回归 `test_host/run_all.sh` 全绿；GCS pytest 35 passed；
  vector-thrust-lab Node 测试 91 passed；GOV-004 注释门禁 308 文件 100 分、0 error/0 warning；
  `tools/sync-params.py --check` 参数无漂移。
- SIL 首次因 `SPI.h` 缺 `<cstddef>` 失败（已修）；后续 SIL 构建在本机 Git Bash 中
  停在 `g++` 调用后无编译器进程/无输出，已停止该后台任务。**SIL 未完成，不计入通过项**；
  固件双环境构建、宿主回归、GCS/Node/注释门禁均已覆盖本次迁移点。建议在干净终端
  （PowerShell 或另一台机器）单独复跑 `bash test_host/fw_sil/build_and_run.sh`。
- 关键提交：`29e38ec`（平台 20.0.0）、`9e40119`（Uart 迁移）、
  `56103f7`（SIL 桩适配）。

## 6. 回滚方式

```bash
# 在 pio-upgrade-20 分支上回滚本次升级（按提交逆序）
git revert 56103f7 9e40119 29e38ec
# 重新解析 19.7.1 的框架
pio pkg install
```

若只需回退平台版本：把 `TandemVec_FCS/platformio.ini` 两个 env 改回
`platform = ststm32@19.7.1`，再 `pio pkg install`；不要只改代码，Core 3.0 的
`Uart` 在 2.12 中不存在。

## 7. 复现命令

```bash
cd TandemVec_FCS
eval "$(proxy env --shell bash)"     # 仅首次安装框架需要
pio pkg install                       # 下载 4.30000.0
pio run -e TandemVec_FCS              # 主固件
pio run -e test                       # 测试环境

cd ..
bash TandemVec_FCS/test_host/run_all.sh
cd TandemVec_FCS/GCS && py -3.12 -m pytest tests/ -q
cd ../../simulations/vector-thrust-lab && node --test "tests/*.test.mjs"
cd ../.. && py -3.12 tools/check-code-comments.py --check
```
