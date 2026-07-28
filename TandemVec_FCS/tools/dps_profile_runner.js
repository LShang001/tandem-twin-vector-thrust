const fs = require("fs");
const path = require("path");
const { spawnSync } = require("child_process");
const { parseProfileFile } = require("./dps_profile_parser");

// DPS310 上板 profile 自动化工具：
// 1. 基于当前 platformio.ini 生成临时 dps_profile 环境；
// 2. 打开 BFS_TASK_PROFILE / BFS_DPS310_DIAG 并按候选参数编译上传；
// 3. 用 PowerShell 串口脚本限时抓取输出，随后调用 parser 汇总。
// 生成文件位于 .pio/，用于本地实测留痕，不纳入 git。
function sanitizeLabel(label) {
  return String(label)
    .trim()
    .replace(/[^A-Za-z0-9_-]+/g, "_")
    .replace(/^_+|_+$/g, "") || "candidate";
}

function formatFloatLiteral(value) {
  const text = Number(value).toString();
  return text.includes(".") ? `${text}f` : `${text}.0f`;
}

function buildProfileConfig(baseConfig, candidate) {
  const intervalMs = Number(candidate.intervalMs);
  const fifoReads = Number(candidate.fifoReads);
  const pressRate = candidate.pressRate || "DPS__MEASUREMENT_RATE_128";
  const tempRate = candidate.tempRate || "DPS__MEASUREMENT_RATE_1";
  const pressOsr = candidate.pressOsr || "DPS__OVERSAMPLING_RATE_2";
  const tempOsr = candidate.tempOsr || "DPS__OVERSAMPLING_RATE_4";
  const measureMode = candidate.measureMode || "both";
  const stopAfterPressure = Boolean(candidate.stopAfterPressure);

  if (!Number.isFinite(intervalMs) || intervalMs <= 0) {
    throw new Error("intervalMs 必须是正数");
  }
  if (!Number.isInteger(fifoReads) || fifoReads <= 0) {
    throw new Error("fifoReads 必须是正整数");
  }
  if (measureMode !== "both" && measureMode !== "pressure") {
    throw new Error("measureMode 必须是 both 或 pressure");
  }

  const pressureOnlyFlag = measureMode === "pressure"
    ? "    -DBFS_DPS310_PRESSURE_ONLY_CONT\n"
    : "";
  const stopAfterPressureFlag = stopAfterPressure
    ? "    -DBFS_DPS310_STOP_AFTER_PRESSURE_SAMPLE\n"
    : "";

  return `${baseConfig.trimEnd()}

[env:dps_profile]
extends = env:TandemVec_FCS
build_flags =
    \${env:TandemVec_FCS.build_flags}
    -DBFS_TASK_PROFILE
    -DBFS_TASK_FRACTIONAL_INTERVALS
    -DBFS_DPS310_DIAG
    -DBFS_PROFILE_TO_ANOCOM_SERIAL
    -DBFS_DPS310_TASK_INTERVAL_MS=${formatFloatLiteral(intervalMs)}
    -DBFS_DPS310_FIFO_MAX_READS=${fifoReads}U
    -DBFS_DPS310_PRESS_MEAS_RATE=${pressRate}
    -DBFS_DPS310_TEMP_MEAS_RATE=${tempRate}
    -DBFS_DPS310_PRESS_OSR=${pressOsr}
    -DBFS_DPS310_TEMP_OSR=${tempOsr}
${stopAfterPressureFlag.trimEnd()}
${pressureOnlyFlag.trimEnd()}
`;
}

function parseArgs(argv) {
  const args = {
    label: "candidate",
    intervalMs: 7.5,
    fifoReads: 1,
    pressRate: "DPS__MEASUREMENT_RATE_128",
    tempRate: "DPS__MEASUREMENT_RATE_1",
    pressOsr: "DPS__OVERSAMPLING_RATE_2",
    tempOsr: "DPS__OVERSAMPLING_RATE_4",
    measureMode: "both",
    port: "COM10",
    baud: 921600,
    durationMs: 45000,
    warmupMs: 8000,
    stopAfterPressure: false,
    skipUpload: false,
  };

  for (let i = 0; i < argv.length; i++) {
    const key = argv[i];
    const value = argv[i + 1];
    switch (key) {
      case "--label":
        args.label = value;
        i++;
        break;
      case "--interval-ms":
        args.intervalMs = Number(value);
        i++;
        break;
      case "--fifo-reads":
        args.fifoReads = Number(value);
        i++;
        break;
      case "--press-rate":
        args.pressRate = value;
        i++;
        break;
      case "--temp-rate":
        args.tempRate = value;
        i++;
        break;
      case "--press-osr":
        args.pressOsr = value;
        i++;
        break;
      case "--temp-osr":
        args.tempOsr = value;
        i++;
        break;
      case "--measure-mode":
        if (value !== "both" && value !== "pressure") {
          throw new Error("measureMode 必须是 both 或 pressure");
        }
        args.measureMode = value;
        i++;
        break;
      case "--stop-after-pressure":
        args.stopAfterPressure = true;
        break;
      case "--port":
        args.port = value;
        i++;
        break;
      case "--baud":
        args.baud = Number(value);
        i++;
        break;
      case "--duration-ms":
        args.durationMs = Number(value);
        i++;
        break;
      case "--warmup-ms":
        args.warmupMs = Number(value);
        i++;
        break;
      case "--skip-upload":
        args.skipUpload = true;
        break;
      default:
        throw new Error(`未知参数: ${key}`);
    }
  }
  return args;
}

function sleepMs(durationMs) {
  if (!Number.isFinite(durationMs) || durationMs <= 0) {
    return;
  }
  Atomics.wait(new Int32Array(new SharedArrayBuffer(4)), 0, 0, durationMs);
}

function runChecked(command, args, options) {
  console.log(`> ${command} ${args.join(" ")}`);
  const result = spawnSync(command, args, {
    stdio: "inherit",
    shell: false,
    ...options,
  });
  if (result.error) {
    throw result.error;
  }
  if (result.status !== 0) {
    throw new Error(`${command} 退出码 ${result.status}`);
  }
}

function captureSerialWithPowerShell(outputPath, port, baud, durationMs) {
  const escapedPath = outputPath.replace(/'/g, "''");
  const escapedPort = port.replace(/'/g, "''");
  const script = `
$ErrorActionPreference = 'Stop'
$portName = '${escapedPort}'
$baud = ${baud}
$durationMs = ${durationMs}
$outPath = '${escapedPath}'
$serial = [System.IO.Ports.SerialPort]::new($portName, $baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 200
$serial.WriteTimeout = 200
$stream = [System.IO.File]::Open($outPath, [System.IO.FileMode]::Create, [System.IO.FileAccess]::Write, [System.IO.FileShare]::Read)
$sw = [System.Diagnostics.Stopwatch]::StartNew()
$bytes = 0
try {
  $serial.Open()
  while ($sw.ElapsedMilliseconds -lt $durationMs) {
    $available = $serial.BytesToRead
    if ($available -gt 0) {
      $buf = New-Object byte[] $available
      $read = $serial.Read($buf, 0, $available)
      if ($read -gt 0) {
        $stream.Write($buf, 0, $read)
        $bytes += $read
      }
    } else {
      Start-Sleep -Milliseconds 20
    }
  }
} finally {
  if ($serial.IsOpen) { $serial.Close() }
  $serial.Dispose()
  $stream.Close()
}
Write-Host "captured_bytes=$bytes"
`;

  runChecked("powershell", ["-NoProfile", "-Command", script], {
    timeout: durationMs + 30000,
  });
}

function timestamp() {
  const now = new Date();
  const pad = (value) => String(value).padStart(2, "0");
  return `${now.getFullYear()}-${pad(now.getMonth() + 1)}-${pad(now.getDate())}_${pad(now.getHours())}${pad(now.getMinutes())}${pad(now.getSeconds())}`;
}

function runProfile(candidate, cwd = process.cwd()) {
  const safeLabel = sanitizeLabel(candidate.label);
  const pioDir = path.join(cwd, ".pio");
  fs.mkdirSync(pioDir, { recursive: true });

  const baseConfigPath = path.join(cwd, "platformio.ini");
  const profileConfigPath = path.join(pioDir, `dps_profile_${safeLabel}.ini`);
  const outputPath = path.join(pioDir, `task_profile_${timestamp()}_${safeLabel}.bin`);

  const baseConfig = fs.readFileSync(baseConfigPath, "utf8");
  fs.writeFileSync(profileConfigPath, buildProfileConfig(baseConfig, candidate), "utf8");

  if (!candidate.skipUpload) {
    runChecked("pio", ["run", "-c", profileConfigPath, "-e", "dps_profile", "-t", "upload"], {
      cwd,
      timeout: 240000,
    });
  }

  if (candidate.warmupMs > 0) {
    console.log(`等待 ${candidate.warmupMs}ms 让启动检测和首个 profile 窗口稳定...`);
    sleepMs(candidate.warmupMs);
  }

  captureSerialWithPowerShell(outputPath, candidate.port, candidate.baud, candidate.durationMs);
  const summary = parseProfileFile(outputPath);
  const result = {
    label: safeLabel,
    config: profileConfigPath,
    capture: outputPath,
    candidate: {
      intervalMs: candidate.intervalMs,
      fifoReads: candidate.fifoReads,
      pressRate: candidate.pressRate,
      tempRate: candidate.tempRate,
      pressOsr: candidate.pressOsr,
      tempOsr: candidate.tempOsr,
      measureMode: candidate.measureMode,
      stopAfterPressure: Boolean(candidate.stopAfterPressure),
      port: candidate.port,
      baud: candidate.baud,
      durationMs: candidate.durationMs,
      warmupMs: candidate.warmupMs,
    },
    summary,
  };
  console.log(JSON.stringify(result, null, 2));
  return result;
}

if (require.main === module) {
  try {
    const args = parseArgs(process.argv.slice(2));
    runProfile(args);
  } catch (error) {
    console.error(error.message);
    process.exit(1);
  }
}

module.exports = {
  buildProfileConfig,
  formatFloatLiteral,
  parseArgs,
  runProfile,
  sanitizeLabel,
};
