const assert = require("assert");
const { buildProfileConfig, parseArgs, sanitizeLabel } = require("../tools/dps_profile_runner");

const baseConfig = `
[env:TandemVec_FCS]
platform = ststm32
build_flags =
    -O3
    -DNDEBUG
`;

assert.strictEqual(sanitizeLabel("7.0ms/read=2"), "7_0ms_read_2");

const config = buildProfileConfig(baseConfig, {
  intervalMs: 6.5,
  fifoReads: 2,
  pressRate: "DPS__MEASUREMENT_RATE_128",
  tempRate: "DPS__MEASUREMENT_RATE_1",
  pressOsr: "DPS__OVERSAMPLING_RATE_1",
  tempOsr: "DPS__OVERSAMPLING_RATE_4",
});

assert(config.includes("[env:dps_profile]"), "creates profile env");
assert(config.includes("extends = env:TandemVec_FCS"), "extends base env");
assert(config.includes("-DBFS_TASK_PROFILE"), "enables task profiling");
assert(config.includes("-DBFS_TASK_FRACTIONAL_INTERVALS"), "enables fractional tick scheduling for interval sweeps");
assert(config.includes("-DBFS_DPS310_DIAG"), "enables dps diagnostics");
assert(config.includes("-DBFS_PROFILE_TO_ANOCOM_SERIAL"), "routes profile to AnoCom serial");
assert(config.includes("-DBFS_DPS310_TASK_INTERVAL_MS=6.5f"), "sets task interval");
assert(config.includes("-DBFS_DPS310_FIFO_MAX_READS=2U"), "sets fifo read depth");
assert(config.includes("-DBFS_DPS310_PRESS_MEAS_RATE=DPS__MEASUREMENT_RATE_128"), "sets pressure rate");
assert(config.includes("-DBFS_DPS310_TEMP_MEAS_RATE=DPS__MEASUREMENT_RATE_1"), "sets temp rate");
assert(config.includes("-DBFS_DPS310_PRESS_OSR=DPS__OVERSAMPLING_RATE_1"), "sets pressure oversampling");
assert(config.includes("-DBFS_DPS310_TEMP_OSR=DPS__OVERSAMPLING_RATE_4"), "sets temperature oversampling");
assert(!config.includes("BFS_DPS310_READ_MODE_DIRECT"), "does not enable rejected direct read mode");
assert(!config.includes("BFS_DPS310_STOP_AFTER_PRESSURE_SAMPLE"), "defaults to bounded raw FIFO reads");
assert(!config.includes("BFS_DPS310_PRESSURE_ONLY_CONT"), "defaults to combined pressure and temperature mode");

const wholeMsConfig = buildProfileConfig(baseConfig, {
  intervalMs: 7.0,
  fifoReads: 1,
});
assert(wholeMsConfig.includes("-DBFS_DPS310_TASK_INTERVAL_MS=7.0f"), "keeps whole millisecond float literal valid");

const pressureOnlyConfig = buildProfileConfig(baseConfig, {
  intervalMs: 7.5,
  fifoReads: 1,
  measureMode: "pressure",
});
assert(pressureOnlyConfig.includes("-DBFS_DPS310_PRESSURE_ONLY_CONT"), "enables pressure-only continuous mode");

const stopAfterPressureConfig = buildProfileConfig(baseConfig, {
  intervalMs: 7.69,
  fifoReads: 2,
  stopAfterPressure: true,
});
assert(
  stopAfterPressureConfig.includes("-DBFS_DPS310_STOP_AFTER_PRESSURE_SAMPLE"),
  "enables pressure-priority bounded FIFO read",
);

const args = parseArgs([
  "--label", "warmup",
  "--interval-ms", "6.5",
  "--fifo-reads", "2",
  "--press-osr", "DPS__OVERSAMPLING_RATE_1",
  "--temp-osr", "DPS__OVERSAMPLING_RATE_4",
  "--measure-mode", "pressure",
  "--stop-after-pressure",
  "--warmup-ms", "9000",
  "--duration-ms", "30000",
]);

assert.strictEqual(args.label, "warmup");
assert.strictEqual(args.intervalMs, 6.5);
assert.strictEqual(args.fifoReads, 2);
assert.strictEqual(args.pressOsr, "DPS__OVERSAMPLING_RATE_1");
assert.strictEqual(args.tempOsr, "DPS__OVERSAMPLING_RATE_4");
assert.strictEqual(args.measureMode, "pressure");
assert.strictEqual(args.stopAfterPressure, true);
assert.strictEqual(args.warmupMs, 9000);
assert.strictEqual(args.durationMs, 30000);

assert.throws(
  () => parseArgs(["--measure-mode", "invalid"]),
  /measureMode 必须是 both 或 pressure/,
  "rejects unknown measure mode",
);

assert.throws(
  () => parseArgs(["--read-mode", "direct"]),
  /未知参数: --read-mode/,
  "rejects removed direct read-mode option",
);

console.log("dps_profile_runner_test passed");
