const fs = require("fs");

// 解析 BFS_TASK_PROFILE / BFS_DPS310_DIAG 诊断固件输出。
// 输入是串口原始抓取文件，输出为任务耗时、CPU 负载和 DPS310 FIFO 状态摘要。
// 该工具用于复核气压计任务周期，不应把生成的 .bin/.ini 产物提交到仓库。
function avg(values) {
  if (values.length === 0) {
    return 0;
  }
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

function max(values) {
  if (values.length === 0) {
    return 0;
  }
  return Math.max(...values);
}

function sum(values) {
  return values.reduce((total, value) => total + value, 0);
}

function round1(value) {
  return Math.round(value * 10) / 10;
}

function round2(value) {
  return Math.round(value * 100) / 100;
}

function round3(value) {
  return Math.round(value * 1000) / 1000;
}

function parseProfileText(text) {
  const taskPattern = /\[TASK\s+(?<idx>\d+)\s+(?<name>[^\]]+)\]\s+max=(?<max>\d+)us\s+avg=(?<avg>\d+)us\s+cnt=(?<cnt>\d+)/g;
  const loopPattern = /\[LOOP\]\s+cpu=(?<cpu>\d+(?:\.\d+)?)%\s+task0_period=(?<period>\d+)us\s+task0_cnt=(?<cnt>\d+)/g;
  const dpsPattern = /\[DPS310\]\s+ok=(?<ok>\d+)\s+unfin=(?<unfin>\d+)\s+ovf=(?<ovf>\d+)\s+other=(?<other>\d+)\s+flush=(?<flush>\d+)\s+avg_us=(?<avg>\d+)\s+max_us=(?<max>\d+)\s+stale_ms=(?<stale>\d+)\s+status=(?<status>-?\d+)(?:\s+pstd=(?<pstd>\d+(?:\.\d+)?)\s+astd=(?<astd>\d+(?:\.\d+)?))?/g;

  const taskBuckets = new Map();
  for (const match of text.matchAll(taskPattern)) {
    const name = match.groups.name;
    if (!taskBuckets.has(name)) {
      taskBuckets.set(name, []);
    }
    taskBuckets.get(name).push({
      avgUs: Number(match.groups.avg),
      maxUs: Number(match.groups.max),
      cnt: Number(match.groups.cnt),
    });
  }

  const tasks = {};
  for (const [name, rows] of taskBuckets.entries()) {
    tasks[name] = {
      windows: rows.length,
      avgUs: round1(avg(rows.map((row) => row.avgUs))),
      maxUs: max(rows.map((row) => row.maxUs)),
      avgCntPerWindow: round1(avg(rows.map((row) => row.cnt))),
    };
  }

  const loopRows = Array.from(text.matchAll(loopPattern), (match) => ({
    cpu: Number(match.groups.cpu),
    task0PeriodUs: Number(match.groups.period),
    task0Cnt: Number(match.groups.cnt),
  }));

  const dpsRows = Array.from(text.matchAll(dpsPattern), (match) => ({
    ok: Number(match.groups.ok),
    unfinished: Number(match.groups.unfin),
    overflow: Number(match.groups.ovf),
    other: Number(match.groups.other),
    flush: Number(match.groups.flush),
    avgReadUs: Number(match.groups.avg),
    maxReadUs: Number(match.groups.max),
    staleMs: Number(match.groups.stale),
    status: Number(match.groups.status),
    pressureStdPa: match.groups.pstd === undefined ? null : Number(match.groups.pstd),
    rawAltStdM: match.groups.astd === undefined ? null : Number(match.groups.astd),
  }));
  const dpsRowsWithStd = dpsRows.filter((row) => row.pressureStdPa !== null && row.rawAltStdM !== null);

  return {
    loop: {
      count: loopRows.length,
      cpuAvg: round1(avg(loopRows.map((row) => row.cpu))),
      cpuMin: round1(loopRows.length ? Math.min(...loopRows.map((row) => row.cpu)) : 0),
      cpuMax: round1(loopRows.length ? Math.max(...loopRows.map((row) => row.cpu)) : 0),
      task0PeriodUsAvg: round1(avg(loopRows.map((row) => row.task0PeriodUs))),
    },
    tasks,
    dps: {
      windows: dpsRows.length,
      okPerSec: round1(avg(dpsRows.map((row) => row.ok))),
      unfinishedPerSec: round1(avg(dpsRows.map((row) => row.unfinished))),
      overflowTotal: sum(dpsRows.map((row) => row.overflow)),
      otherTotal: sum(dpsRows.map((row) => row.other)),
      flushTotal: sum(dpsRows.map((row) => row.flush)),
      avgReadUs: round1(avg(dpsRows.map((row) => row.avgReadUs))),
      maxReadUs: max(dpsRows.map((row) => row.maxReadUs)),
      staleAvgMs: round1(avg(dpsRows.map((row) => row.staleMs))),
      staleMaxMs: max(dpsRows.map((row) => row.staleMs)),
      pressureStdPaAvg: round2(avg(dpsRowsWithStd.map((row) => row.pressureStdPa))),
      pressureStdPaMax: round2(max(dpsRowsWithStd.map((row) => row.pressureStdPa))),
      rawAltStdMAvg: round3(avg(dpsRowsWithStd.map((row) => row.rawAltStdM))),
      rawAltStdMMax: round3(max(dpsRowsWithStd.map((row) => row.rawAltStdM))),
    },
  };
}

function parseProfileFile(path) {
  const text = fs.readFileSync(path, "latin1");
  return parseProfileText(text);
}

function printSummary(summary) {
  console.log(JSON.stringify(summary, null, 2));
}

if (require.main === module) {
  const path = process.argv[2];
  if (!path) {
    console.error("用法: node tools/dps_profile_parser.js <profile.bin>");
    process.exit(1);
  }
  printSummary(parseProfileFile(path));
}

module.exports = {
  parseProfileText,
  parseProfileFile,
};
