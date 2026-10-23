/**
 * @file    dps_profile_parser_test.js
 * @module  固件/宿主研究（fw-tests：DPS310 profile 文本解析器）
 * @brief   用手写的固件 profile 文本样本校验 parseProfileText 的窗口聚合与字段提取
 * @details 被测：`TandemVec_FCS/tools/dps_profile_parser.js`。
 *          输入：两段采样窗口的原始 profile 文本（[TASK n]、[LOOP]、[DPS310] 行）；
 *          处理：Windows 行尾/混合日志解析 → 按窗口聚合 → 平均数、最大值、每秒计数；
 *          输出：process.exitCode 非零即失败（assert 抛错）。
 *          运行：node test_host/dps_profile_parser_test.js
 * @theory  N/A（日志文本解析，非控制/导航模型）
 * @frame   N/A
 * @units   时间 µs/ms，计数为无量纲，压力 Pa，高度 m，温度 ℃
 * @note    期望值来自样本本身（两窗口 [136,135] → max 136、avg 105.5 等），
 *          改样本或解析口径必须同步改断言；本文件是纯文本测试，不连硬件。
 * @see     TandemVec_FCS/tools/dps_profile_parser.js
 */
const assert = require("assert");
const { parseProfileText } = require("../tools/dps_profile_parser");

const sample = `
[TASK 0 ICM42688] max=140us avg=108us cnt=3940
[TASK 1 DPS310] max=136us avg=106us cnt=267
[TASK 2 Navigation] max=716us avg=564us cnt=400
[LOOP] cpu=35.8% task0_period=507us task0_cnt=3940

[DPS310] ok=130 unfin=4 ovf=0 other=0 flush=0 avg_us=101 max_us=108 stale_ms=0 status=0 pstd=0.42 astd=0.035 raw_alt=0.123 alt=0.120 pressure=100000.00 temp=25.00
[TASK 0 ICM42688] max=141us avg=109us cnt=3941
[TASK 1 DPS310] max=135us avg=105us cnt=266
[TASK 2 Navigation] max=700us avg=560us cnt=400
[LOOP] cpu=35.9% task0_period=507us task0_cnt=3941

[DPS310] ok=129 unfin=5 ovf=0 other=0 flush=0 avg_us=102 max_us=109 stale_ms=7 status=-4 pstd=0.46 astd=0.039 raw_alt=0.123 alt=0.120 pressure=100000.00 temp=25.00
`;

const parsed = parseProfileText(sample);

assert.strictEqual(parsed.loop.count, 2);
assert.strictEqual(parsed.tasks.DPS310.windows, 2);
assert.strictEqual(parsed.tasks.DPS310.maxUs, 136);
assert.strictEqual(parsed.tasks.DPS310.avgUs, 105.5);
assert.strictEqual(parsed.tasks.DPS310.avgCntPerWindow, 266.5);
assert.strictEqual(parsed.dps.windows, 2);
assert.strictEqual(parsed.dps.okPerSec, 129.5);
assert.strictEqual(parsed.dps.unfinishedPerSec, 4.5);
assert.strictEqual(parsed.dps.overflowTotal, 0);
assert.strictEqual(parsed.dps.otherTotal, 0);
assert.strictEqual(parsed.dps.flushTotal, 0);
assert.strictEqual(parsed.dps.avgReadUs, 101.5);
assert.strictEqual(parsed.dps.maxReadUs, 109);
assert.strictEqual(parsed.dps.staleMaxMs, 7);
assert.strictEqual(parsed.dps.pressureStdPaAvg, 0.44);
assert.strictEqual(parsed.dps.pressureStdPaMax, 0.46);
assert.strictEqual(parsed.dps.rawAltStdMAvg, 0.037);
assert.strictEqual(parsed.dps.rawAltStdMMax, 0.039);

console.log("dps_profile_parser_test passed");
