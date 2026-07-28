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
