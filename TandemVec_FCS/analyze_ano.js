// AnoCom 姿态数据统计分析脚本
// 用法: node analyze_ano.js [ano_raw.bin]
// 解析固件 AnoComProtocol 发送的二进制帧流 (帧头 0xAB)，统计姿态稳定性 / 漂移 / 异常。
// 协议与 lib/AnoComProtocol 严格对齐：2 字节小端长度、双校验(SC+AC)、0x03 姿态帧 int16×100。
const fs = require('fs');

// ---- 文件读取（带友好报错） ----
const path = process.argv[2] || 'ano_raw.bin';
let buf;
try {
  buf = fs.readFileSync(path);
} catch (e) {
  console.error(`无法读取文件 "${path}": ${e.message}`);
  console.error(`用法: node analyze_ano.js [ano_raw.bin]`);
  process.exit(1);
}
console.log(`读取 ${buf.length} 字节 (${path})`);

// ---- 基础解码函数 ----
// 用乘法而非 <<，避免 4 字节最高位导致的符号问题；les 统一处理有符号
const le  = (d,o,n)=>{let v=0;for(let k=0;k<n;k++)v+=d[o+k]*(2**(8*k));return v;};
const les = (d,o,n)=>{let v=le(d,o,n);const ms=2**(8*n-1);if(v>=ms)v-=2**(8*n);return v;};
// 逐步 &0xFF，与固件 uint8_t 累加行为完全一致
function sumCheck(d,l){let s=0;for(let i=0;i<l;i++)s=(s+d[i])&0xFF;return s;}
function addCheck(d,l){let s=0,a=0;for(let i=0;i<l;i++){s=(s+d[i])&0xFF;a=(a+s)&0xFF;}return a;}

// ---- 数组统计辅助（循环求值，避免 Math.min(...arr) 对大数组栈溢出） ----
function minOf(arr){let m=Infinity;for(const v of arr)if(v<m)m=v;return m;}
function maxOf(arr){let m=-Infinity;for(const v of arr)if(v>m)m=v;return m;}

// ---- 逐帧解码（无帧数上限，分析整个文件） ----
let i = Math.max(0, buf.indexOf(0xAB));
const atts = [];
let totalFrames=0, crcOk=0, crcFail=0, attFail=0;
const funcTally = {};

while (i + 8 <= buf.length) {
  if (buf[i] !== 0xAB) { i++; continue; }
  const func = buf[i+3], len = buf[i+4] | (buf[i+5]<<8);
  // 长度非法或越界：当前位置不是合法帧头，逐字节重新同步
  if (len > 256 || i+8+len > buf.length) { i++; continue; }

  const cr = buf.slice(i, i+6+len);
  const isCrcOk = sumCheck(cr,len+6)===buf[i+6+len] && addCheck(cr,len+6)===buf[i+7+len];

  if (!isCrcOk) {
    // CRC 失败：可能是错位/噪声，不信任 len，逐字节重新同步（而非跳过整帧）
    crcFail++;
    if (func === 0x03) attFail++;
    i++;
    continue;
  }

  // —— CRC 通过，本帧可信 ——
  crcOk++;
  totalFrames++;
  funcTally[func] = (funcTally[func]||0)+1;

  if (func===0x03 && len>=7) {
    const p = buf.slice(i+6, i+6+len);
    const roll_raw  = les(p,0,2);  // int16, x100 deg
    const pitch_raw = les(p,2,2);
    const yaw_raw   = les(p,4,2);
    atts.push({
      idx: totalFrames,
      roll: roll_raw/100, pitch: pitch_raw/100, yaw: yaw_raw/100,
      roll_raw, pitch_raw, yaw_raw,
      fusion: p[6]
    });
  }
  i = i + 8 + len;  // 仅 CRC 通过才信任 len 跳进整帧
}

// ---- 1. 总体统计 ----
console.log(`\n==============================`);
console.log(`  AnoCom 数据统计报告`);
console.log(`==============================`);
console.log(`有效帧: ${totalFrames} (CRC通过 ${crcOk}, CRC失败 ${crcFail})`);
console.log(`姿态帧(0x03): ${atts.length}${attFail?`，另有 ${attFail} 帧 CRC 失败被丢弃`:''}`);
console.log(`功能码分布:`, Object.entries(funcTally).map(([k,v])=>`0x${(+k).toString(16).padStart(2,'0')}=${v}`).join(', '));

if(atts.length < 2) { console.log('\n姿态帧不足，无法分析'); process.exit(0); }

// ---- 2. 数据范围健康检查 ----
// 固件 sendAttitudeEuler 用 (int16_t)(角度×100) 编码，理论限幅 ±327.67°。
// 但 communication.cpp 已将 yaw wrap 到 ±180°，roll/pitch 物理上也在 ±180° 内，
// 因此正常情况下 raw 恒在 ±18000，绝不会触及 int16 限幅。
// 这里检测真正可能发生的异常：raw 超出 ±180° 预期范围（AHRS 异常 / 约定变更 / int16 回绕征兆）。
console.log(`\n--- 数据范围健康检查 ---`);
const RANGE_180 = 18000; // ±180° × 100
const axes = [
  ['roll ', atts.map(a=>a.roll_raw)],
  ['pitch', atts.map(a=>a.pitch_raw)],
  ['yaw  ', atts.map(a=>a.yaw_raw)],
];
let anyOut = false;
for(const [name, raw] of axes){
  const lo = minOf(raw), hi = maxOf(raw);
  let out = 0; for(const v of raw) if(Math.abs(v) > RANGE_180) out++;
  if(out>0) anyOut = true;
  const flag = out>0 ? `⚠️  ${out} 帧超出 ±180°` : '✅';
  console.log(`  ${name} raw: [${lo}, ${hi}]  (${(lo/100).toFixed(2)}° ~ ${(hi/100).toFixed(2)}°)  ${flag}`);
}
if(anyOut) console.log(`  ⚠️  存在超出 ±180° 的样本：固件约定应为 ±180°，请检查 AHRS 输出或 int16 回绕`);
else       console.log(`  ✅ 全部落在 ±180° 预期范围内（int16 限幅 ±327.67° 远未触及）`);

// ---- 3. 姿态稳定性统计 ----
console.log(`\n--- 姿态稳定性 ---`);
const stats = (arr, name) => {
  const min = minOf(arr), max = maxOf(arr);
  const range = max - min;
  let sum=0; for(const v of arr) sum+=v;
  const mean = sum/arr.length;
  let varSum=0; for(const v of arr) varSum+=(v-mean)**2;
  const stddev = Math.sqrt(varSum/arr.length);
  console.log(`  ${name}: 均值=${mean.toFixed(3)}° 标准差=${stddev.toFixed(4)}° 范围=[${min.toFixed(2)}, ${max.toFixed(2)}] 波动=${range.toFixed(3)}°`);
};
stats(atts.map(a=>a.roll), 'roll ');
stats(atts.map(a=>a.pitch), 'pitch');
// yaw 需要处理 wrap-around（±180° 边界）
const yawUnwrapped = [atts[0].yaw];
for(let j=1; j<atts.length; j++){
  let diff = atts[j].yaw - atts[j-1].yaw;
  if(diff > 180) diff -= 360;
  if(diff < -180) diff += 360;
  yawUnwrapped.push(yawUnwrapped[j-1] + diff);
}
stats(yawUnwrapped, 'yaw  ');

// ---- 4. yaw 漂移率 ----
// 姿态帧实际间隔：handleAnoCom @200Hz，4 包分组，0x03 每 4 次发一次 → 50Hz / 20ms
const DT = 0.02;
const dt_total = (atts.length - 1) * DT;
const yaw_drift_total = yawUnwrapped[yawUnwrapped.length-1] - yawUnwrapped[0];
const yaw_drift_rate = yaw_drift_total / dt_total;
console.log(`\n--- yaw 漂移率 ---`);
console.log(`  总变化: ${yaw_drift_total.toFixed(3)}° over ${dt_total.toFixed(1)}s (按 ${atts.length} 帧 × 20ms 估算)`);
console.log(`  漂移率: ${yaw_drift_rate.toFixed(4)}°/s`);
if(attFail>0)
  console.log(`  ⚠️  期间有 ${attFail} 个姿态帧 CRC 失败被丢弃，真实时长可能更长，漂移率为偏大估计`);
if(Math.abs(yaw_drift_rate) < 0.05) console.log(`  ✅ 漂移率极低 (无磁力计静止场景可接受)`);
else if(Math.abs(yaw_drift_rate) < 0.5) console.log(`  ⚠️  轻微漂移 (无磁力计正常范围)`);
else console.log(`  ❌ 显著漂移，零偏估计可能未收敛`);

// ---- 5. yaw 突变检测 ----
// 相邻原始 yaw 跳变 >30°：跨 ±180° 边界的 wrap (|Δ|≈360) 是正常的，其余才是异常
console.log(`\n--- yaw 突变检测 ---`);
let jumps = 0, wraps = 0;
for(let j=1; j<atts.length; j++){
  const diff = atts[j].yaw - atts[j-1].yaw;
  if(Math.abs(diff) > 30) {
    if(Math.abs(Math.abs(diff) - 360) < 30) { wraps++; continue; } // |Δ|≈360 → ±180 边界 wrap
    jumps++;
    console.log(`  帧 ${j}: yaw 从 ${atts[j-1].yaw.toFixed(2)}° 跳到 ${atts[j].yaw.toFixed(2)}° (Δ=${diff.toFixed(2)}°)`);
  }
}
if(wraps>0) console.log(`  ℹ️  ${wraps} 次 ±180° 边界 wrap（正常，已在漂移统计中解缠）`);
if(jumps === 0) console.log(`  ✅ 无异常突变`);
else console.log(`  ⚠️  共 ${jumps} 次异常突变（非 wrap），检查 AHRS 输出或丢帧`);

// ---- 6. 数据趋势（首5帧 + 末5帧） ----
console.log(`\n--- 姿态趋势 (首5帧) ---`);
atts.slice(0,5).forEach(a =>
  console.log(`  t${String(a.idx).padStart(3)}: roll=${a.roll.toFixed(2)} pitch=${a.pitch.toFixed(2)} yaw=${a.yaw.toFixed(2)} (raw: ${a.yaw_raw})`)
);
console.log(`--- 姿态趋势 (末5帧) ---`);
atts.slice(-5).forEach(a =>
  console.log(`  t${String(a.idx).padStart(3)}: roll=${a.roll.toFixed(2)} pitch=${a.pitch.toFixed(2)} yaw=${a.yaw.toFixed(2)} (raw: ${a.yaw_raw})`)
);
