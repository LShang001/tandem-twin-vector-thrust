// ============================================================
//  回归基线生成器：node tests/fixtures/generate.mjs
//  从当前 core 实现采集场景轨迹，写入 regression-baseline.json
//  ⚠️ 仅在有意的行为变更后重新生成，并在提交说明中记录原因
// ============================================================
import { writeFileSync } from 'node:fs';
import { SCENARIOS } from '../scenarios.mjs';

const baseline = {
  meta: {
    model: 'PAR-C0-001 v0.1.0',
    frameDt: 1 / 60,
    substep: 0.004,
    note: '由 core 实现采集的行为基线（配平直飞 / 离地自由飞行俯仰阶跃 / 滚转阶跃）',
  },
  scenarios: {
    trim: SCENARIOS.trim(),
    pitchStep: SCENARIOS.pitchStep(),
    rollStep: SCENARIOS.rollStep(),
  },
};

const url = new URL('./regression-baseline.json', import.meta.url);
writeFileSync(url, JSON.stringify(baseline, null, 2) + '\n', 'utf-8');
console.log('[OK] 回归基线已写入', url.pathname);
