/**
 * @file    regression.test.mjs
 * @module  仿真 tests/行为基线
 * @brief   把四个标准场景的轨迹逐点比对回归基线 JSON
 * @details 覆盖：trim（配平直飞 10 s）、pitchStep（8° 俯仰阶跃）、rollStep（滚转阶跃）、
 *          yawStep（8° 偏航阶跃）；逐项比较 t/V/alpha/euler/pqr/pos/T/M。
 *          容差依据：max(1e-9, |x|·1e-9) —— 相对 1e-9 ≈ 数个双精度 ulp 的累积余量，
 *          用于吸收跨 Node 版本数学库（sin/atan2 实现）的末位差异；任何真实行为变化
 *          都远大于该量级。基线只允许在"有意行为变更"后重采（见红线清单）。
 * @theory  N/A（行为锁定）；场景物理背景 @ref MOD-001 §7.3
 * @frame   机体系 FRD / NED
 * @units   混合（见 scenarios.mjs 快照字段）
 * @redline 重新生成基线即触碰行为红线：须重采 tests/fixtures/regression-baseline.json
 *          并在提交说明写明原因（@ref docs/registers/行为保持红线清单.md）
 * @see     tests/scenarios.mjs、tests/fixtures/generate.mjs
 */
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { SCENARIOS } from './scenarios.mjs';

const baseline = JSON.parse(
  readFileSync(new URL('./fixtures/regression-baseline.json', import.meta.url), 'utf-8'));

// 相对/绝对混合容差（跨 Node 版本的数学库差异余量）
function assertClose(actual, expected, path) {
  const tol = Math.max(1e-9, Math.abs(expected) * 1e-9);
  assert.ok(
    Number.isFinite(actual) && Math.abs(actual - expected) <= tol,
    `${path}: ${actual} ≉ ${expected}（容差 ${tol}）`);
}

function compareTrees(actual, expected, path) {
  for (const [k, v] of Object.entries(expected)) {
    const p = path ? `${path}.${k}` : k;
    if (typeof v === 'number') assertClose(actual[k], v, p);
    else compareTrees(actual[k], v, p);
  }
}

for (const name of ['trim', 'pitchStep', 'rollStep', 'yawStep']) {
  test(`回归基线：${name}`, () => {
    const actual = SCENARIOS[name]();
    compareTrees(actual, baseline.scenarios[name], name);
  });
}
