// 回归测试：与 fixtures/regression-baseline.json 的行为基线比对
// 基线由 tests/fixtures/generate.mjs 从当前实现采集；
// 仅在有意的行为变更后才允许重新生成（提交说明必须记录原因）。
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

for (const name of ['trim', 'pitchStep', 'rollStep']) {
  test(`回归基线：${name}`, () => {
    const actual = SCENARIOS[name]();
    compareTrees(actual, baseline.scenarios[name], name);
  });
}
