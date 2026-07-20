// parameters.mjs 单元测试（生成物完整性与物理合理性）
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { P } from '../src/core/parameters.mjs';

test('全部 51 个参数存在且为有限数', () => {
  const keys = [
    'kT', 'kQ', 'Jp', 'wMax', 'a', 'b', 'tauM',
    'Ix', 'Iy', 'Iz', 'dMax',
    'm', 'g', 'rho', 'Sw', 'cbar', 'bspan',
    'CLa', 'CD0', 'CDk', 'CYb', 'Cm0', 'Cma', 'Cmq', 'Clb', 'Clp', 'Cnb', 'Cnr',
    'sasQ', 'sasR', 'sasP', 'sasTh', 'sasPhi', 'sasI', 'sasIPhi',
    'intThMax', 'intPhiMax', 'dwMax', 'dwUiMax',
    'rateKq', 'rateKr', 'rateKp',
    'maxStep', 'frameCap', 'vMin', 'groundZ',
    'rateQMax', 'ratePMax',
    'aTrim', 'vTrim', 'thrTrim',
  ];
  assert.equal(keys.length, 51);
  for (const k of keys) {
    assert.ok(Number.isFinite(P[k]), `参数 ${k} 缺失或非有限`);
  }
});

test('关键物理量为正', () => {
  for (const k of ['m', 'Ix', 'Iy', 'Iz', 'rho', 'Sw', 'cbar', 'bspan', 'kT', 'kQ', 'Jp', 'wMax', 'tauM', 'maxStep', 'frameCap']) {
    assert.ok(P[k] > 0, `${k} 应为正`);
  }
});

test('阻尼导数符号（静稳定与角速率阻尼）', () => {
  assert.ok(P.Cma < 0 && P.Cmq < 0, '俯仰静稳定/阻尼应为负');
  assert.ok(P.Clb < 0 && P.Clp < 0, '横滚静稳定/阻尼应为负');
  assert.ok(P.Cnb > 0 && P.Cnr < 0, '航向静稳定为正、阻尼为负');
});

test('限幅关系自洽', () => {
  assert.ok(P.dwUiMax < P.dwMax, 'UI 差速幅值应小于 SAS 限幅');
  assert.ok(P.maxStep <= 0.004, '子步长不超过 4 ms');
  assert.ok(P.dMax > 0 && P.dMax < Math.PI / 2, '摆角限幅合理');
});

test('参数对象冻结', () => {
  assert.ok(Object.isFrozen(P));
});

test('摆角限幅与配平角度换算正确', () => {
  assert.ok(Math.abs(P.dMax - 25 * Math.PI / 180) < 1e-15);
  assert.ok(Math.abs(P.aTrim - 1.6 * Math.PI / 180) < 1e-15);
});
