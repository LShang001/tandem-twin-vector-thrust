/**
 * @file    generate.mjs
 * @module  仿真 tests/基线生成器
 * @brief   从当前 core 实现采集四场景轨迹并写入 regression-baseline.json
 * @details 输入：tests/scenarios.mjs 的场景函数；处理：跑完四场景取整份结果；
 *          输出：fixtures/regression-baseline.json（含 meta 与四个场景）。
 *          不变量：meta 记录模型版本、帧步长与子步长，便于判断基线是否过期。
 * @theory  N/A（工具脚本）；场景物理背景 @ref MOD-001 §7.3
 * @frame   机体系 FRD / NED
 * @units   继承 scenarios.mjs 快照字段
 * @redline 重采基线属行为红线流程：仅在有意的行为变更后运行，并在提交说明记录原因
 * @note 本脚本会覆盖已提交的基线文件——运行前先确认 diff 是否符合预期
 * @see     tests/scenarios.mjs、tests/regression.test.mjs
 */
import { writeFileSync } from 'node:fs';
import { SCENARIOS } from '../scenarios.mjs';

const baseline = {
  meta: {
    model: 'PAR-C0-001 v0.2.0',
    frameDt: 1 / 60,
    substep: 0.004,
    note: '由 core 实现采集的行为基线（配平直飞 / 离地自由飞行俯仰阶跃 / 滚转阶跃 / 偏航阶跃）。2026-08-05 重采：RK4 各阶段用中间状态重算气动（对齐 Python rk4_step），固定翼数值微变（10s 配平末点 ~0.004m）',
  },
  scenarios: {
    trim: SCENARIOS.trim(),
    pitchStep: SCENARIOS.pitchStep(),
    rollStep: SCENARIOS.rollStep(),
    yawStep: SCENARIOS.yawStep(),
  },
};

const url = new URL('./regression-baseline.json', import.meta.url);
writeFileSync(url, JSON.stringify(baseline, null, 2) + '\n', 'utf-8');
console.log('[OK] 回归基线已写入', url.pathname);
