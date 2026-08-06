// 交叉对比：py-*.json vs js-*.json（门槛：各字段最大偏差 < 1e-6）
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const HERE = dirname(fileURLToPath(import.meta.url));
const OUT = resolve(HERE, 'cross');
const SCENES = ['vtol', 'vtol_pert', 'vtol_alt', 'vtol_btrue', 'cruise_sas', 'vtol_dw'];
const FIELDS = ['quat', 'omega', 'vel', 'pos'];

let worst = { scene: '', field: '', t: 0, dv: 0 };
let allPass = true;
for (const scene of SCENES) {
  const py = JSON.parse(readFileSync(resolve(OUT, `py-${scene}.json`), 'utf8'));
  const js = JSON.parse(readFileSync(resolve(OUT, `js-${scene}.json`), 'utf8'));
  if (py.length !== js.length) {
    console.log(`[FAIL] ${scene}: 采样点数不一致 py=${py.length} js=${js.length}`);
    allPass = false;
    continue;
  }
  let sceneWorst = { dv: 0, field: '', t: 0, py: 0, js: 0 };
  for (let i = 0; i < py.length; i++) {
    if (Math.abs(py[i].t - js[i].t) > 1e-9) { console.log(`[FAIL] ${scene}: t 不一致 @${i}`); allPass = false; break; }
    for (const f of FIELDS) {
      for (let k = 0; k < py[i][f].length; k++) {
        const dv = Math.abs(py[i][f][k] - js[i][f][k]);
        if (dv > sceneWorst.dv) sceneWorst = { dv, field: `${f}[${k}]`, t: py[i].t, py: py[i][f][k], js: js[i][f][k] };
      }
    }
    for (const f of ['thr', 'dtAct', 'dfAct', 'dwAct', 'intAlt', 'intTh', 'intPhi']) {
      const dv = Math.abs(py[i][f] - js[i][f]);
      if (dv > sceneWorst.dv) sceneWorst = { dv, field: f, t: py[i].t, py: py[i][f], js: js[i][f] };
    }
  }
  const ok = sceneWorst.dv < 1e-6;
  if (!ok) allPass = false;
  if (sceneWorst.dv > worst.dv) worst = { scene, ...sceneWorst };
  console.log(`${ok ? '[PASS]' : '[FAIL]'} ${scene}: 最大偏差 ${sceneWorst.dv.toExponential(2)}` +
    ` (${sceneWorst.field} @t=${sceneWorst.t})`);
}
console.log(allPass ? '\n=== 全部场景交叉一致（<1e-6）===' : `\n=== 最坏点: ${worst.scene} ${worst.field} @t=${worst.t} py=${worst.py} js=${worst.js} dv=${worst.dv.toExponential(2)} ===`);
process.exit(allPass ? 0 : 1);
